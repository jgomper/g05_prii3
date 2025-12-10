#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import math
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RealNavigator(Node):
    def __init__(self):
        super().__init__('real_navigator')

        # 1. GESTIÓN DE ARGUMENTOS
        self.declare_parameter('target_id', 22)
        # Intentamos leer de sys.argv si se pasa como argumento posicional, sino usamos parámetro
        if len(sys.argv) > 1 and sys.argv[1].isdigit():
             self.target_id = int(sys.argv[1])
        else:
             self.target_id = self.get_parameter('target_id').value

        self.target_frame = f"aruco_{self.target_id}"
        self.robot_frame = "base_footprint" # Frame base del robot real
        
        self.get_logger().info(f"Iniciando Navegación REAL hacia: {self.target_frame}")

        # 2. CONFIGURACIÓN ROS
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_tf_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz

    def control_loop(self):
        msg = Twist()
        current_time = self.get_clock().now()

        # --- 1. OBTENER TRANSFORMACIÓN ---
        target_visible = False
        distancia = 0.0
        angulo = 0.0

        try:
            # Buscamos la transformación más reciente
            t_nav = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.target_frame,
                rclpy.time.Time())
            
            # Verificamos cuán vieja es la transformación
            # (En ROS 2, lookup_transform sin tiempo específico da la última, pero debemos chequear su stamp)
            tf_time = rclpy.time.Time.from_msg(t_nav.header.stamp)
            time_diff = (current_time - tf_time).nanoseconds / 1e9

            # DEADMAN SWITCH: Si la TF es más vieja de 0.5s, es peligroso
            if time_diff < 0.5:
                x = t_nav.transform.translation.x
                y = t_nav.transform.translation.y
                distancia = math.hypot(x, y)
                angulo = math.atan2(y, x)
                target_visible = True
                self.last_tf_time = current_time
            else:
                self.get_logger().warn(f"TF desactualizada ({time_diff:.2f}s). Parada de emergencia.")

        except TransformException as ex:
            # Es normal si se tapa el marcador momentáneamente
            # self.get_logger().warn(f"No se encuentra transformación: {ex}")
            pass

        # --- 2. LÓGICA DE SEGURIDAD (DEADMAN SWITCH) ---
        # Si no hemos visto el objetivo recientemente (0.5s), PARADA TOTAL
        time_since_last_valid = (current_time - self.last_tf_time).nanoseconds / 1e9
        if time_since_last_valid > 0.5:
            self.get_logger().error("PERDIDA DE OBJETIVO > 0.5s. DETENIENDO ROBOT.")
            self.publisher.publish(msg) # Twist 0,0
            return

        # --- 3. LÓGICA DE CONTROL (TURN & GO) ---
        # Constantes
        KP_ANGULAR = 0.8
        MAX_ANGULAR_VEL = 0.5  # Límite estricto solicitado
        MAX_LINEAR_VEL = 0.15  # Velocidad conservadora para real
        ANGULO_TOLERANCIA_AVANCE = 0.15 # Radianes (~8 grados) - Precisión requerida

        # A. CASO DE LLEGADA
        if distancia < 0.15: # 15 cm de tolerancia
            self.get_logger().info(f"¡Llegada al objetivo {self.target_id}! Deteniendo.")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            # No hacemos exit para mantener el control (frenado activo)
            return

        # B. CÁLCULO DE GIRO
        giro_deseado = KP_ANGULAR * angulo
        # Saturación (Clamp) estricta
        msg.angular.z = max(min(giro_deseado, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)

        # C. MÁQUINA DE ESTADOS IMPLÍCITA (TURN THEN GO)
        if abs(angulo) > ANGULO_TOLERANCIA_AVANCE:
            # FASE 1: GIRO PURO
            # El robot solo gira hasta alinearse
            msg.linear.x = 0.0
            self.get_logger().info(f"GIRANDO: Ang={angulo:.2f} rad")
        else:
            # FASE 2: AVANCE
            # El robot está alineado, avanza hacia el objetivo
            msg.linear.x = MAX_LINEAR_VEL
            # Mantenemos corrección angular suave mientras avanza
            self.get_logger().info(f"AVANZANDO: Dist={distancia:.2f} m")

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
