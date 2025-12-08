#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import math
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class VisualNavigator(Node):
    def __init__(self):
        super().__init__('visual_navigator')

        # 1. GESTIÓN DE ARGUMENTOS
        if len(sys.argv) < 2:
            self.get_logger().error("Uso: ros2 run sprint4_eurobot visual_navigator <ID_OBJETIVO>")
            self.get_logger().error("IDs válidos: 20, 21, 22, 23")
            sys.exit(1)

        try:
            self.target_id = int(sys.argv[1])
        except ValueError:
            self.get_logger().error("El ID debe ser un número entero.")
            sys.exit(1)

        if self.target_id not in [20, 21, 22, 23]:
            self.get_logger().error(f"ID {self.target_id} no válido. Debe ser 20, 21, 22 o 23.")
            sys.exit(1)

        self.target_frame = f"aruco_{self.target_id}"
        self.robot_frame = "robot_visual_link" # USAMOS EL LINK VISUAL QUE CREAMOS EN EL LAUNCH
        
        self.get_logger().info(f"Iniciando Navegación Visual hacia: {self.target_frame}")

        # 2. CONFIGURACIÓN ROS
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz

    def control_loop(self):
        msg = Twist()

        # 3. OBTENER TRANSFORMADA (ROBOT -> TARGET)
        try:
            # Queremos las coordenadas del TARGET vistas desde el ROBOT.
            t = self.tf_buffer.lookup_transform(
                self.robot_frame,    # Target Frame (Desde donde miro: Robot)
                self.target_frame,   # Source Frame (Lo que miro: ArUco Objetivo)
                rclpy.time.Time())
        except TransformException:
            self.get_logger().warn(f"Esperando ver marcador {self.target_frame}...", throttle_duration_sec=2.0)
            self.publisher.publish(msg) # Parada
            return

        # 4. CÁLCULOS MATEMÁTICOS EXACTOS
        x = t.transform.translation.x
        y = t.transform.translation.y

        distancia = math.hypot(x, y)
        angulo = math.atan2(y, x)

        self.get_logger().info(f"Dist: {distancia:.2f}m | Ang: {angulo:.2f}rad", throttle_duration_sec=0.5)

        # 5. LÓGICA DE CONTROL "TURN & GO"
        K_ANGULAR = 0.5
        K_LINEAR = 0.3
        
        # CASO DE LLEGADA
        if distancia < 0.25:
            self.get_logger().info(f"¡Llegada al objetivo {self.target_id}! Deteniendo.")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            raise SystemExit # Salida limpia

        # FASE 1: ORIENTARSE (Girar sin avanzar)
        # Si el error angular es grande (> 0.1 rad), solo giramos.
        elif abs(angulo) > 0.1:
            msg.linear.x = 0.0
            msg.angular.z = K_ANGULAR * angulo

        # FASE 2: AVANZAR (Acercarse)
        # Si ya miramos al objetivo (error <= 0.1 rad), avanzamos y corregimos suave.
        else:
            msg.linear.x = K_LINEAR * distancia
            # Limitamos velocidad máxima por seguridad
            msg.linear.x = min(msg.linear.x, 0.3)
            
            msg.angular.z = K_ANGULAR * angulo

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualNavigator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
