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

        self.missed_frames = 0 # Para filtro de estabilidad
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz

    def control_loop(self):
        msg = Twist()

        # --- 1. LOCALIZACIÓN (Robot respecto a Origen aruco_0) ---
        loc_str = "LOC [No Visible]"
        try:
            t_loc = self.tf_buffer.lookup_transform(
                "aruco_0",
                "robot_visual_link",
                rclpy.time.Time())
            loc_str = "LOC [X:{:.2f}, Y:{:.2f}]".format(
                t_loc.transform.translation.x,
                t_loc.transform.translation.y)
        except TransformException:
            pass

        # --- 2. NAVEGACIÓN (Robot respecto a Objetivo) ---
        nav_str = "NAV [Buscando objetivo...]"
        target_visible = False
        distancia = 0.0
        angulo = 0.0

        try:
            t_nav = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.target_frame,
                rclpy.time.Time())
            
            x = t_nav.transform.translation.x
            y = t_nav.transform.translation.y
            distancia = math.hypot(x, y)
            angulo = math.atan2(y, x)
            
            nav_str = "NAV [Dist:{:.2f}, Ang:{:.2f}]".format(distancia, angulo)
            target_visible = True
            self.missed_frames = 0
            
        except TransformException:
            self.missed_frames += 1

        # --- 3. LOG UNIFICADO ---
        self.get_logger().info(f"{loc_str} | {nav_str}", throttle_duration_sec=0.5)

        # --- 4. LÓGICA DE CONTROL ---
        if not target_visible:
            if self.missed_frames < 5:
                return # Mantener inercia
            else:
                self.publisher.publish(msg) # Parada total
                return

        # 5. LÓGICA DE CONTROL SUAVE (SMOOTH PROPORTIONAL)
        K_ANGULAR = 0.8
        K_LINEAR_MAX = 0.3
        ANGULO_LIMITE = 0.5

        # CASO DE LLEGADA
        if distancia < 0.20:
            self.get_logger().info(f"¡Llegada al objetivo {self.target_id}! Deteniendo.")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            raise SystemExit

        # VELOCIDAD ANGULAR
        msg.angular.z = K_ANGULAR * angulo
        msg.angular.z = max(min(msg.angular.z, 1.5), -1.5)

        # VELOCIDAD LINEAL
        if abs(angulo) > ANGULO_LIMITE:
            msg.linear.x = 0.0
        else:
            factor_reduccion = 1.0 - (abs(angulo) / ANGULO_LIMITE)
            msg.linear.x = K_LINEAR_MAX * distancia * factor_reduccion
            msg.linear.x = min(msg.linear.x, K_LINEAR_MAX)
            msg.linear.x = max(msg.linear.x, 0.0)

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
