#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
import math
import threading
import time

class RealNavigatorJSON(Node):
    def __init__(self):
        super().__init__('real_navigator')

        # Parámetros
        self.declare_parameter('target_id', 22)
        self.declare_parameter('robot_id', 3)
        self.declare_parameter('orientation_offset', -1.57) # -90 grados (Green/Y is Up in Image)

        self.target_id = self.get_parameter('target_id').value
        self.robot_id = self.get_parameter('robot_id').value
        self.offset = self.get_parameter('orientation_offset').value

        self.listen_ids = [self.robot_id, self.target_id]
        self.raw = {}
        self.lock = threading.Lock()

        for tid in self.listen_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            self.create_subscription(String, topic, 
                lambda msg, tid=tid: self.aruco_cb(msg, tid), 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"--- NAVEGADOR JSON DEGUB (I3L7) ---")
        self.get_logger().info(f"Offset: {self.offset:.2f} rad")

        # Constantes de Control (Ajustadas para 1 Hz - LENTAS compaginando lag)
        self.TOL_DIST = 20.0  # MÁS PRECISO (Antes 40.0)
        self.TOL_ANG = 0.15 
        self.KP_ANG = 0.6     # Un poco más reactivo
        self.KP_LIN = 0.002
        self.MAX_W = 0.4      
        self.MAX_V = 0.15
        self.MIN_W = 0.08     # SUAVIZADO (Antes 0.15) para evitar "baile"
        self.DATA_TIMEOUT = 2.0

    def aruco_cb(self, msg, tid):
        try:
            data = json.loads(msg.data)
            with self.lock:
                # --- AUTO-DETECTAR UNIDADES (Deg vs Rad) ---
                raw_angle = float(data.get('orientation', 0.0))
                # Si es > 2*PI (aprox 7), asumimos GRADOS y convertimos a RADIANES
                orientation = math.radians(raw_angle) if abs(raw_angle) > 7.0 else raw_angle

                self.raw[tid] = {
                    'px': float(data.get('px', 0.0)),
                    'py': float(data.get('py', 0.0)),
                    'orientation': orientation,
                    'last_seen': time.time()
                }
        except: pass

    def control_loop(self):
        msg = Twist()
        
        with self.lock:
            robot = self.raw.get(self.robot_id)
            target = self.raw.get(self.target_id)
            
        if not robot or not target:
            self.get_logger().warn(f"Esperando datos... R:{'SI' if robot else 'NO'} T:{'SI' if target else 'NO'}", throttle_duration_sec=1.0)
            self.pub.publish(msg)
            return

        # --- PULSE CONTROL (Anti-Overshoot para 1 Hz) ---
        # Si el dato tiene más de 0.3s, asumimos que ya hemos reaccionado a él y PARAMOS.
        # Esto evita estar 1 segundo girando a ciegas.
        time_since_update = time.time() - robot['last_seen']
        if time_since_update > 0.3:
             # Mandar ceros para frenar
             self.pub.publish(msg) 
             return

        dx = target['px'] - robot['px']
        dy = target['py'] - robot['py']
        dist = math.hypot(dx, dy)
        
        # --- TOLERANCIA DINAMICA (Compensar perspectiva) ---
        # Si el destino está "arriba" (py < 360), los píxeles valen más -> Ser exigentes (12px)
        # Si el destino está "abajo" (py > 360), los píxeles valen menos -> Ser normales (20px)
        current_tol = 12.0 if target['py'] < 360 else 20.0
        
        target_angle = math.atan2(dy, dx)
        robot_angle = robot['orientation'] + self.offset
        
        error_angle = target_angle - robot_angle
        error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))

        # DEBUG EN GRADOS
        deg_error = math.degrees(error_angle)
        deg_robot = math.degrees(robot_angle)
        deg_target = math.degrees(target_angle)
        
        self.get_logger().info(
            f"D:{dist:.0f}px (Tol:{current_tol:.0f}) | Err:{deg_error:.0f}° | R:{deg_robot:.0f}° -> T:{deg_target:.0f}°", 
            throttle_duration_sec=0.5
        )

        if dist < current_tol:
            self.get_logger().info("✅ LLEGADA")
            self.pub.publish(msg)
            return

        # FSM con Mínimos
        if abs(error_angle) > self.TOL_ANG:
             # Giro Puro (Signo Invertido por sistema de coordenadas Y-Down vs Z-Up)
             w = -1 * error_angle * self.KP_ANG
             # Aplicar mínimo
             if abs(w) < self.MIN_W:
                 w = self.MIN_W if w > 0 else -self.MIN_W
             
             msg.angular.z = w
             msg.linear.x = 0.0
        else:
             # Avanzar
             msg.linear.x = dist * self.KP_LIN
             msg.angular.z = -1 * error_angle * self.KP_ANG
             
        # Saturación final
        msg.angular.z = max(min(msg.angular.z, self.MAX_W), -self.MAX_W)
        msg.linear.x = max(min(msg.linear.x, self.MAX_V), 0.0)
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RealNavigatorJSON()
    try:
        rclpy.spin(node)
    except: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
