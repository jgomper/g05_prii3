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
        
        # Estado Move & Wait
        self.state = 'IDLE' 
        self.last_move_finish = 0
        self.move_start_time = 0
        self.last_cmd = Twist()

        for tid in self.listen_ids:
            topic = f'/overhead_camera/aruco_{tid}'
            self.create_subscription(String, topic, 
                lambda msg, tid=tid: self.aruco_cb(msg, tid), 10)

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info(f"--- NAVEGADOR JSON DEGUB (I3L7) ---")
        self.get_logger().info(f"Namespace: {self.get_namespace()}")
        self.get_logger().info(f"Publishing to: {self.get_namespace().rstrip('/')}/cmd_vel")
        self.get_logger().info(f"Offset: {self.offset:.2f} rad")

        # Constantes de Control (Ajustadas para 1 Hz - LENTAS compaginando lag)
        self.TOL_DIST = 20.0  # MÁS PRECISO (Antes 40.0)
        self.TOL_ANG = 0.25   # MÁS PERMISIVO (Antes 0.15) -> Menos giros bruscos
        self.KP_ANG = 0.8     # SUAVIZADO (Antes 1.0)
        self.KP_LIN = 0.002
        self.MAX_W = 1.0      # DOBLE (Antes 0.5)
        self.MAX_V = 0.3      # DOBLE (Antes 0.15)
        self.MIN_W = 0.2      # REDUCIDO (Antes 0.3) -> Menor "patada" inicial
        self.DATA_TIMEOUT = 2.0

    def aruco_cb(self, msg, tid):
        try:
            data = json.loads(msg.data)
            with self.lock:
                # --- AUTO-DETECTAR UNIDADES (Deg vs Rad) ---
                raw_angle = float(data.get('orientation', 0.0))
                # Si es > 2*PI (aprox 7), asumimos GRADOS y convertimos a RADIANES
                orientation = math.radians(raw_angle) if abs(raw_angle) > 7.0 else raw_angle

                # SI ES EL ROBOT: Actualizar siempre (lo necesitamos fresco)
                if tid == self.robot_id:
                    self.raw[tid] = {
                        'px': float(data.get('px', 0.0)),
                        'py': float(data.get('py', 0.0)),
                        'orientation': orientation,
                        'last_seen': time.time()
                    }
                
                # SI ES EL TARGET: Actualizar SOLO si no lo tenemos ya (LATCHING / FIJAR)
                elif tid == self.target_id:
                    if tid not in self.raw:
                        self.raw[tid] = {
                            'px': float(data.get('px', 0.0)),
                            'py': float(data.get('py', 0.0)),
                            'orientation': orientation,
                            'last_seen': time.time()
                        }
                        self.get_logger().info(f" TARGET {tid} FIJADO EN: ({self.raw[tid]['px']:.0f}, {self.raw[tid]['py']:.0f})")
                        self.get_logger().info("   (Ignoraremos futuras actualizaciones para evitar ruido al taparlo)")
        except: pass

    def control_loop(self):
        msg = Twist()
        
        with self.lock:
            robot = self.raw.get(self.robot_id)
            target = self.raw.get(self.target_id)
            
        if not robot or not target:
            self.get_logger().warn(f"Esperando datos...", throttle_duration_sec=1.0)
            self.pub.publish(msg)
            return

        # --- ESTADO DE NAVEGACIÓN "MOVE & WAIT" (Blind Mode) ---
        current_time = time.time()
        
        # 1. ESTADO WAITING (Esperar a que la cámara "alcance" a la realidad)
        if self.state == 'WAITING':
            if current_time - self.last_move_finish > 3.0: # 3 segundos de espera
                self.state = 'IDLE' # Ya podemos volver a mirar
                self.get_logger().info(" CÁMARA ACTUALIZADA - LEEYENDO...")
            else:
                self.pub.publish(Twist()) # FRENAR
            return

        # 2. ESTADO MOVING (Moverse a ciegas X tiempo)
        if self.state == 'MOVING':
            if current_time - self.move_start_time > 1.0: # 1.0s de movimiento
                self.state = 'WAITING'
                self.last_move_finish = current_time
                self.pub.publish(Twist()) # FRENAR
                self.get_logger().info(" PARANDO - ESPERANDO 3s...")
            else:
                # Seguir publicando la última velocidad calculada
                self.pub.publish(self.last_cmd)
            return

        # 3. ESTADO IDLE (Calcular nuevo movimiento)
        # Solo entramos aquí si estamos parados y la foto es "reciente" (relativamente)
        
        dx = target['px'] - robot['px']
        dy = target['py'] - robot['py']
        dist = math.hypot(dx, dy)
        
        # --- TOLERANCIA DINAMICA ---
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
            self.get_logger().info(" LLEGADA")
            self.pub.publish(msg)
            return

        # CALCULO DE VELOCIDAD
        if abs(error_angle) > self.TOL_ANG:
             w = -1 * error_angle * self.KP_ANG
             if abs(w) < self.MIN_W: w = self.MIN_W if w > 0 else -self.MIN_W
             msg.angular.z = w
             msg.linear.x = 0.0
        else:
             msg.linear.x = dist * self.KP_LIN
             msg.angular.z = -1 * error_angle * self.KP_ANG
             
        # Saturación final
        msg.angular.z = max(min(msg.angular.z, self.MAX_W), -self.MAX_W)
        msg.linear.x = max(min(msg.linear.x, self.MAX_V), 0.0)
        
        # INICIAR MOVIMIENTO
        self.last_cmd = msg
        self.state = 'MOVING'
        self.move_start_time = current_time
        self.pub.publish(msg)
        self.get_logger().info(f" MOVIENDO (0.5s)...")

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