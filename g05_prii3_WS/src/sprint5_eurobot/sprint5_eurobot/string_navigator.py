#!/usr/bin/env python3
"""
NAVEGADOR: Primero alinear, luego recto
1. Calcular ángulo hacia destino
2. Girar robot hasta que mire al destino
3. Avanzar en línea recta
"""
import rclpy, sys, math, json
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class Nav(Node):
    def __init__(self):
        super().__init__('string_navigator')
        self.target_id = int(sys.argv[1]) if len(sys.argv) > 1 else 22
        
        # Esquinas FIJAS
        self.esquinas = {}
        self.esquinas_listas = False
        
        # Robot (posición + orientación)
        self.rx, self.ry = None, None
        self.robot_theta = None  # radianes
        
        # Estado: 'alinear' o 'avanzar'
        self.estado = 'alinear'
        
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        for aid in [20, 21, 22, 23]:
            self.create_subscription(String, f'/overhead_camera/aruco_{aid}',
                lambda m, i=aid: self.cb_esquina(m, i), 10)
        self.create_subscription(String, '/overhead_camera/aruco_3', self.cb_robot, 10)
        
        self.create_timer(0.3, self.nav)
        self.get_logger().info(f"🎯 Objetivo: ArUco {self.target_id}")

    def cb_esquina(self, msg, aid):
        if aid in self.esquinas:
            return
        try:
            d = json.loads(msg.data)
            self.esquinas[aid] = (float(d['px']), float(d['py']))
            self.get_logger().info(f"📍 Esquina {aid}: ({d['px']:.0f}, {d['py']:.0f})")
            if len(self.esquinas) == 4:
                self.esquinas_listas = True
        except: pass

    def cb_robot(self, msg):
        try:
            d = json.loads(msg.data)
            self.rx, self.ry = float(d['px']), float(d['py'])
            self.robot_theta = math.radians(float(d['orientation']))
        except: pass

    def normalizar_angulo(self, ang):
        """Normalizar ángulo a [-pi, pi]"""
        while ang > math.pi: ang -= 2*math.pi
        while ang < -math.pi: ang += 2*math.pi
        return ang

    def nav(self):
        msg = Twist()
        
        if not self.esquinas_listas or self.rx is None or self.robot_theta is None:
            self.get_logger().info(f"⏳ Esq:{len(self.esquinas)}/4 Robot:{'OK' if self.rx else 'NO'}")
            self.cmd.publish(msg)
            return
        
        tx, ty = self.esquinas[self.target_id]
        dx, dy = tx - self.rx, ty - self.ry
        dist = math.sqrt(dx*dx + dy*dy)
        
        # Ángulo hacia el objetivo (en el sistema de coordenadas de la imagen)
        angulo_objetivo = math.atan2(dy, dx)
        
        # Error angular
        error = self.normalizar_angulo(angulo_objetivo - self.robot_theta)
        error_grados = math.degrees(error)
        
        self.get_logger().info(
            f"Robot:({self.rx:.0f},{self.ry:.0f}) θ={math.degrees(self.robot_theta):.0f}° | "
            f"Obj:({tx:.0f},{ty:.0f}) | Dist:{dist:.0f} | Error:{error_grados:.0f}°"
        )
        
        # ¿Llegamos?
        if dist < 50:
            self.get_logger().info("🏁 ¡LLEGAMOS!")
            self.cmd.publish(msg)
            rclpy.shutdown()
            return
        
        # Máquina de estados
        if self.estado == 'alinear':
            if abs(error_grados) < 20:
                self.estado = 'avanzar'
                self.get_logger().info("✅ ALINEADO → Avanzar")
            else:
                # Giro PROPORCIONAL al error (el signo indica la dirección correcta)
                vel_giro = max(-0.25, min(0.25, error * 0.4))  # Proporcional, límite ±0.25
                msg.angular.z = vel_giro
                self.get_logger().info(f"🔄 Giro: {vel_giro:.2f} (error={error_grados:.0f}°)")
        
        elif self.estado == 'avanzar':
            if abs(error_grados) > 35:
                self.estado = 'alinear'
                self.get_logger().info("⚠️ Desalineado → Realinear")
            else:
                vel = min(0.15, max(0.06, dist / 800.0))
                msg.linear.x = vel
                msg.angular.z = error * 0.5  # Corrección proporcional
                self.get_logger().info(f"➡️ Avanzando vel={vel:.2f}")
        
        self.cmd.publish(msg)

def main():
    rclpy.init()
    n = Nav()
    try:
        rclpy.spin(n)
    except:
        pass
    finally:
        n.cmd.publish(Twist())
        n.destroy_node()

if __name__ == '__main__':
    main()
