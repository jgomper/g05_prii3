import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import serial
import time
import os
import numpy as np

class RobotSiguelineas(Node):
    def __init__(self):
        super().__init__('siguelineas_node')
        
        # --- 1. CONEXIÓN ARDUINO ---
        os.system("sudo fuser -k /dev/ttyACM0")
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2) 
        except Exception as e:
            self.get_logger().error(f"No se detectó el Arduino: {e}")
            exit()
            
        # --- 2. CONFIGURACIÓN BRAZO (ROS 2) ---
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.POS_BUSCADOR = [-0.36201946594121814, 1.1228739367320824, -1.3330293046728292]

    def mover_brazo(self, posiciones, segundos):
        msg = JointTrajectory()
        msg.joint_names = ['Junta1', 'Junta2', 'Junta3']
        punto = JointTrajectoryPoint()
        punto.positions = posiciones
        punto.time_from_start = Duration(sec=segundos, nanosec=0)
        msg.points.append(punto)
        self.publisher.publish(msg)
        time.sleep(segundos + 1.0)

    def enviar_velocidad(self, vel_izq, vel_der):
        vel_izq = max(-40, min(70, int(vel_izq)))
        vel_der = max(-40, min(70, int(vel_der)))
        comando = f"M {vel_izq} {vel_der}\n"
        self.arduino.write(comando.encode())
        return vel_izq, vel_der

    def parar_motores(self):
        self.arduino.write(b'S')

def main():
    rclpy.init()
    robot = RobotSiguelineas()
    
    print("\n[INFO] Moviendo brazo a posición de búsqueda...")
    robot.mover_brazo(robot.POS_BUSCADOR, 2)
    
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("ERROR FATAL: Cámara no disponible.")
        return

    VEL_BASE = 35  
    KP = 0.4       

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret: break

            frame = cv2.resize(frame, (320, 240))
            roi = frame[120:240, 10:310] 
            height, width, _ = roi.shape
            centro_pantalla = width // 2

            # --- LA SOLUCIÓN: BLANCO Y NEGRO ESTRICTO ---
            # 1. Convertimos a escala de grises puros (0 es negro absoluto, 255 es blanco)
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 2. UMBRAL (THRESHOLD) BRUTAL: 
            # Todo lo que sea más claro de "60" (sombras, hojas, madera) se ignora. 
            # Solo la línea negra (que será menor a 60) pasará el filtro.
            _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

            # Limpiamos ruido
            kernel = np.ones((5,5),np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            contornos_validos = []
            
            for c in contours:
                if cv2.contourArea(c) > 200: # Ignorar motas pequeñas
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        contornos_validos.append((c, cx, cy))

            # --- LÓGICA: EL MÁS CERCANO AL CENTRO ---
            if contornos_validos:
                # Si por algún milagro sobrevive alguna sombra, cogemos SIEMPRE la mancha del centro
                contornos_validos.sort(key=lambda item: abs(item[1] - centro_pantalla))
                
                mejor_contorno, cx, cy = contornos_validos[0]
                
                cv2.drawContours(roi, [mejor_contorno], -1, (255, 0, 0), 2)
                cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(roi, (centro_pantalla, 0), (centro_pantalla, height), (0, 255, 0), 1)

                error = cx - centro_pantalla
                ajuste = error * KP
                v_izq, v_der = robot.enviar_velocidad(VEL_BASE + ajuste, VEL_BASE - ajuste)
                print(f"Error: {error:4d} | Izq: {v_izq:3d} | Der: {v_der:3d}      ", end='\r')
            else:
                print("Línea perdida -> Parando motores                           ", end='\r')
                robot.parar_motores()

            cv2.imshow("Vision Siguelineas", roi)
            cv2.imshow("Filtro Estricto B/N", thresh)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rclpy.spin_once(robot, timeout_sec=0.01)

    except KeyboardInterrupt:
        print("\nDeteniendo robot por teclado...")
    finally:
        robot.parar_motores()
        cap.release()
        cv2.destroyAllWindows()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()