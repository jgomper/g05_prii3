import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import time
import sys

class CapturadorAruco(Node):
    def __init__(self):
        super().__init__('capturador_aruco_node')
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        # Tu posición de cámara para orientar bien
        self.POS_CAMARA = [0.463262, 1.038505, -2.08621]

    def preparar_brazo(self):
        self.get_logger().info("Colocando brazo en posición de cámara...")
        msg = JointTrajectory()
        msg.joint_names = ['Junta1', 'Junta2', 'Junta3']
        punto = JointTrajectoryPoint()
        punto.positions = self.POS_CAMARA
        punto.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(punto)
        self.publisher.publish(msg)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(3.0)

def main():
    rclpy.init()
    nodo_brazo = CapturadorAruco()
    nodo_brazo.preparar_brazo()

    # --- CONFIGURACIÓN ARUCO (Basada en el script de tu amigo) ---
    ARUCO_DICT = cv2.aruco.DICT_4X4_50
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    if not cap.isOpened():
        print("Error: No se pudo abrir la cámara USB")
        return

    print("\n" + "="*45)
    print(" BUSCANDO DATOS (Pon la pieza en el sitio ideal)")
    print("="*45)

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                for i in range(len(ids)):
                    # 1. Calcular Centro X (como el script de tu amigo)
                    c = corners[i][0]
                    centro_x = int(c[:, 0].mean())
                    
                    # 2. Calcular Área (esto nos dará la distancia)
                    area = cv2.contourArea(corners[i])
                    
                    # Imprimir en la misma línea para que sea fácil de leer
                    sys.stdout.write(f"\rID: {ids[i][0]} | CENTRO X: {centro_x} | ÁREA: {area:.2f}          ")
                    sys.stdout.flush()

            # Procesar un poco de ROS para mantener el nodo vivo
            rclpy.spin_once(nodo_brazo, timeout_sec=0.01)

    except KeyboardInterrupt:
        print("\n\nCaptura detenida.")
    finally:
        cap.release()
        nodo_brazo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()