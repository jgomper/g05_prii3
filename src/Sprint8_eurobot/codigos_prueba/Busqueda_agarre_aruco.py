import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import serial
import time
import os

class MisionAruco(Node):
    def __init__(self):
        super().__init__('mision_aruco_node')
        
        # 1. CONEXIÓN ARDUINO (Ruedas y Ventosa)
        os.system("sudo fuser -k /dev/ttyACM0") # Limpiar puerto
        self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1) 
        
        # 2. CONFIGURACIÓN BRAZO (ROS 2)
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10) 
        
        # POSICIONES (RADIANES)
        self.POS_BUSCADOR = [-0.36201946594121814, 1.1228739367320824, -1.3330293046728292] 
        self.POS_NATURAL  = [0.052155, -0.09664, -1.20724] 
        self.POS_MEDIA    = [0.463262, 1.038505, -2.08621] 
        self.POS_AGARRE   = [1.535515, 0.483204, -1.98190] 

        # DATOS DE CALIBRACIÓN ACTUALIZADOS (Mucho menos estrictos)
        self.TARGET_ID = 36 #
        # El ideal es 43146. Paramos en 38000 para que tenga margen de inercia y no falle.
        self.TARGET_AREA_MIN = 5500.0 
        
        self.get_logger().info("Sistema Iniciado. Preparando posición de búsqueda...")
        self.preparar_camara()

    def enviar_brazo(self, posiciones, segundos):
        msg = JointTrajectory()
        msg.joint_names = ['Junta1', 'Junta2', 'Junta3']
        punto = JointTrajectoryPoint()
        punto.positions = posiciones
        punto.time_from_start = Duration(sec=segundos, nanosec=0)
        msg.points.append(punto)
        self.publisher.publish(msg)
        time.sleep(segundos + 1.0) # Tiempo extra para asegurar la posición física

    def preparar_camara(self):
        self.enviar_brazo(self.POS_BUSCADOR, 2)
        self.get_logger().info(f"Brazo en posición. Listo para buscar ID {self.TARGET_ID}.")

    def control_robot(self, comando):
        """ Comandos: 'F' (Adelante), 'S' (Parar), 'V' (Vacío ON), 'v' (Vacío OFF) """
        self.arduino.write(comando.encode()) 

    def ejecutar_pick_and_place(self):
        self.get_logger().info("--- INICIANDO PICK AND PLACE ---")
        
        # 1. Bajar a por la pieza
        self.enviar_brazo(self.POS_NATURAL, 2)
        self.enviar_brazo(self.POS_MEDIA, 2)
        self.enviar_brazo(self.POS_AGARRE, 2)
        
        # 2. Succionar
        self.control_robot('V') 
        time.sleep(1.0)
        
        # 3. Levantar pieza y llevarla a transporte
        self.enviar_brazo(self.POS_MEDIA, 2)
        self.enviar_brazo(self.POS_NATURAL, 2)
        self.get_logger().info("Pieza recogida. Procediendo a soltar...")

        # 4. Soltar pieza (Faltaba en tu script original)
        self.enviar_brazo(self.POS_MEDIA, 2)
        self.enviar_brazo(self.POS_AGARRE, 2)
        self.control_robot('v') # Apagar vacío
        time.sleep(1.0)

        # 5. Volver a posición inicial
        self.enviar_brazo(self.POS_MEDIA, 2)
        self.enviar_brazo(self.POS_NATURAL, 2)
        self.get_logger().info("--- MISIÓN COMPLETADA ---")

def main():
    rclpy.init()
    mision = MisionAruco()
    
    # Configurar Cámara forzando V4L2 para evitar cuelgues
    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters()
    )

    if not cap.isOpened():
        print("ERROR FATAL: Cámara no disponible.")
        return

    print("\nIniciando movimiento de búsqueda lento...")
    mision.control_robot('F') # Arranca motores

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret: break

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None and mision.TARGET_ID in ids:
                idx = list(ids.flatten()).index(mision.TARGET_ID)
                area = cv2.contourArea(corners[idx])
                
                # LÓGICA DE DETECCIÓN: Menos estricta
                if area >= mision.TARGET_AREA_MIN:
                    print(f"\n¡OBJETIVO ALCANZADO! ID: {mision.TARGET_ID} | Área: {area:.2f}")
                    mision.control_robot('S') # PARAR INMEDIATAMENTE
                    time.sleep(1.0) # Esperar a que se quite la inercia de las ruedas
                    mision.ejecutar_pick_and_place()
                    break
                else:
                    print(f"Buscando... Área actual: {area:.0f} / {mision.TARGET_AREA_MIN} mínimo", end='\r')
            
            rclpy.spin_once(mision, timeout_sec=0.01)

    except KeyboardInterrupt:
        mision.control_robot('S')
    finally:
        cap.release()
        mision.control_robot('v') # Apagar ventosa por seguridad
        mision.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
