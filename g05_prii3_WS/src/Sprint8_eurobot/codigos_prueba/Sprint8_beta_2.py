import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import cv2
import serial
import time
import os
import numpy as np


class RobotSiguelineasBusqueda(Node):
    def __init__(self):
        super().__init__('siguelineas_busqueda_node')

        # --- 1. CONEXION ARDUINO ---
        os.system("sudo fuser -k /dev/ttyACM0")
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            self.arduino.write(b'S') 
        except Exception as e:
            self.get_logger().error(f"No se detecto el Arduino: {e}")
            exit()

        # --- 2. CONFIGURACION BRAZO (ROS 2) ---
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        self.POS_BUSCADOR = [0.13345632854584366, 1.541650691824862, -1.7088545977048113]
        self.POS_AGARRE = [1.535515, 0.483204, -1.98190]

        # --- 3. PARAMETROS ARUCO ---
        self.TARGET_ID = 36
        self.TARGET_AREA_MIN = 23000.0  
        self.TARGET_ERR_X_MIN = 30      
        self.TARGET_ERR_X_MAX = 95      

        # --- 4. PARAMETROS ALMACEN VERDE ---
        self.GREEN_HSV_MIN = (35, 60, 60)
        self.GREEN_HSV_MAX = (85, 255, 255)
        self.GREEN_AREA_MIN = 2000

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

    def avanzar_ms(self, milisegundos, velocidad=25):
        self.enviar_velocidad(velocidad, velocidad)
        time.sleep(milisegundos / 1000.0)
        self.parar_motores()

    def activar_ventosa(self):
        self.arduino.write(b'V')

    def desactivar_ventosa(self):
        self.arduino.write(b'v')

    def ejecutar_agarre(self):
        self.get_logger().info("--- INICIANDO AGARRE ---")
        self.mover_brazo(self.POS_AGARRE, 2)
        self.activar_ventosa()
        time.sleep(1.0)
        self.mover_brazo(self.POS_BUSCADOR, 2)
        self.get_logger().info("Pieza recogida.")

    def soltar_pieza(self):
        self.get_logger().info("--- SOLTANDO PIEZA ---")
        self.mover_brazo(self.POS_AGARRE, 2)
        self.desactivar_ventosa()
        time.sleep(1.0)
        self.mover_brazo(self.POS_BUSCADOR, 2)
        self.get_logger().info("--- MISION COMPLETADA ---")

    def detectar_almacen_verde(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.GREEN_HSV_MIN, self.GREEN_HSV_MAX)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= self.GREEN_AREA_MIN for c in contours)


def main():
    rclpy.init()
    robot = RobotSiguelineasBusqueda()

    print("\n[INFO] Moviendo brazo a posicion de busqueda...")
    robot.mover_brazo(robot.POS_BUSCADOR, 2)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("ERROR FATAL: Camara no disponible.")
        return

    print("[INFO] Purgando sensor de la camara...")
    for _ in range(40):  
        ret, _ = cap.read()
        time.sleep(0.01) 
    print("[INFO] Camara estabilizada. ¡Listo!")

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters()
    )

    VEL_NORMAL = 35 
    VEL_CARGA = 25  
    KP = 0.4
    
    carrying = False
    ha_girado_en_t = False  
    
    green_seen_count = 0
    green_current = False
    green_lock_until = 0.0
    
    interseccion_lock_until = 0.0
    UMBRAL_PIXELES_CRUCE = 400 
    
    frames_procesados = 0 

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret: break

            frames_procesados += 1
            frame_full = frame

            frame = cv2.resize(frame, (320, 240))
            roi = frame[120:240, 10:310]
            height, width, _ = roi.shape
            centro_pantalla = width // 2

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

            kernel = np.ones((5, 5), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

            cruce_detectado_ahora = False

            if carrying and not ha_girado_en_t and frames_procesados >= 10:
                zona_izq = thresh[0:height, 0:40]
                zona_der = thresh[0:height, width-40:width]
                
                pix_izq = cv2.countNonZero(zona_izq)
                pix_der = cv2.countNonZero(zona_der)
                
                color_izq = (0, 0, 255) if pix_izq > UMBRAL_PIXELES_CRUCE else (0, 255, 255)
                color_der = (0, 0, 255) if pix_der > UMBRAL_PIXELES_CRUCE else (0, 255, 255)
                
                cv2.rectangle(roi, (0, 0), (40, height), color_izq, 2)
                cv2.rectangle(roi, (width-40, 0), (width, height), color_der, 2)
                
                if time.time() > interseccion_lock_until:
                    # --- SOLO GIRA A LA DERECHA ---
                    if pix_der > UMBRAL_PIXELES_CRUCE:
                        print(f"\n[INFO] ¡CRUCE DERECHO DETECTADO! (Píxeles: {pix_der})")
                        robot.parar_motores()
                        time.sleep(0.5)
                        
                        # -- SECUENCIA MEJORADA: GIRO DERECHO + AVANCE CORRECTOR --
                        print("[INFO] Ejecutando giro y avance de corrección...")
                        robot.avanzar_ms(300, velocidad=30) 
                        robot.enviar_velocidad(70, -20)     # Giro a la derecha
                        time.sleep(0.55) # Tu tiempo perfecto de giro
                        robot.parar_motores()
                        
                        # ¡AVANCE EXTRA PARA ENCARAR LA LÍNEA!
                        robot.avanzar_ms(250, velocidad=35) 
                        
                        # Purga del buffer
                        print("[INFO] Purgando cámara...")
                        for _ in range(15):
                            cap.read()
                            time.sleep(0.01)
                        
                        ha_girado_en_t = True 
                        green_lock_until = time.monotonic() + 2.0 
                        cruce_detectado_ahora = True
            
            if cruce_detectado_ahora:
                continue

            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contornos_validos = [] 
            
            for c in contours:
                if cv2.contourArea(c) > 200:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        contornos_validos.append((c, cx, cy))

            if contornos_validos:
                contornos_validos.sort(key=lambda item: abs(item[1] - centro_pantalla))
                mejor_contorno, cx, cy = contornos_validos[0]

                cv2.drawContours(roi, [mejor_contorno], -1, (255, 0, 0), 2)
                cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
                cv2.line(roi, (centro_pantalla, 0), (centro_pantalla, height), (0, 255, 0), 1)

                error = cx - centro_pantalla
                ajuste = error * KP

                velocidad_actual = VEL_CARGA if carrying else VEL_NORMAL

                if frames_procesados < 10:
                    robot.parar_motores()
                else:
                    v_izq, v_der = robot.enviar_velocidad(velocidad_actual + ajuste, velocidad_actual - ajuste)
                    print(f"Siguiendo linea -> Err: {error:4d} | Izq: {v_izq:3d} | Der: {v_der:3d}     ", end='\r')
            else:
                robot.parar_motores()

            cv2.imshow("Vision Original", roi)
            cv2.imshow("Filtro Estricto (Cruces)", thresh)
            if cv2.waitKey(1) & 0xFF == ord('q'): break
            rclpy.spin_once(robot, timeout_sec=0.01)

            # --- FASE 3: DETECCIÓN ALMACÉN VERDE (LOGICA DEL ARCHIVO ADJUNTO) ---
            if carrying and ha_girado_en_t:
                detected_green = robot.detectar_almacen_verde(frame_full)
                now = time.monotonic()
                if detected_green and not green_current and now >= green_lock_until:
                    green_seen_count += 1
                    green_lock_until = now + 2.0
                    if green_seen_count == 1:
                        print("Almacen detectado: 1/2 -> siguiendo")
                    else:
                        print(f"Almacen detectado: {green_seen_count}/2", end='\r')
                
                green_current = detected_green
                if green_seen_count >= 2:
                    print("\nAlmacen detectado: 2/2 -> dejando pieza")
                    robot.parar_motores()
                    time.sleep(0.5)
                    # Avance extra de 2000ms antes de soltar extraído de tu código adjunto
                    robot.avanzar_ms(2000, velocidad=25) 
                    robot.soltar_pieza()
                    robot.parar_motores()
                    break

            # --- FASE 1: DETECCION ARUCO ---
            if not carrying and frames_procesados >= 10:
                gray_full = cv2.cvtColor(frame_full, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray_full)
                if ids is not None and robot.TARGET_ID in ids:
                    idx = list(ids.flatten()).index(robot.TARGET_ID)
                    area = cv2.contourArea(corners[idx])
                    esquinas = corners[idx][0]
                    cx_aruco = int((esquinas[0][0] + esquinas[2][0]) / 2)
                    error_x_aruco = cx_aruco - (frame_full.shape[1] // 2)
                    if area >= robot.TARGET_AREA_MIN and (robot.TARGET_ERR_X_MIN <= error_x_aruco <= robot.TARGET_ERR_X_MAX):
                        robot.parar_motores()
                        time.sleep(1.0) 
                        robot.ejecutar_agarre()
                        carrying = True
                        ha_girado_en_t = False
                        interseccion_lock_until = time.time() + 3.0 
    except KeyboardInterrupt:
        pass
    finally:
        robot.parar_motores()
        robot.desactivar_ventosa()
        cap.release()
        cv2.destroyAllWindows()
        robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
