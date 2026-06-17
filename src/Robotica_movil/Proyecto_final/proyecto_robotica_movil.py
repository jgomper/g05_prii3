import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import LaserScan
import cv2
import serial
import time
import os
import numpy as np
import threading
import math
from enum import Enum, auto


class State(Enum):
    """Estados de la máquina de estados finitos del robot."""
    WAIT_START = auto()
    INIT = auto()
    SEARCH_ARUCO = auto()
    ALIGN_GRAB = auto()
    GRAB = auto()
    SEARCH_INTERSECTION = auto()
    TURN_RIGHT = auto()
    SEARCH_GREEN = auto()
    DROP = auto()
    FINISHED = auto()


class RobotSiguelineasFSM(Node):
    def __init__(self):
        super().__init__('siguelineas_fsm_node')

        # --- VARIABLES DE IMU (BNO055) ---
        self.yaw_actual = 0.0
        self.giro_iniciado = False
        self.yaw_inicial = 0.0

        # --- 1. CONEXION ARDUINO ---
        os.system("sudo fuser -k /dev/ttyACM0")
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            self.arduino.write(b'S')
            
            # --- INICIAR EL HILO QUE ESCUCHA LA IMU ---
            self.hilo_lectura = threading.Thread(target=self.leer_serie_arduino, daemon=True)
            self.hilo_lectura.start()
            
        except Exception as e:
            self.get_logger().error(f"No se detecto el Arduino: {e}")
            exit()

        # --- 2. CONFIGURACION BRAZO (ROS 2) ---
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.POS_BUSCADOR = [0.13345632854584366, 1.541650691824862, -1.7088545977048113]
        self.POS_AGARRE = [1.535515, 0.483204, -1.98190]
        self.POS_SOLTAR = [1.5355147686733197, 0.4832039481837702, -1.9819031779484553]

        # --- 3. PARAMETROS ARUCO ---
        self.TARGET_ID = 36
        self.TARGET_AREA_MIN = 23000.0
        self.TARGET_ERR_X_MIN = 30
        self.TARGET_ERR_X_MAX = 95

        # --- 3b. PARAMETROS ALINEACION PRE-AGARRE ---
        self.ALIGN_TARGET_AREA = 24000.0
        self.ALIGN_AREA_TOLERANCE = 500.0
        self.ALIGN_VEL = 15
        self.ALIGN_TIMEOUT = 5.0
        self.align_start_time = 0.0

        # --- 4. PARAMETROS ALMACEN VERDE ---
        self.GREEN_HSV_MIN = (35, 60, 60)
        self.GREEN_HSV_MAX = (85, 255, 255)
        self.GREEN_AREA_MIN = 2000
        self.GREEN_ASPECT_RATIO_MIN = 3.0

        # --- 5. CONSTANTES DE CONTROL ---
        self.VEL_NORMAL = 35
        self.VEL_CARGA = 30
        self.KP = 0.4  
        self.SATURACION_MAX_NEGRO = 90 

        # --- 6. ESTADO FSM ---
        self.state = State.WAIT_START
        self.frames_procesados = 0
        self.green_seen_count = 0
        self.green_current = False
        self.green_lock_until = 0.0
        self.interseccion_lock_until = 0.0
        self.line_lost_count = 0
        self.last_cx = None 

        # --- 7. CAMARA Y DETECTOR ---
        self.cap = None
        self.detector = cv2.aruco.ArucoDetector(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            cv2.aruco.DetectorParameters()
        )

        # --- 8. CONFIGURACIÓN LIDAR 4ROS (ESCUDO ANTI-COLISIONES) ---
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data)
        self.obstaculo_frente = False

    # ================================================================
    # LECTURA DEL LIDAR (TÚNEL VIRTUAL ASIMÉTRICO)
    # ================================================================
    def lidar_callback(self, msg):
        """Escucha el YDLidar 4ROS y crea una caja de colisión recortada por la izquierda."""
        puntos_detectados = 0 
        
        for i, distancia in enumerate(msg.ranges):
            if distancia < 0.05 or distancia > 30.0 or math.isinf(distancia):
                continue

            angulo = msg.angle_min + i * msg.angle_increment

            x = distancia * math.cos(angulo)
            y = distancia * math.sin(angulo)

            # --- TÚNEL VIRTUAL ASIMÉTRICO ---
            # X: Reducido a ~8.3 cm de distancia frontal (0.05 a 0.083 metros)
            # Y: -0.13 (13 cm derecha) hasta 0.03 (solo 3 cm izquierda para ignorar la barrera)
            if 0.05 < x < 0.083 and -0.13 < y < 0.03:
                puntos_detectados += 1
                
                # --- FILTRO ANTI-FANTASMAS ---
                if puntos_detectados >= 4:
                    self.obstaculo_frente = True
                    return 

        self.obstaculo_frente = False

    # ================================================================
    # LECTURA DE IMU EN SEGUNDO PLANO
    # ================================================================
    def leer_serie_arduino(self):
        while True:
            try:
                if self.arduino.in_waiting > 0:
                    linea = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if linea.startswith("Y:"):
                        self.yaw_actual = float(linea.split(":")[1])
            except Exception:
                pass
            time.sleep(0.005)

    # ================================================================
    # MÉTODOS DE HARDWARE
    # ================================================================
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

    def detectar_almacen_verde(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.GREEN_HSV_MIN, self.GREEN_HSV_MAX)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for c in contours:
            area = cv2.contourArea(c)
            if area < self.GREEN_AREA_MIN:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if h == 0:
                continue
            aspect_ratio = w / h
            if aspect_ratio >= self.GREEN_ASPECT_RATIO_MIN:
                return True
        return False

    def purgar_camara(self, n_frames=40):
        for _ in range(n_frames):
            self.cap.read()
            time.sleep(0.01)

    def transition(self, new_state):
        self.get_logger().info(f"FSM: {self.state.name} --> {new_state.name}")
        self.state = new_state

    # ================================================================
    # LÓGICA DE SIGUELÍNEAS 
    # ================================================================
    def seguir_linea(self, roi, thresh, con_carga):
        height, width, _ = roi.shape
        centro_pantalla = width // 2
        OFFSET_X = 40 
        target_x = centro_pantalla - OFFSET_X

        thresh_recortado = thresh.copy()
        corte_y = int(height * 0.4) 
        thresh_recortado[0:corte_y, :] = 0

        contours, _ = cv2.findContours(thresh_recortado, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contornos_validos = []

        for c in contours:
            if cv2.contourArea(c) > 100:
                M = cv2.moments(c)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    contornos_validos.append((c, cx, cy))

        if contornos_validos:
            self.line_lost_count = 0
            contornos_validos.sort(key=lambda item: abs(item[1] - target_x)) 
            mejor_contorno, cx, cy = contornos_validos[0]

            cv2.drawContours(roi, [mejor_contorno], -1, (255, 0, 0), 2)
            cv2.circle(roi, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(roi, (target_x, 0), (target_x, height), (255, 255, 0), 1)

            error = cx - target_x
            ajuste = error * self.KP
            velocidad_actual = self.VEL_CARGA if con_carga else self.VEL_NORMAL

            if self.frames_procesados < 10:
                self.parar_motores()
            else:
                v_izq, v_der = self.enviar_velocidad(velocidad_actual + ajuste, velocidad_actual - ajuste)
        else:
            self.line_lost_count += 1
            if self.state == State.SEARCH_GREEN:
                self.enviar_velocidad(10, 25)
            else:
                if self.line_lost_count <= 25:
                    self.enviar_velocidad(15, -15)
                else:
                    self.enviar_velocidad(-15, 15)
                    if self.line_lost_count >= 50:
                        self.line_lost_count = 0

    # ================================================================
    # HANDLERS DE CADA ESTADO
    # ================================================================
    def _handle_wait_start(self):
        print("\n========================================")
        print("  ROBOT EUROBOT - MÁQUINA DE ESTADOS")
        print("  [+] LIDAR 4ROS ANTI-COLISIONES ACTIVO")
        print("========================================")
        print("Presiona ENTER para iniciar la misión...")
        input()
        self.transition(State.INIT)

    def _handle_init(self):
        print("\n[INIT] Moviendo brazo a posicion de busqueda...")
        self.mover_brazo(self.POS_BUSCADOR, 2)

        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            print("ERROR FATAL: Camara no disponible.")
            self.transition(State.FINISHED)
            return

        print("[INIT] Purgando sensor de la camara...")
        self.purgar_camara(40)
        print("[INIT] Camara estabilizada. ¡Listo!")

        self.frames_procesados = 0
        self.transition(State.SEARCH_ARUCO)

    def _handle_search_aruco(self, frame_full, roi, thresh):
        self.seguir_linea(roi, thresh, con_carga=False)
        if self.frames_procesados >= 10:
            gray_full = cv2.cvtColor(frame_full, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detector.detectMarkers(gray_full)
            if ids is not None and self.TARGET_ID in ids:
                idx = list(ids.flatten()).index(self.TARGET_ID)
                area = cv2.contourArea(corners[idx])
                esquinas = corners[idx][0]
                cx_aruco = int((esquinas[0][0] + esquinas[2][0]) / 2)
                error_x_aruco = cx_aruco - (frame_full.shape[1] // 2)
                
                if area >= self.TARGET_AREA_MIN and abs(error_x_aruco) <= self.TARGET_ERR_X_MAX:
                    self.parar_motores()
                    time.sleep(0.5)
                    self.align_start_time = time.time()
                    self.transition(State.ALIGN_GRAB)

    def _handle_align_grab(self, frame_full):
        if time.time() - self.align_start_time > self.ALIGN_TIMEOUT:
            self.parar_motores()
            time.sleep(0.5)
            self.transition(State.GRAB)
            return

        gray_full = cv2.cvtColor(frame_full, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray_full)

        if ids is None or self.TARGET_ID not in ids:
            self.parar_motores()
            time.sleep(0.5)
            self.transition(State.GRAB)
            return

        idx = list(ids.flatten()).index(self.TARGET_ID)
        area = cv2.contourArea(corners[idx])
        diff_area = area - self.ALIGN_TARGET_AREA

        if abs(diff_area) <= self.ALIGN_AREA_TOLERANCE:
            self.parar_motores()
            time.sleep(0.5)
            self.transition(State.GRAB)
        elif diff_area < 0:
            self.enviar_velocidad(self.ALIGN_VEL, self.ALIGN_VEL)
        else:
            self.enviar_velocidad(-self.ALIGN_VEL, -self.ALIGN_VEL)

    def _handle_grab(self):
        self.get_logger().info("--- INICIANDO AGARRE ---")
        self.mover_brazo(self.POS_AGARRE, 2)
        self.activar_ventosa()
        time.sleep(1.0)
        self.mover_brazo(self.POS_BUSCADOR, 2)
        self.get_logger().info("Pieza recogida.")

        self.interseccion_lock_until = time.time() + 3.0
        self.transition(State.SEARCH_INTERSECTION)

    def _handle_search_intersection(self, frame_full, roi, thresh_linea, thresh_cruce):
        height, width, _ = roi.shape

        if self.frames_procesados >= 10:
            inicio_x = width // 2 + 20
            zona_derecha = thresh_linea[0:height, inicio_x:width] 
            contours_derecha, _ = cv2.findContours(zona_derecha, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            cruce_valido = False
            for c in contours_derecha:
                if cv2.contourArea(c) > 600: 
                    x, y, w, h = cv2.boundingRect(c)
                    
                    es_horizontal = w > h * 2
                    es_fina = h < 80
                    es_larga = w > 65 
                    
                    if es_horizontal and es_fina and es_larga: 
                        cruce_valido = True
                        cv2.rectangle(roi, (inicio_x + x, y), (inicio_x + x + w, y + h), (0, 255, 0), 2)
                        break 
                    else:
                        cv2.rectangle(roi, (inicio_x + x, y), (inicio_x + x + w, y + h), (0, 0, 255), 2)

            cv2.line(roi, (inicio_x, 0), (inicio_x, height), (0, 255, 255), 1)

            if time.time() > self.interseccion_lock_until and cruce_valido:
                self.parar_motores()
                time.sleep(0.5)
                self.transition(State.TURN_RIGHT)
                return

        self.seguir_linea(roi, thresh_linea, con_carga=True)

    def _handle_turn_right(self):
        if not self.giro_iniciado:
            self.yaw_inicial = self.yaw_actual
            self.enviar_velocidad(50, -30)
            self.giro_iniciado = True

        diferencia = abs(self.yaw_actual - self.yaw_inicial)
        if diferencia > 180.0:
            diferencia = 360.0 - diferencia
        
        grados_girados = diferencia

        if grados_girados >= 40.0: 
            self.parar_motores()
            self.avanzar_ms(250, velocidad=35)
            self.purgar_camara(15)

            self.giro_iniciado = False
            self.green_lock_until = time.monotonic() + 2.0
            self.transition(State.SEARCH_GREEN)
            
        time.sleep(0.01) 

    def _handle_search_green(self, frame_full, roi, thresh):
        self.seguir_linea(roi, thresh, con_carga=True)
        detected_green = self.detectar_almacen_verde(frame_full)
        now = time.monotonic()

        if detected_green and not self.green_current and now >= self.green_lock_until:
            self.green_seen_count += 1
            self.green_lock_until = now + 3.0

        self.green_current = detected_green

        if self.green_seen_count >= 2:
            self.parar_motores()
            time.sleep(0.5)
            self.transition(State.DROP)

    def _handle_drop(self):
        self.get_logger().info("--- AVANZANDO AL ALMACÉN ---")
        self.avanzar_ms(2000, velocidad=25)
        
        self.get_logger().info("--- SOLTANDO PIEZA ---")
        self.mover_brazo(self.POS_SOLTAR, 2)
        self.desactivar_ventosa()
        time.sleep(1.0)
        self.mover_brazo(self.POS_BUSCADOR, 2)
        
        self.get_logger().info("--- MISION COMPLETADA ---")
        self.parar_motores()
        self.transition(State.FINISHED)

    # ================================================================
    # BUCLE PRINCIPAL DE LA FSM
    # ================================================================
    def run(self):
        try:
            while rclpy.ok() and self.state in (State.WAIT_START, State.INIT):
                if self.state == State.WAIT_START:
                    self._handle_wait_start()
                elif self.state == State.INIT:
                    self._handle_init()

            while rclpy.ok() and self.state != State.FINISHED:

                # ====================================================
                # ¡AQUÍ ENTRA EL ESCUDO ANTI-COLISIONES DEL LIDAR!
                # ====================================================
                if self.obstaculo_frente:
                    print("\n[🚨 LIDAR] ¡RIVAL DETECTADO! Frenando motores temporalmente...     ", end='\r')
                    self.parar_motores()
                    rclpy.spin_once(self, timeout_sec=0.01)
                    continue # Detenemos la cámara y la FSM hasta que el rival se vaya

                # Si no hay obstáculo, informamos de que la ruta está libre
                print(f"[{self.state.name}] Ruta libre. Conduciendo... Yaw: {self.yaw_actual:.1f}º    ", end='\r')


                if self.state == State.GRAB:
                    self._handle_grab()
                    continue
                elif self.state == State.TURN_RIGHT:
                    self._handle_turn_right()
                    continue
                elif self.state == State.DROP:
                    self._handle_drop()
                    continue

                ret, frame = self.cap.read()
                if not ret:
                    break

                self.frames_procesados += 1
                frame_full = frame
                frame_resized = cv2.resize(frame, (320, 240))
                roi = frame_resized[120:240, 10:310]

                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5, 5), 0)
                _, thresh_linea = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)
                kernel = np.ones((5, 5), np.uint8)
                thresh_linea = cv2.morphologyEx(thresh_linea, cv2.MORPH_OPEN, kernel)

                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                _, s_channel, _ = cv2.split(hsv_roi)
                _, mask_s_baja = cv2.threshold(s_channel, self.SATURACION_MAX_NEGRO, 255, cv2.THRESH_BINARY_INV)
                thresh_cruce = cv2.bitwise_and(thresh_linea, mask_s_baja)

                if self.state == State.SEARCH_ARUCO:
                    self._handle_search_aruco(frame_full, roi, thresh_linea)
                elif self.state == State.ALIGN_GRAB:
                    self._handle_align_grab(frame_full)
                elif self.state == State.SEARCH_INTERSECTION:
                    self._handle_search_intersection(frame_full, roi, thresh_linea, thresh_cruce)
                elif self.state == State.SEARCH_GREEN:
                    self._handle_search_green(frame_full, roi, thresh_linea)

                cv2.imshow("Vision Original", roi)
                cv2.imshow("Filtro Linea Normal", thresh_linea)
                cv2.imshow("Filtro T (Solo Negro)", thresh_cruce)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                rclpy.spin_once(self, timeout_sec=0.01)

        except KeyboardInterrupt:
            pass
        finally:
            self.get_logger().info(f"FSM finalizada en estado: {self.state.name}")
            self.parar_motores()
            self.desactivar_ventosa()
            if self.cap is not None:
                self.cap.release()
            cv2.destroyAllWindows()
            self.destroy_node()
            rclpy.shutdown()

def main():
    rclpy.init()
    robot = RobotSiguelineasFSM()
    robot.run()

if __name__ == '__main__':
    main()
