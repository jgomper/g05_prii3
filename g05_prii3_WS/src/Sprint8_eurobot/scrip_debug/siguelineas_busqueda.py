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
        except Exception as e:
            self.get_logger().error(f"No se detecto el Arduino: {e}")
            exit()

        # --- 2. CONFIGURACION BRAZO (ROS 2) ---
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.POS_BUSCADOR = [-0.36201946594121814, 1.1228739367320824, -1.3330293046728292]
        self.POS_NATURAL = [0.052155, -0.09664, -1.20724]
        self.POS_MEDIA = [0.463262, 1.038505, -2.08621]
        self.POS_AGARRE = [1.535515, 0.483204, -1.98190]

        # --- 3. PARAMETROS ARUCO ---
        self.TARGET_ID = 36
        self.TARGET_AREA_MIN = 5500.0

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

    def retroceder_ms(self, milisegundos, velocidad=25):
        self.enviar_velocidad(-velocidad, -velocidad)
        time.sleep(milisegundos / 1000.0)
        self.parar_motores()

    def avanzar_ms(self, milisegundos, velocidad=25):
        self.enviar_velocidad(velocidad, velocidad)
        time.sleep(milisegundos / 1000.0)
        self.parar_motores()

    def girar_ms(self, direccion, milisegundos, velocidad=25):
        if direccion == "izquierda":
            self.enviar_velocidad(-velocidad, velocidad)
        else:
            self.enviar_velocidad(velocidad, -velocidad)
        time.sleep(milisegundos / 1000.0)
        self.parar_motores()

    def activar_ventosa(self):
        self.arduino.write(b'V')

    def desactivar_ventosa(self):
        self.arduino.write(b'v')

    def ejecutar_agarre(self):
        self.get_logger().info("--- INICIANDO AGARRE ---")
        self.mover_brazo(self.POS_NATURAL, 2)
        self.mover_brazo(self.POS_MEDIA, 2)
        self.mover_brazo(self.POS_AGARRE, 2)
        self.activar_ventosa()
        time.sleep(1.0)
        self.mover_brazo(self.POS_MEDIA, 2)
        self.mover_brazo(self.POS_NATURAL, 2)
        self.get_logger().info("Pieza recogida.")

    def soltar_pieza(self):
        self.get_logger().info("--- SOLTANDO PIEZA ---")
        self.mover_brazo(self.POS_MEDIA, 2)
        self.mover_brazo(self.POS_AGARRE, 2)
        self.desactivar_ventosa()
        time.sleep(1.0)
        self.mover_brazo(self.POS_MEDIA, 2)
        self.mover_brazo(self.POS_NATURAL, 2)
        self.get_logger().info("--- MISION COMPLETADA ---")

    def detectar_almacen_verde(self, frame_bgr):
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.GREEN_HSV_MIN, self.GREEN_HSV_MAX)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return any(cv2.contourArea(c) >= self.GREEN_AREA_MIN for c in contours)

    def detectar_bifurcacion(self, thresh, centro_pantalla):
        height, width = thresh.shape
        y_start = max(0, int(height * 0.25))
        y_end = max(y_start + 1, int(height * 0.55))
        band = thresh[y_start:y_end, :]
        left_band = band[:, 0:max(0, centro_pantalla - 20)]
        right_band = band[:, min(width, centro_pantalla + 20):width]

        left_count = int(np.count_nonzero(left_band))
        right_count = int(np.count_nonzero(right_band))

        left_area = max(1, left_band.size)
        right_area = max(1, right_band.size)

        left_ratio = left_count / left_area
        right_ratio = right_count / right_area

        col_hits = np.count_nonzero(band, axis=0)
        span_cols = int(np.count_nonzero(col_hits >= max(1, int(band.shape[0] * 0.2))))
        span_ratio = span_cols / max(1, band.shape[1])

        center_band = band[:, max(0, centro_pantalla - 3):min(width, centro_pantalla + 4)]
        center_ratio = np.count_nonzero(center_band) / max(1, center_band.size)

        if span_ratio < 0.45 or center_ratio < 0.12:
            return None
        if left_ratio >= right_ratio:
            return "izquierda"
        return "derecha"


def main():
    rclpy.init()
    robot = RobotSiguelineasBusqueda()

    print("\n[INFO] Moviendo brazo a posicion de busqueda...")
    robot.mover_brazo(robot.POS_BUSCADOR, 2)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("ERROR FATAL: Camara no disponible.")
        return

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters()
    )

    VEL_BASE = 35
    KP = 0.4
    carrying = False
    carrying_turn_done = False
    green_seen_count = 0
    green_current = False
    green_lock_until = 0.0
    TURN_SPEED = 30
    TURN_MS = 600

    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                break

            frame_full = frame

            # --- SIGUELINEAS (MISMA VISION, SIN CAMBIOS) ---
            frame = cv2.resize(frame, (320, 240))
            roi = frame[120:240, 10:310]
            height, width, _ = roi.shape
            centro_pantalla = width // 2

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, thresh = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

            kernel = np.ones((5, 5), np.uint8)
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

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
                v_izq, v_der = robot.enviar_velocidad(VEL_BASE + ajuste, VEL_BASE - ajuste)
                print(f"Error: {error:4d} | Izq: {v_izq:3d} | Der: {v_der:3d}      ", end='\r')
            else:
                print("Linea perdida -> Parando motores                           ", end='\r')
                robot.parar_motores()

            cv2.imshow("Vision Siguelineas", roi)
            cv2.imshow("Filtro Estricto B/N", thresh)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rclpy.spin_once(robot, timeout_sec=0.01)

            if carrying and not carrying_turn_done:
                hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                green_mask = cv2.inRange(hsv_roi, robot.GREEN_HSV_MIN, robot.GREEN_HSV_MAX)
                kernel_green = np.ones((5, 5), np.uint8)
                green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel_green)
                thresh_bif = thresh.copy()
                thresh_bif[green_mask > 0] = 0

                direccion = robot.detectar_bifurcacion(thresh_bif, centro_pantalla)
                if direccion:
                    print(f"\nBifurcacion detectada: {direccion} -> girando")
                    robot.parar_motores()
                    time.sleep(0.2)
                    robot.girar_ms(direccion, TURN_MS, velocidad=TURN_SPEED)
                    carrying_turn_done = True

            if carrying and carrying_turn_done:
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
                    robot.avanzar_ms(2000, velocidad=25)
                    robot.soltar_pieza()
                    robot.parar_motores()
                    break

            # --- DETECCION ARUCO (NO TOCAR LOGICA DE LINEA) ---
            if not carrying:
                gray_full = cv2.cvtColor(frame_full, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = detector.detectMarkers(gray_full)
                if ids is not None and robot.TARGET_ID in ids:
                    idx = list(ids.flatten()).index(robot.TARGET_ID)
                    area = cv2.contourArea(corners[idx])
                    if area >= robot.TARGET_AREA_MIN:
                        print(f"\nOBJETIVO ALCANZADO ID {robot.TARGET_ID} | Area: {area:.2f}")
                        robot.parar_motores()
                        time.sleep(1.0)
                        robot.retroceder_ms(400, velocidad=25)
                        robot.ejecutar_agarre()
                        robot.mover_brazo(robot.POS_BUSCADOR, 2)
                        carrying = True
                        carrying_turn_done = False
                        green_seen_count = 0
                        green_current = False
                        green_lock_until = 0.0

    except KeyboardInterrupt:
        print("\nDeteniendo robot por teclado...")
    finally:
        robot.parar_motores()
        robot.desactivar_ventosa()
        cap.release()
        cv2.destroyAllWindows()
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
