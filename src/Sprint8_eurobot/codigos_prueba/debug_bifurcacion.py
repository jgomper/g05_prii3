import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time
import serial
import os


def detectar_bifurcacion_robusta(thresh, centro_pantalla):
    height, width = thresh.shape
    direccion = None

    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, thresh

    contornos_validos = [c for c in contours if cv2.contourArea(c) > 200]
    if not contornos_validos:
        return None, thresh

    contornos_validos.sort(
        key=lambda c: abs(
            (cv2.moments(c)["m10"] / max(cv2.moments(c)["m00"], 1)) - centro_pantalla
        )
    )
    main_contour = contornos_validos[0]

    clean_mask = np.zeros_like(thresh)
    cv2.drawContours(clean_mask, [main_contour], -1, 255, thickness=cv2.FILLED)

    y_start = int(height * 0.25)
    y_end = int(height * 0.55)
    band = clean_mask[y_start:y_end, :]

    if np.count_nonzero(band) == 0:
        return None, clean_mask

    col_hits = np.sum(band, axis=0)
    if np.max(col_hits) == 0:
        return None, clean_mask

    main_col = int(np.argmax(col_hits))

    min_extension = 40
    max_left_ext = 0
    max_right_ext = 0

    for r in range(band.shape[0]):
        row = band[r, :]
        if row[main_col] > 0:
            left_x = main_col
            while left_x > 0 and row[left_x - 1] > 0:
                left_x -= 1
            left_ext = main_col - left_x

            right_x = main_col
            while right_x < width - 1 and row[right_x + 1] > 0:
                right_x += 1
            right_ext = right_x - main_col

            if left_ext > max_left_ext:
                max_left_ext = left_ext
            if right_ext > max_right_ext:
                max_right_ext = right_ext

    if max_left_ext > min_extension and max_left_ext > max_right_ext:
        direccion = "izquierda"
    elif max_right_ext > min_extension and max_right_ext > max_left_ext:
        direccion = "derecha"

    return direccion, clean_mask


def main():
    rclpy.init()
    node = Node('debug_bifurcacion_node')
    publisher = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

    os.system("sudo fuser -k /dev/ttyACM0")
    try:
        arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
    except Exception as e:
        print(f"No se detecto el Arduino: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    pos_buscador = [0.463262, 1.038505, -2.08621]
    msg = JointTrajectory()
    msg.joint_names = ['Junta1', 'Junta2', 'Junta3']
    punto = JointTrajectoryPoint()
    punto.positions = pos_buscador
    punto.time_from_start = Duration(sec=2, nanosec=0)
    msg.points.append(punto)
    publisher.publish(msg)
    time.sleep(3.0)

    cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
    if not cap.isOpened():
        print("ERROR FATAL: Camara no disponible.")
        node.destroy_node()
        rclpy.shutdown()
        return

    vel_base = 35
    kp = 0.4
    turn_speed = 30
    turn_ms = 600

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            frame = cv2.resize(frame, (320, 240))
            roi = frame[120:240, 10:310]
            centro_pantalla = roi.shape[1] // 2

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            _, dark_mask = cv2.threshold(blur, 60, 255, cv2.THRESH_BINARY_INV)

            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_green = np.array([30, 40, 40])
            upper_green = np.array([90, 255, 255])
            green_mask = cv2.inRange(hsv, lower_green, upper_green)

            kernel_dilate = np.ones((7, 7), np.uint8)
            green_mask_fat = cv2.dilate(green_mask, kernel_dilate, iterations=2)

            safe_thresh = cv2.bitwise_and(dark_mask, cv2.bitwise_not(green_mask_fat))

            kernel = np.ones((5, 5), np.uint8)
            safe_thresh = cv2.morphologyEx(safe_thresh, cv2.MORPH_OPEN, kernel)

            direccion, clean_mask = detectar_bifurcacion_robusta(safe_thresh, centro_pantalla)

            cv2.imshow("1. ROI Original", roi)
            cv2.imshow("2. Filtro Oscuro (Con error verde)", dark_mask)
            cv2.imshow("3. Escudo Verde Engordado", green_mask_fat)
            cv2.imshow("4. Safe Thresh (Resta)", safe_thresh)
            cv2.imshow("5. Linea Principal Aislada (Clean)", clean_mask)

            if direccion:
                print(f"T DETECTADA -> {direccion}               ", end='\r')
                arduino.write(b'S')
                time.sleep(0.2)
                if direccion == "izquierda":
                    arduino.write(f"M {-turn_speed} {turn_speed}\n".encode())
                else:
                    arduino.write(f"M {turn_speed} {-turn_speed}\n".encode())
                time.sleep(turn_ms / 1000.0)
                arduino.write(b'S')
                time.sleep(0.2)
            else:
                contornos_validos = []
                for c in cv2.findContours(clean_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]:
                    if cv2.contourArea(c) > 200:
                        M = cv2.moments(c)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            contornos_validos.append((c, cx, cy))

                if contornos_validos:
                    _, cx, _ = contornos_validos[0]
                    error = cx - centro_pantalla
                    ajuste = error * kp
                    v_izq = max(-40, min(70, int(vel_base + ajuste)))
                    v_der = max(-40, min(70, int(vel_base - ajuste)))
                    arduino.write(f"M {v_izq} {v_der}\n".encode())
                    print(f"Siguiendo linea... Error: {error:4d}                ", end='\r')
                else:
                    arduino.write(b'S')
                    print("Linea perdida -> Parando motores                    ", end='\r')

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        arduino.write(b'S')
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
