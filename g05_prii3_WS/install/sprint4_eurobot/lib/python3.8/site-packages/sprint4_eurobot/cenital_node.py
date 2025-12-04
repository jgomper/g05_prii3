#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped


class CenitalCameraNode(Node):
    def __init__(self):
        super().__init__('cenital_camera_node')

        self.marker_size = 0.10 # 10 cm
        
        # --- CONFIGURACIÓN DE VERSIÓN OPENCV ---
        self.use_new_api = False
        try:
            # Intentamos cargar lo nuevo (OpenCV >= 4.7)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.use_new_api = True
            self.get_logger().info('Usando API Moderna (4.7+)')
        except AttributeError:
            # Fallback a lo viejo (OpenCV < 4.7)
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.get_logger().info('Usando API Antigua (<4.7)')

        # Puntos 3D del marcador (para solvePnP)
        half = self.marker_size / 2.0
        self.marker_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        self.sub_img = self.create_subscription(Image, '/overhead_camera/image_raw', self.img_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/overhead_camera/camera_info', self.info_callback, 10)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.get_logger().info('Nodo Cenital Listo. Esperando imagen...')

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('¡Calibración recibida!')

    def img_callback(self, msg):
        if self.camera_matrix is None: return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. DETECTAR
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Imprimir en terminal (opcional, lo dejo por si acaso)
            ids_list = ids.flatten()
            # self.get_logger().info(f"-> IDs Detectados: {ids_list}")

            # 2. CALCULAR POSE Y DIBUJAR EN IMAGEN
            for i in range(len(ids)):
                marker_id = ids[i][0]
                try:
                    _, rvec, tvec = cv2.solvePnP(
                        self.marker_points, corners[i], self.camera_matrix, self.dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE
                    )
                    
                    # Print ID and Axes to terminal
                    self.get_logger().info(f"ID: {marker_id} | Rvec: {rvec.flatten()} | Tvec: {tvec.flatten()}")

                    # Dibujar Ejes
                    try:
                        cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)
                    except AttributeError:
                        cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

                    # --- NUEVO: DIBUJAR TEXTO "ID: X" EN LA IMAGEN ---
                    # Cogemos la primera esquina del marcador para posicionar el texto
                    corner_x = int(corners[i][0][0][0])
                    corner_y = int(corners[i][0][0][1])
                    
                    cv2.putText(
                        frame, 
                        f"ID: {marker_id}", 
                        (corner_x, corner_y - 10), # Posición (un poco más arriba de la esquina)
                        cv2.FONT_HERSHEY_SIMPLEX,  # Fuente
                        0.5,                       # Escala de la fuente
                        (0, 255, 0),               # Color (Verde en BGR)
                        2                          # Grosor
                    )
                    # ------------------------------------------------

                    # Publicar TF
                    self.publish_tf(rvec, tvec, f"aruco_{marker_id}")
                    
                except Exception as e:
                    self.get_logger().warn(f"Error calculando pose marcador {marker_id}: {e}")

        cv2.imshow("Camara Cenital", frame)
        cv2.waitKey(1)

    def publish_tf(self, rvec, tvec, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "overhead_camera_link" 
        t.child_frame_id = child_frame_id

        t_flat = np.array(tvec).flatten()
        r_flat = np.array(rvec).flatten()

        t.transform.translation.x = float(t_flat[0])
        t.transform.translation.y = float(t_flat[1])
        t.transform.translation.z = float(t_flat[2])

        rmat, _ = cv2.Rodrigues(r_flat)
        
        # Manual Rotation Matrix to Quaternion (x, y, z, w)
        # Replacing scipy to avoid dependency issues
        tr = rmat[0,0] + rmat[1,1] + rmat[2,2]
        if tr > 0:
            S = np.sqrt(tr+1.0) * 2
            qw = 0.25 * S
            qx = (rmat[2,1] - rmat[1,2]) / S
            qy = (rmat[0,2] - rmat[2,0]) / S
            qz = (rmat[1,0] - rmat[0,1]) / S
        elif (rmat[0,0] > rmat[1,1]) and (rmat[0,0] > rmat[2,2]):
            S = np.sqrt(1.0 + rmat[0,0] - rmat[1,1] - rmat[2,2]) * 2
            qw = (rmat[2,1] - rmat[1,2]) / S
            qx = 0.25 * S
            qy = (rmat[0,1] + rmat[1,0]) / S
            qz = (rmat[0,2] + rmat[2,0]) / S
        elif rmat[1,1] > rmat[2,2]:
            S = np.sqrt(1.0 + rmat[1,1] - rmat[0,0] - rmat[2,2]) * 2
            qw = (rmat[0,2] - rmat[2,0]) / S
            qx = (rmat[0,1] + rmat[1,0]) / S
            qy = 0.25 * S
            qz = (rmat[1,2] + rmat[2,1]) / S
        else:
            S = np.sqrt(1.0 + rmat[2,2] - rmat[0,0] - rmat[1,1]) * 2
            qw = (rmat[1,0] - rmat[0,1]) / S
            qx = (rmat[0,2] + rmat[2,0]) / S
            qy = (rmat[1,2] + rmat[2,1]) / S
            qz = 0.25 * S
        
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = CenitalCameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()