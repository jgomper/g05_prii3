#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
from geometry_msgs.msg import TransformStamped

class RealVisionNode(Node):
    def __init__(self):
        super().__init__('real_vision_node')

        self.declare_parameter('marker_size', 0.10) # 10 cm
        self.marker_size = self.get_parameter('marker_size').value
        
        # Parámetro para robustez en iluminación (Thresholding)
        self.declare_parameter('detection_threshold', 7) 
        self.detection_threshold = self.get_parameter('detection_threshold').value

        # --- CONFIGURACIÓN ARUCO (COMPATIBILIDAD OPENCV 4.0.6) ---
        # En OpenCV 4.0.6 no existe ArucoDetector ni getPredefinedDictionary
        # Usamos estrictamente la API antigua.
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.use_new_api = False
        self.get_logger().info('Forzando API Antigua (OpenCV 4.0.6)')

        # Ajustes de robustez
        self.aruco_params.adaptiveThreshConstant = float(self.detection_threshold)
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 30
        self.aruco_params.adaptiveThreshWinSizeStep = 5
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # Puntos 3D del marcador
        half = self.marker_size / 2.0
        self.marker_points = np.array([
            [-half, half, 0],
            [half, half, 0],
            [half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        # Suscripciones
        self.sub_img = self.create_subscription(Image, '/image_raw', self.img_callback, 10)
        self.sub_info = self.create_subscription(CameraInfo, '/camera_info', self.info_callback, 10)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Matriz de cámara por defecto (se sobrescribe con camera_info)
        self.camera_matrix = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5,1), dtype=np.float32)
        self.calibrated = False

        self.get_logger().info('Real Vision Node Iniciado. Esperando imágenes...')

    def info_callback(self, msg):
        if not self.calibrated:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.calibrated = True
            self.get_logger().info('¡Calibración de cámara recibida!')

    def img_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Error convirtiendo imagen: {e}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. DETECTAR
        if self.use_new_api:
            corners, ids, rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            for i in range(len(ids)):
                marker_id = ids[i][0]
                try:
                    # 2. CALCULAR POSE
                    _, rvec, tvec = cv2.solvePnP(
                        self.marker_points, corners[i], self.camera_matrix, self.dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    # 3. PUBLICAR TF
                    # overhead_camera -> aruco_X
                    self.publish_tf(rvec, tvec, f"aruco_{marker_id}")

                    # Dibujar para debug (opcional, pero útil si se visualiza el topic de imagen procesada)
                    # No publicamos la imagen procesada para ahorrar ancho de banda, pero podríamos.
                    
                except Exception as e:
                    self.get_logger().warn(f"Error pose marcador {marker_id}: {e}")

    def publish_tf(self, rvec, tvec, child_frame_id):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "overhead_camera" # Frame de la cámara
        t.child_frame_id = child_frame_id

        t_flat = np.array(tvec).flatten()
        r_flat = np.array(rvec).flatten()

        t.transform.translation.x = float(t_flat[0])
        t.transform.translation.y = float(t_flat[1])
        t.transform.translation.z = float(t_flat[2])

        rmat, _ = cv2.Rodrigues(r_flat)
        
        # Matriz a Cuaternión
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
    node = RealVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
