"""
Sprint 6 - Detector de ArUco (Multi-diccionario)
=================================================
Nodo ROS2 que detecta cualquier marcador ArUco probando múltiples
diccionarios automáticamente. Muestra por terminal el ID detectado,
el diccionario que lo reconoció, su posición y tamaño.

Topic de entrada: /csi_cam_0/image_raw
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


# Diccionarios ArUco a probar (id → nombre)
ARUCO_DICTS = {
    cv2.aruco.DICT_4X4_50:   '4X4_50',
    cv2.aruco.DICT_4X4_100:  '4X4_100',
    cv2.aruco.DICT_4X4_250:  '4X4_250',
    cv2.aruco.DICT_5X5_50:   '5X5_50',
    cv2.aruco.DICT_5X5_100:  '5X5_100',
    cv2.aruco.DICT_6X6_50:   '6X6_50',
    cv2.aruco.DICT_6X6_100:  '6X6_100',
}


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/csi_cam_0/image_raw')
        image_topic = self.get_parameter('image_topic').value

        # Construir detectores para cada diccionario
        self._detectors = {}
        try:
            # OpenCV >= 4.7
            params = cv2.aruco.DetectorParameters()
            for dict_id, name in ARUCO_DICTS.items():
                d = cv2.aruco.getPredefinedDictionary(dict_id)
                self._detectors[name] = (d, params)
        except AttributeError:
            # OpenCV < 4.7
            params = cv2.aruco.DetectorParameters_create()
            for dict_id, name in ARUCO_DICTS.items():
                d = cv2.aruco.Dictionary_get(dict_id)
                self._detectors[name] = (d, params)

        # Suscripción a la cámara
        self.create_subscription(
            Image,
            image_topic,
            self._image_cb,
            qos_profile_sensor_data
        )

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║   Sprint 6 - Detector ArUco         ║')
        self.get_logger().info('╚══════════════════════════════════════╝')
        self.get_logger().info(f'  Topic: {image_topic}')
        self.get_logger().info(f'  Diccionarios: {list(self._detectors.keys())}')
        self.get_logger().info('  Esperando marcadores ArUco...')

    def _image_cb(self, msg):
        """Procesa cada frame y detecta cualquier ArUco con todos los diccionarios."""
        try:
            # Convertir Image ROS2 → numpy BGR
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            found_any = False
            for dict_name, (aruco_dict, aruco_params) in self._detectors.items():
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=aruco_params)

                if ids is not None:
                    found_any = True
                    for i, marker_id in enumerate(ids.flatten()):
                        pts = corners[i][0]
                        cx  = float((pts[0][0] + pts[2][0]) / 2.0)
                        cy  = float((pts[0][1] + pts[2][1]) / 2.0)
                        w   = float(abs(pts[1][0] - pts[0][0]))
                        self.get_logger().info(
                            f'  ✅ [{dict_name}] ArUco ID={marker_id:3d}  '
                            f'cx={cx:.0f}px  cy={cy:.0f}px  ancho={w:.0f}px'
                        )

            if not found_any:
                self.get_logger().info(
                    '  ❌ No se detecta ningún ArUco',
                    throttle_duration_sec=2.0
                )

        except Exception as e:
            self.get_logger().warn(f'Error imagen: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
