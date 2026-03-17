
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ArucoDetectorNode(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('target_id', 17)
        self.declare_parameter('min_size', 25)  
        image_topic = self.get_parameter('image_topic').value
        self._target_id = self.get_parameter('target_id').value
        self._min_size = self.get_parameter('min_size').value

        
        try:
            params = cv2.aruco.DetectorParameters()
        except AttributeError:
            params = cv2.aruco.DetectorParameters_create()

        params.adaptiveThreshConstant       = 7
        params.minMarkerPerimeterRate       = 0.03   
        params.maxMarkerPerimeterRate       = 4.0
        params.polygonalApproxAccuracyRate  = 0.03   
        params.minCornerDistanceRate        = 0.05  
        params.minDistanceToBorder          = 3      
        params.cornerRefinementMethod       = cv2.aruco.CORNER_REFINE_SUBPIX

      
        dict_ids = {
            'ARUCO_ORIGINAL': cv2.aruco.DICT_ARUCO_ORIGINAL,
            '4X4_50':  cv2.aruco.DICT_4X4_50,
            '4X4_100': cv2.aruco.DICT_4X4_100,
            '4X4_250': cv2.aruco.DICT_4X4_250,
            '5X5_50':  cv2.aruco.DICT_5X5_50,
            '5X5_100': cv2.aruco.DICT_5X5_100,
            '6X6_50':  cv2.aruco.DICT_6X6_50,
            '6X6_100': cv2.aruco.DICT_6X6_100,
        }

        self._detectors = {}
        for name, did in dict_ids.items():
            try:
                try:
                    d = cv2.aruco.getPredefinedDictionary(did)
                except AttributeError:
                    d = cv2.aruco.Dictionary_get(did)
                self._detectors[name] = (d, params)
            except Exception:
                pass  

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            Image, image_topic, self._image_cb, qos)

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║   Sprint 6 - Detector ArUco         ║')
        self.get_logger().info('╚══════════════════════════════════════╝')
        self.get_logger().info(f'  Topic    : {image_topic}')
        self.get_logger().info(f'  Target ID: {self._target_id}')
        self.get_logger().info(f'  Min size : {self._min_size}px')
        self.get_logger().info(f'  Dicts    : {list(self._detectors.keys())}')
        self.get_logger().info('  Esperando imágenes...')

    def _image_cb(self, msg):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            found_any = False
            for dict_name, (aruco_dict, aruco_params) in self._detectors.items():
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=aruco_params)

                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten()):
                        if marker_id != self._target_id:
                            continue  
                        pts = corners[i][0]
                        cx  = float((pts[0][0] + pts[2][0]) / 2.0)
                        cy  = float((pts[0][1] + pts[2][1]) / 2.0)
                        w   = float(abs(pts[1][0] - pts[0][0]))
                       
                        if w < self._min_size:
                            continue
                        found_any = True
                        self.get_logger().info(
                            f'  [{dict_name}] ID={marker_id}  '
                            f'cx={cx:.0f}  cy={cy:.0f}  w={w:.0f}px'
                        )

            if not found_any:
                self.get_logger().info(
                    f'  Nada detectado  (encoding={msg.encoding} '
                    f'{msg.width}x{msg.height})',
                    throttle_duration_sec=2.0
                )

        except Exception as e:
            self.get_logger().warn(f'Error: {e}', throttle_duration_sec=2.0)


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
