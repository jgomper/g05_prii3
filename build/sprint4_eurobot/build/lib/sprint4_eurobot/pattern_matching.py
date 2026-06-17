#!/usr/bin/env python3
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import cv2
import json
import numpy as np

class PatternMatchingNode(Node):
    """
    Nodo ROS2 para la detección de almacenes usando PATTERN MATCHING (Template Matching).
    
    ALGORITMO: MULTI-SCALE TEMPLATE MATCHING
    ----------------------------------------
    Tal y como se solicita en la teoría (basado en plantillas):
    1.  Genera una plantilla sintética (cuadrado verde) en el __init__.
    2.  Escala la imagen de entrada en varios tamaños para manejar la perspectiva
        (objetos lejanos se ven más pequeños).
    3.  Aplica `cv2.matchTemplate` con el método TM_CCOEFF_NORMED.
    4.  Filtra los resultados por un umbral de coincidencia (threshold).
    5.  Utiliza Non-Maximum Suppression (NMS) simplificado para evitar múltiples
        detecciones del mismo objeto.
    """
    def __init__(self):
        super().__init__('pattern_matching_node')
        
        self.bridge = CvBridge()
        
        # Parámetros
        self.declare_parameter('image_topic', '/overhead_camera/image_raw')
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        
        self.declare_parameter('show_gui', True)
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value
        
        # Threshold de coincidencia (0.0 a 1.0)
        # En binario, un match perfecto es 1.0. Bajamos para tolerar rotación.
        self.declare_parameter('match_threshold', 0.50)
        self.match_threshold = self.get_parameter('match_threshold').get_parameter_value().double_value

        # GENERACIÓN DE PLANTILLAS BINARIAS (MULTI-ANGLE)
        # Para mejorar la detección en perspectiva, generamos el template rotado.
        # Los almacenes laterales se ven como rombos/trapecios.
        self.template_size = 20
        self.templates = []
        
        # Rotaciones: [-90, -80, ... 90] para cubrir todo (Paso 10°)
        angles = list(range(-90, 91, 10))
        
        base_template = np.zeros((self.template_size, self.template_size), dtype=np.uint8)
        cv2.rectangle(base_template, (3, 3), (17, 17), 255, 2)
        
        for angle in angles:
            center = (self.template_size // 2, self.template_size // 2)
            M = cv2.getRotationMatrix2D(center, angle, 1.0)
            rotated = cv2.warpAffine(base_template, M, (self.template_size, self.template_size))
            self.templates.append(rotated)

        # Suscriptores y Publicadores
        self.get_logger().info(f"Subscribing to: {self.image_topic} (Sensor Data QoS)")
        self.sub_img = self.create_subscription(
            Image, 
            self.image_topic, 
            self.img_callback, 
            qos_profile_sensor_data
        )
        
        self.pub_warehouses = self.create_publisher(String, '/overhead_camera/pattern_matches', 10)
        # RELIABLE para visualización externa estable
        self.pub_debug = self.create_publisher(Image, '/overhead_camera/pattern_debug', 10)
        self.pub_markers = self.create_publisher(MarkerArray, '/overhead_camera/pattern_markers', 10)
        
        self.get_logger().info('Pattern Matching Node Started (STABLE: 0.5x Scale + No GUI)')

    def img_callback(self, msg):
        try:
            if msg.encoding == 'bgr8':
                frame = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
            elif msg.encoding == 'rgb8':
                frame = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                self.get_logger().warn(f"Unsupported encoding: {msg.encoding}")
                return
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # ESTRATEGIA ESTABLE: Downscale 0.5x (fijo)
        # 1280x720 -> 640x360. Suficiente detalle, muy rápido.
        scale_input = 0.5
        small_frame = cv2.resize(frame, (0,0), fx=scale_input, fy=scale_input)
        
        debug_frame = frame.copy()
        
        # 1. MASCARA EN BAJA RESOLUCION (Rapido)
        # H: 30 incluye amarillo. Subimos a 45 para quedarnos solo con verde puro.
        hsv = cv2.cvtColor(small_frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([45, 40, 40])
        upper_green = np.array([90, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Closing muy suave (2x2) para NO borrar almacenes lejanos/finos
        kernel = np.ones((2,2), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 2. MATCHING
        found_matches = []
        best_overall_score = 0
        
        # Escalas relativas a la imagen reducida.
        # Ampliamos rango (0.15 a 3.5) y mas pasos (30) para pillar los muy pequeños y los muy grandes
        search_scales = np.linspace(0.15, 3.5, 30) 
        
        for s in search_scales[::-1]: 
            resized_mask = cv2.resize(mask, (int(mask.shape[1] * s), int(mask.shape[0] * s)))
            r = mask.shape[1] / float(resized_mask.shape[1]) 
            
            if resized_mask.shape[0] < self.template_size or resized_mask.shape[1] < self.template_size:
                break
            
            for template in self.templates:
                result = cv2.matchTemplate(resized_mask, template, cv2.TM_CCOEFF_NORMED)
                # Threshold parametrizable (y algo más exigente)
                loc = np.where(result >= self.match_threshold) 
                
                for pt in zip(*loc[::-1]):
                    score = result[pt[1], pt[0]]
                    if score > best_overall_score: best_overall_score = score
                    
                    # Coordenadas en SMALL FRAME
                    orig_x_small = int(pt[0] * r)
                    orig_y_small = int(pt[1] * r)
                    w_small = int(self.template_size * r)
                    h_small = int(self.template_size * r)
                    
                    # Centro en SMALL FRAME
                    cX_small = orig_x_small + w_small // 2
                    cY_small = orig_y_small + h_small // 2
                    
                    found_matches.append((cX_small, cY_small, w_small, h_small, score))

        # 3. NMS
        final_matches = []
        found_matches.sort(key=lambda x: x[4], reverse=True)
        
        normalized_matches = []
        nms_radius_small = 60 # AUMENTADO (Antes 40) para evitar detectar 2 veces el mismo almacén
        
        for (cX, cY, w, h, score) in found_matches:
            is_new = True
            for (nx, ny, _, _, _) in normalized_matches:
                dist = np.sqrt((cX - nx)**2 + (cY - ny)**2)
                if dist < nms_radius_small: 
                    is_new = False
                    break
            
            if is_new:
                normalized_matches.append((cX, cY, w, h, score))
                
                # ESCALAR A FULL RES
                final_x = int(cX / scale_input)
                final_y = int(cY / scale_input)
                final_w = int(w / scale_input)
                final_h = int(h / scale_input)
                
                final_matches.append({
                    "px": final_x, "py": final_y, "w": final_w, "h": final_h, "score": score
                })
        
        # VISUALIZACIÓN
        warehouses_msg = []
        markers = MarkerArray()
        
        for i, m in enumerate(final_matches):
            cX, cY, w, h, score = m['px'], m['py'], m['w'], m['h'], m['score']
            
            cv2.rectangle(debug_frame, (cX - w//2, cY - h//2), (cX + w//2, cY + h//2), (255, 0, 0), 2)
            cv2.putText(debug_frame, f"{score:.2f}", (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            print(f"¡ALMACEN DETECTADO! Pos: ({cX}, {cY}) - Score: {score:.2f}", flush=True)
            warehouses_msg.append({"px": cX, "py": cY})
            
            marker = Marker()
            marker.header = msg.header
            marker.ns = "pattern_matches"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            scale = 0.003
            marker.pose.position.x = (cX - msg.width/2) * scale
            marker.pose.position.y = (cY - msg.height/2) * scale
            marker.pose.position.z = 0.0
            marker.scale.x = 0.15; marker.scale.y = 0.15; marker.scale.z = 0.01
            marker.color.a = 0.8; marker.color.r = 0.0; marker.color.g = 0.0; marker.color.b = 1.0
            markers.markers.append(marker)
            
        self.pub_markers.publish(markers)
        if warehouses_msg:
             msg_str = json.dumps(warehouses_msg)
             self.pub_warehouses.publish(String(data=msg_str))
             print(f"ALMACENES DETECTADOS: {len(warehouses_msg)} (Max Score: {best_overall_score:.2f})", flush=True)
        
        try:
            debug_frame = np.ascontiguousarray(debug_frame)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            debug_msg.header = msg.header
            debug_msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_debug.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")

        # GUI DEPURACIÓN (Re-activada tras optimización)
        if self.show_gui:
            cv2.imshow("Pattern Matching (Binary) Debug", debug_frame)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PatternMatchingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
