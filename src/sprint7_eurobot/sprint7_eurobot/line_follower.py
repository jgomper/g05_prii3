#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        
        # Parámetros ajustables
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('kp', 0.005)
        self.declare_parameter('kd', 0.002)
        self.declare_parameter('base_speed', 0.2)
        self.declare_parameter('threshold_value', 60) # Píxeles más oscuros que esto se consideran línea
        
        image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.threshold_value = self.get_parameter('threshold_value').value

        self.get_logger().info(f'Iniciando siguelíneas. Suscrito a {image_topic}, publicando en {cmd_vel_topic}')
        self.get_logger().info(f'Parametros: Kp={self.kp}, Kd={self.kd}, BaseSpeed={self.base_speed}, Thresh={self.threshold_value}')

        # Subs y Pubs
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Variables de control
        self.last_error = 0.0
        self.line_found = False

    def image_callback(self, msg):
        try:
            # Convertir mensaje de ROS a imagen de OpenCV
            # Si dice bgr8 usamos ese encoding, rgb8, mono8, etc.
            # Convertilo a bgr8 de forma segura
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error procesando imagen: {e}', throttle_duration_sec=2.0)
            return

        h, w, d = cv_image.shape

        # Recortar (Crop) la imagen para quedarnos solo con el tercio inferior.
        # Esto evita ver objetos lejanos y se concentra en la línea justo delante.
        crop_h_start = int(h * 2/3)
        crop_image = cv_image[crop_h_start:h, 0:w]

        # Convertir a escala de grises
        gray = cv2.cvtColor(crop_image, cv2.COLOR_BGR2GRAY)
        
        # Aplicar Blur para reducir el ruido
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Umbralización Binaria Inversa (línea negra sobre fondo blanco)
        # Los píxeles de la línea negra (valores bajos) pasan a ser 255 (blanco), y el fondo 0 (negro).
        _, mask = cv2.threshold(blur, self.threshold_value, 255, cv2.THRESH_BINARY_INV)

        # Encontrar contornos en la máscara
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        cmd = Twist()

        if len(contours) > 0:
            # Seleccionar el contorno cuyo centro esté más cerca del centro del robot (w/2)
            # Esto evita distraerse con las franjas verdes laterales que tienen mucha área
            best_c = None
            min_dist = float('inf')
            
            for c in contours:
                if cv2.contourArea(c) > 50:  # Ignorar ruido muy pequeño
                    M = cv2.moments(c)
                    if M['m00'] > 0:
                        cx_test = int(M['m10']/M['m00'])
                        dist = abs(cx_test - (w / 2.0))
                        
                        if dist < min_dist:
                            min_dist = dist
                            best_c = c
            
            if best_c is not None:
                M = cv2.moments(best_c)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Para depurar, podríamos dibujar el centro:
                # cv2.circle(crop_image, (cx, cy), 10, (0, 0, 255), -1)
                
                # Calcular el error (desviación del centro de la imagen)
                # El centro de la imagen es w/2
                error = cx - (w / 2.0)
                
                # Control PD
                proporcional = self.kp * error
                derivativo = self.kd * (error - self.last_error)
                
                angular_z = -1.0 * (proporcional + derivativo)
                
                # Si el error es grande (curva pronunciada), podemos reducir un poco la velocidad lineal
                velocidad_lineal = self.base_speed
                if abs(error) > (w / 4): 
                    velocidad_lineal = self.base_speed * 0.5
                
                cmd.linear.x = velocidad_lineal
                cmd.angular.z = float(angular_z)
                
                self.last_error = error
                self.line_found = True
            else:
                self.line_found = False
        else:
            self.line_found = False
            
        # Si no vemos la línea, nos paramos o giramos en el sitio buscando (comportamiento de recuperación)
        if not self.line_found:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('¡Línea perdida! Esperando...', throttle_duration_sec=1.0)
            
        self.cmd_pub.publish(cmd)

        # Opcional: mostrar la cámara para depurar en VNC
        # cv2.imshow("Visión Original", crop_image)
        # cv2.imshow("Máscara Binaria", mask)
        # cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar al salir
        cmd = Twist()
        node.cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
