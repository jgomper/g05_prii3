"""
Sprint 6 - PBI 6.1: Maquina de Estados Finitos PURA con transitions
====================================================================
FSM con 5 estados funcionales:
  1. BUSCAR           - Gira buscando el ArUco 17
  2. ACERCAR          - Avanza hacia el ArUco centrandose
  3. GIRAR_180        - Gira 180 grados
  4. CUADRADO_AVANZAR - Avanza un lado del cuadrado
  5. CUADRADO_GIRAR   - Gira 90 grados

Implementacion FSM PURA usando conditions de transitions.
"""

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from transitions import Machine


class ArucoFollowerFSM(Node):
    """Nodo ROS2 con FSM pura usando transitions."""

    # =====================================================================
    # DEFINICION DE ESTADOS (5 funcionales + INIT + DONE)
    # =====================================================================
    states = ['INIT', 'BUSCAR', 'ACERCAR', 'GIRAR_180', 
              'CUADRADO_AVANZAR', 'CUADRADO_GIRAR', 'DONE']

    # =====================================================================
    # DEFINICION DE TRANSICIONES (FSM pura con conditions)
    # =====================================================================
    transitions = [
        # INIT -> BUSCAR (arranque)
        {
            'trigger': 'start',
            'source': 'INIT',
            'dest': 'BUSCAR',
            'after': 'on_enter_buscar'
        },
        # BUSCAR -> ACERCAR (cuando detecta ArUco)
        {
            'trigger': 'check',
            'source': 'BUSCAR',
            'dest': 'ACERCAR',
            'conditions': 'is_aruco_detected',
            'after': 'on_enter_acercar'
        },
        # ACERCAR -> GIRAR_180 (cuando ArUco es grande = cerca)
        {
            'trigger': 'check',
            'source': 'ACERCAR',
            'dest': 'GIRAR_180',
            'conditions': 'is_close_enough',
            'after': 'on_enter_girar_180'
        },
        # GIRAR_180 -> CUADRADO_AVANZAR (giro completado)
        {
            'trigger': 'check',
            'source': 'GIRAR_180',
            'dest': 'CUADRADO_AVANZAR',
            'conditions': 'is_giro_180_done',
            'after': 'on_enter_cuadrado_avanzar'
        },
        # CUADRADO_AVANZAR -> CUADRADO_GIRAR (lado completado)
        {
            'trigger': 'check',
            'source': 'CUADRADO_AVANZAR',
            'dest': 'CUADRADO_GIRAR',
            'conditions': 'is_lado_done',
            'after': 'on_enter_cuadrado_girar'
        },
        # CUADRADO_GIRAR -> CUADRADO_AVANZAR (continuar cuadrado)
        {
            'trigger': 'check',
            'source': 'CUADRADO_GIRAR',
            'dest': 'CUADRADO_AVANZAR',
            'conditions': ['is_giro_90_done', 'is_cuadrado_incompleto'],
            'after': 'on_enter_cuadrado_avanzar'
        },
        # CUADRADO_GIRAR -> DONE (cuadrado completo)
        {
            'trigger': 'check',
            'source': 'CUADRADO_GIRAR',
            'dest': 'DONE',
            'conditions': ['is_giro_90_done', 'is_cuadrado_completo'],
            'after': 'on_enter_done'
        },
    ]

    def __init__(self):
        super().__init__('aruco_follower_fsm')

        # -----------------------------------------------------------------
        # Parametros configurables
        # -----------------------------------------------------------------
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('target_id', 17)
        self.declare_parameter('min_size', 25)
        self.declare_parameter('target_size', 75)
        self.declare_parameter('giro_vel', 0.8)
        self.declare_parameter('avance_vel', 0.15)
        self.declare_parameter('giro_180_duracion', 3.93)
        self.declare_parameter('giro_90_duracion', 1.96)
        self.declare_parameter('lado_duracion', 2.0)
        self.declare_parameter('image_width', 640)

        self._image_topic = self.get_parameter('image_topic').value
        self._target_id = self.get_parameter('target_id').value
        self._min_size = self.get_parameter('min_size').value
        self._target_size = self.get_parameter('target_size').value
        self._giro_vel = self.get_parameter('giro_vel').value
        self._avance_vel = self.get_parameter('avance_vel').value
        self._giro_180_duracion = self.get_parameter('giro_180_duracion').value
        self._giro_90_duracion = self.get_parameter('giro_90_duracion').value
        self._lado_duracion = self.get_parameter('lado_duracion').value
        self._image_width = self.get_parameter('image_width').value

        # -----------------------------------------------------------------
        # Variables de estado
        # -----------------------------------------------------------------
        self._aruco_detected = False
        self._aruco_cx = 0.0
        self._aruco_size = 0.0
        self._action_start_time = None
        self._cuadrado_lados = 0

        # -----------------------------------------------------------------
        # Configurar detector ArUco
        # -----------------------------------------------------------------
        try:
            params = cv2.aruco.DetectorParameters()
        except AttributeError:
            params = cv2.aruco.DetectorParameters_create()

        params.adaptiveThreshConstant = 7
        params.minMarkerPerimeterRate = 0.03
        params.maxMarkerPerimeterRate = 4.0
        params.polygonalApproxAccuracyRate = 0.03
        params.minCornerDistanceRate = 0.05
        params.minDistanceToBorder = 3
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        dict_ids = {
            '4X4_50': cv2.aruco.DICT_4X4_50,
            '4X4_100': cv2.aruco.DICT_4X4_100,
            '4X4_250': cv2.aruco.DICT_4X4_250,
        }

        self._detectors = {}
        for name, did in dict_ids.items():
            try:
                d = cv2.aruco.getPredefinedDictionary(did)
                self._detectors[name] = (d, params)
            except Exception:
                pass

        # -----------------------------------------------------------------
        # Inicializar Maquina de Estados (transitions) - FSM PURA
        # -----------------------------------------------------------------
        self._machine = Machine(
            model=self,
            states=ArucoFollowerFSM.states,
            transitions=ArucoFollowerFSM.transitions,
            initial='INIT',
            auto_transitions=False,
            ignore_invalid_triggers=True  # Ignorar triggers invalidos
        )

        # -----------------------------------------------------------------
        # Publishers y Subscribers
        # -----------------------------------------------------------------
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(Image, self._image_topic, self._image_cb, qos)

        # Timer principal de control (10 Hz)
        self._control_timer = self.create_timer(0.1, self._control_loop)

        # -----------------------------------------------------------------
        # Informacion inicial
        # -----------------------------------------------------------------
        self.get_logger().info('========================================================')
        self.get_logger().info('  Sprint 6 - PBI 6.1: FSM PURA con transitions')
        self.get_logger().info('========================================================')
        self.get_logger().info('  Estados: BUSCAR -> ACERCAR -> GIRAR_180 ->')
        self.get_logger().info('           CUADRADO_AVANZAR <-> CUADRADO_GIRAR -> DONE')
        self.get_logger().info('--------------------------------------------------------')
        self.get_logger().info(f'  Target ID     : {self._target_id}')
        self.get_logger().info(f'  Target size   : {self._target_size}px (= muy cerca)')
        self.get_logger().info(f'  Giro vel      : {self._giro_vel} rad/s')
        self.get_logger().info(f'  Giro 180 dur  : {self._giro_180_duracion:.2f}s')
        self.get_logger().info(f'  Giro 90 dur   : {self._giro_90_duracion:.2f}s')
        self.get_logger().info('')
        self.get_logger().info('  Iniciando FSM...')

        # Arrancar FSM inmediatamente
        self.start()

    # =====================================================================
    # CONDITIONS (Funciones de condicion para transiciones)
    # =====================================================================

    def is_aruco_detected(self):
        """Condicion: ArUco detectado."""
        return self._aruco_detected

    def is_close_enough(self):
        """Condicion: ArUco suficientemente grande (cerca)."""
        return self._aruco_detected and self._aruco_size >= self._target_size

    def is_giro_180_done(self):
        """Condicion: Giro de 180 grados completado."""
        if self._action_start_time is None:
            return False
        elapsed = (self.get_clock().now() - self._action_start_time).nanoseconds / 1e9
        return elapsed >= self._giro_180_duracion

    def is_lado_done(self):
        """Condicion: Lado del cuadrado completado."""
        if self._action_start_time is None:
            return False
        elapsed = (self.get_clock().now() - self._action_start_time).nanoseconds / 1e9
        return elapsed >= self._lado_duracion

    def is_giro_90_done(self):
        """Condicion: Giro de 90 grados completado."""
        if self._action_start_time is None:
            return False
        elapsed = (self.get_clock().now() - self._action_start_time).nanoseconds / 1e9
        return elapsed >= self._giro_90_duracion

    def is_cuadrado_incompleto(self):
        """Condicion: Cuadrado aun no completado (menos de 4 lados)."""
        return self._cuadrado_lados < 4

    def is_cuadrado_completo(self):
        """Condicion: Cuadrado completado (4 lados)."""
        return self._cuadrado_lados >= 4

    # =====================================================================
    # CALLBACKS DE ESTADO (on_enter_*)
    # =====================================================================

    def on_enter_buscar(self):
        self.get_logger().info('[BUSCAR] Girando para encontrar ArUco 17...')

    def on_enter_acercar(self):
        self.get_logger().info('[ACERCAR] ArUco detectado! Acercandose...')

    def on_enter_girar_180(self):
        self.get_logger().info('[GIRAR_180] Muy cerca! Girando 180 grados...')
        self._stop_robot()
        self._action_start_time = self.get_clock().now()
        self._cuadrado_lados = 0

    def on_enter_cuadrado_avanzar(self):
        self._cuadrado_lados += 1
        self.get_logger().info(f'[CUADRADO_AVANZAR] Lado {self._cuadrado_lados}/4')
        self._action_start_time = self.get_clock().now()

    def on_enter_cuadrado_girar(self):
        self.get_logger().info(f'[CUADRADO_GIRAR] Girando 90 grados')
        self._action_start_time = self.get_clock().now()

    def on_enter_done(self):
        self.get_logger().info('[DONE] Secuencia completada!')
        self._stop_robot()

    # =====================================================================
    # PROCESAMIENTO DE IMAGEN
    # =====================================================================

    def _image_cb(self, msg):
        """Procesa imagen y actualiza deteccion."""
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.equalizeHist(gray)

            self._image_width = msg.width
            self._aruco_detected = False

            for dict_name, (aruco_dict, aruco_params) in self._detectors.items():
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, aruco_dict, parameters=aruco_params)

                if ids is not None:
                    for i, marker_id in enumerate(ids.flatten()):
                        if marker_id != self._target_id:
                            continue
                        pts = corners[i][0]
                        cx = float((pts[0][0] + pts[2][0]) / 2.0)
                        w = float(abs(pts[1][0] - pts[0][0]))
                        if w < self._min_size:
                            continue
                        self._aruco_detected = True
                        self._aruco_cx = cx
                        self._aruco_size = w
                        return

        except Exception as e:
            self.get_logger().warn(f'Error: {e}', throttle_duration_sec=2.0)

    # =====================================================================
    # BUCLE DE CONTROL (ejecuta acciones y trigger check)
    # =====================================================================

    def _control_loop(self):
        """Bucle de control: ejecuta accion del estado actual y trigger check."""
        state = self.state

        # Ejecutar accion segun estado actual
        if state == 'BUSCAR':
            self._action_buscar()
        elif state == 'ACERCAR':
            self._action_acercar()
        elif state == 'GIRAR_180':
            self._action_girar()
        elif state == 'CUADRADO_AVANZAR':
            self._action_avanzar()
        elif state == 'CUADRADO_GIRAR':
            self._action_girar()

        # Intentar transicion (FSM evalua conditions automaticamente)
        if state != 'DONE' and state != 'INIT':
            self.check()

    # =====================================================================
    # ACCIONES POR ESTADO
    # =====================================================================

    def _action_buscar(self):
        """Accion en BUSCAR: girar buscando."""
        cmd = Twist()
        cmd.angular.z = self._giro_vel
        self._cmd_pub.publish(cmd)

    def _action_acercar(self):
        """Accion en ACERCAR: avanzar hacia el ArUco."""
        cmd = Twist()
        if self._aruco_detected:
            center_x = self._image_width / 2.0
            error_x = (self._aruco_cx - center_x) / center_x
            cmd.linear.x = self._avance_vel
            cmd.angular.z = -error_x * self._giro_vel * 1.5
            self.get_logger().info(
                f'    -> size={self._aruco_size:.0f}/{self._target_size}px',
                throttle_duration_sec=0.5)
        else:
            cmd.linear.x = self._avance_vel * 0.5
        self._cmd_pub.publish(cmd)

    def _action_girar(self):
        """Accion en GIRAR_180 o CUADRADO_GIRAR: girar."""
        cmd = Twist()
        cmd.angular.z = self._giro_vel
        self._cmd_pub.publish(cmd)

    def _action_avanzar(self):
        """Accion en CUADRADO_AVANZAR: avanzar recto."""
        cmd = Twist()
        cmd.linear.x = self._avance_vel
        self._cmd_pub.publish(cmd)

    # =====================================================================
    # UTILIDADES
    # =====================================================================

    def _stop_robot(self):
        """Detiene el robot."""
        cmd = Twist()
        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerFSM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd = Twist()
        node._cmd_pub.publish(cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
