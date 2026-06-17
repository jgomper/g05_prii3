"""
Sprint 6 - FSM ArUco Navigator (Cámara CSI Frontal - JetBot)
=============================================================
Nodo ROS2 que usa la cámara CSI frontal del JetBot (IMX219) para detectar
un ArUco y ejecutar la siguiente secuencia de forma autónoma:

  1. BUSCANDO       : Gira buscando el ArUco con la cámara frontal.
  2. NAVEGANDO      : Navega hacia el ArUco (control proporcional visual).
  3. GIRANDO_180    : Gira 180° una vez está frente al marcador.
  4. RETROCEDIENDO  : Avanza recto el doble del tiempo que tardó en llegar.
  5. DONE           : Para el robot.

Nota: usa cv2.VideoCapture(0) directamente (OpenCV sin GStreamer).
Los frames se redimensionan a 640x480 para mayor velocidad de detección.

Implementación PURA de transitions:
  - Sin if/elif en el flujo principal.
  - Toda la lógica en callbacks on_enter_*.
  - Cada estado crea sus propios timers ROS2 y los cancela al salir.

Captura de cámara:
  - Usa OpenCV + GStreamer (nvarguscamerasrc) en un hilo de fondo.
  - Compatible con la cámara CSI IMX219 del JetBot Nano.
  - NO requiere usb_cam ni v4l2_camera.

Dependencias adicionales:
    pip install opencv-contrib-python transitions
"""

import time
import math
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from transitions import Machine, State


# ─── ESTADOS ───────────────────────────────────────────────────────────────────
STATES = [
    State(name='BUSCANDO'),
    State(name='NAVEGANDO'),
    State(name='GIRANDO_180'),
    State(name='RETROCEDIENDO'),
    State(name='DONE', final=True),
]

# ─── TRANSICIONES ──────────────────────────────────────────────────────────────
TRANSITIONS = [
    # ArUco encontrado mientras buscaba → navegar hacia él
    {'trigger': 'aruco_detectado',   'source': 'BUSCANDO',      'dest': 'NAVEGANDO'},

    # Llegada al ArUco → girar 180°
    {'trigger': 'llegada',           'source': 'NAVEGANDO',      'dest': 'GIRANDO_180'},

    # 180° completado → retroceder el doble
    {'trigger': 'giro_completo',     'source': 'GIRANDO_180',    'dest': 'RETROCEDIENDO'},

    # Retroceso completado → fin
    {'trigger': 'retroceso_listo',   'source': 'RETROCEDIENDO',  'dest': 'DONE'},
]


# ─── NODO ROS2 ─────────────────────────────────────────────────────────────────
class FSMArucoNavNode(Node):
    """
    FSM pura con transitions: gira buscando ArUco → navega → 180° → retrocede.
    Toda la lógica vive en callbacks on_enter_*. Sin if/elif.
    La cámara CSI se captura con OpenCV+GStreamer en un hilo de fondo.
    """

    # ── Constructor ────────────────────────────────────────────────────────────
    def __init__(self):
        super().__init__('fsm_aruco_navigator')

        # Parámetros ROS2
        self.declare_parameter('aruco_dict_id', 0)      # DICT_4X4_50 = 0
        self.declare_parameter('target_id',     17)     # ID del ArUco a buscar

        # Control de búsqueda y navegación
        self.declare_parameter('busqueda_vel',   0.30)  # rad/s al girar buscando
        self.declare_parameter('nav_vel_lin',    0.15)  # m/s avanzando
        self.declare_parameter('kp_ang',         0.003) # ganancia proporcional angular (px→rad/s)
        self.declare_parameter('close_px',       80)    # ancho marker (px) para "llegada"
        self.declare_parameter('giro_vel',       0.50)  # rad/s para el giro 180°
        self.declare_parameter('retro_vel',      0.15)  # m/s para el retroceso

        self.TARGET_ID    = self.get_parameter('target_id').value
        self.BUSQUEDA_VEL = self.get_parameter('busqueda_vel').value
        self.NAV_VEL_LIN  = self.get_parameter('nav_vel_lin').value
        self.KP_ANG       = self.get_parameter('kp_ang').value
        self.CLOSE_PX     = self.get_parameter('close_px').value
        self.GIRO_VEL     = self.get_parameter('giro_vel').value
        self.RETRO_VEL    = self.get_parameter('retro_vel').value

        # Duración del giro 180°: π rad / giro_vel
        self.GIRO_180_DUR = math.pi / self.GIRO_VEL

        # Variables de estado compartidas
        self._lock           = threading.Lock()
        self._marker_cx      = None   # centro X del marker en la imagen (px)
        self._marker_w       = None   # ancho del marker en la imagen (px)
        self._img_cx         = None   # centro X de la imagen (px)
        self._nav_start_time = 0.0    # momento en que se empezó a navegar

        # Timers activos (se gestionan en on_enter/on_exit)
        self._pub_timer = None
        self._dur_timer = None

        # Detector ArUco — compatible con OpenCV < 4.7 y >= 4.7
        aruco_dict_id = self.get_parameter('aruco_dict_id').value
        try:
            # OpenCV >= 4.7
            self._aruco_dict   = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            # OpenCV < 4.7
            self._aruco_dict   = cv2.aruco.Dictionary_get(aruco_dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters_create()

        # Publisher cmd_vel
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── Hilo de captura de cámara CSI ─────────────────────────────────────
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            self.get_logger().error('❌ No se pudo abrir la cámara (VideoCapture(0)).')
        else:
            self.get_logger().info('  Cámara abierta con VideoCapture(0) ✓')

        self._cam_running = True
        self._cam_thread  = threading.Thread(target=self._camera_loop, daemon=True)
        self._cam_thread.start()

        # ── FSM ──────────────────────────────────────────────────────────────
        self.machine = Machine(
            model=self,
            states=STATES,
            transitions=TRANSITIONS,
            initial='BUSCANDO',
            ignore_invalid_triggers=True,
        )

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║  Sprint 6 - FSM ArUco CSI JetBot    ║')
        self.get_logger().info('╚══════════════════════════════════════╝')
        self.get_logger().info(f'  Target ArUco ID : {self.TARGET_ID}')
        self.get_logger().info(f'  Giro 180° dur.  : {self.GIRO_180_DUR:.2f} s')

        # transitions no llama on_enter para el estado inicial → arrancar manual
        self.on_enter_BUSCANDO()

    # ─── HILO DE CAPTURA CSI ───────────────────────────────────────────────────

    def _camera_loop(self):
        """Captura frames y detecta ArUco continuamente en un hilo de fondo."""
        self.get_logger().info('  Hilo de cámara iniciado.')
        while self._cam_running:
            if not self._cap.isOpened():
                time.sleep(0.1)
                continue

            ret, frame = self._cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # Reducir resolución para acelerar la detección (la cámara da 3264x2464)
            frame = cv2.resize(frame, (640, 480))
            h, w = frame.shape[:2]

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict, parameters=self._aruco_params)

            with self._lock:
                self._img_cx    = w / 2.0
                self._marker_cx = None
                self._marker_w  = None

                if ids is not None:
                    for i, mid in enumerate(ids.flatten()):
                        if mid == self.TARGET_ID:
                            pts = corners[i][0]
                            self._marker_cx = float((pts[0][0] + pts[2][0]) / 2.0)
                            self._marker_w  = float(abs(pts[1][0] - pts[0][0]))
                            break

        self._cap.release()
        self.get_logger().info('  Hilo de cámara detenido.')

    # ─── HELPERS ───────────────────────────────────────────────────────────────

    def _get_vision(self):
        """Devuelve (marker_cx, marker_w, img_cx) de forma thread-safe."""
        with self._lock:
            return self._marker_cx, self._marker_w, self._img_cx

    def _cancel_timers(self):
        """Cancela los timers activos del estado anterior."""
        if self._pub_timer:
            self._pub_timer.cancel()
            self._pub_timer = None
        if self._dur_timer:
            self._dur_timer.cancel()
            self._dur_timer = None

    # ─── CALLBACKS on_enter — toda la lógica aquí ──────────────────────────────

    def on_enter_BUSCANDO(self):
        """Gira lentamente. En cada tick comprueba si el ArUco aparece en cámara."""
        self.get_logger().info('[FSM] → BUSCANDO | Girando en busca del ArUco...')
        self._pub_timer = self.create_timer(0.05, self._tick_buscando)

    def on_enter_NAVEGANDO(self):
        """Control proporcional visual: centra el marker y avanza hacia él."""
        self._nav_start_time = time.time()
        self.get_logger().info('[FSM] → NAVEGANDO | ArUco encontrado, acercándome...')
        self._pub_timer = self.create_timer(0.05, self._tick_navegando)

    def on_enter_GIRANDO_180(self):
        """Gira exactamente 180° con temporizador."""
        nav_dur = time.time() - self._nav_start_time
        self.get_logger().info(
            f'[FSM] → GIRANDO_180 | Tiempo navegación: {nav_dur:.1f}s '
            f'→ Retroceso será {2*nav_dur:.1f}s'
        )
        self._pub_timer = self.create_timer(0.05,            self._pub_girar_180)
        self._dur_timer = self.create_timer(self.GIRO_180_DUR, self._fin_giro_180)

    def on_enter_RETROCEDIENDO(self):
        """Avanza recto durante 2× el tiempo de navegación."""
        nav_dur   = time.time() - self._nav_start_time - self.GIRO_180_DUR
        retro_dur = max(2.0 * nav_dur, 1.0)   # mínimo 1 s por seguridad
        self.get_logger().info(
            f'[FSM] → RETROCEDIENDO | Avanzando {retro_dur:.1f}s a {self.RETRO_VEL} m/s'
        )
        self._pub_timer = self.create_timer(0.05,    self._pub_retroceder)
        self._dur_timer = self.create_timer(retro_dur, self._fin_retroceso)

    def on_enter_DONE(self):
        """Misión completada. Para el robot."""
        self.pub.publish(Twist())
        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║      🏁  MISION COMPLETADA          ║')
        self.get_logger().info('╚══════════════════════════════════════╝')

    # ─── TICKS de publicación por estado ───────────────────────────────────────

    def _tick_buscando(self):
        """Solo detecta el ArUco (sin girar) y loguea si lo ve."""
        marker_cx, marker_w, img_cx = self._get_vision()

        if marker_cx is not None:
            self.get_logger().info(
                f'  ✅ ArUco {self.TARGET_ID} DETECTADO! ancho={marker_w:.0f}px  cx={marker_cx:.0f}px'
            )
            # (giro desactivado temporalmente para pruebas)
            # self._cancel_timers()
            # self.pub.publish(Twist())
            # self.aruco_detectado()   # → NAVEGANDO
        else:
            self.get_logger().info(
                '  ❌ ArUco no visible...',
                throttle_duration_sec=1.0
            )

        # Robot parado
        self.pub.publish(Twist())

    def _tick_navegando(self):
        """Control proporcional: centra el marker y avanza. Si está cerca → llegada."""
        marker_cx, marker_w, img_cx = self._get_vision()

        if marker_cx is None:
            self.pub.publish(Twist())
            self.get_logger().warn('ArUco perdido, esperando...', throttle_duration_sec=1.0)
            return

        error_x = marker_cx - img_cx    # + = marker a la derecha, - = izquierda

        # Llegada: marker suficientemente grande (cerca)
        if marker_w >= self.CLOSE_PX:
            self._cancel_timers()
            self.pub.publish(Twist())
            self.get_logger().info(f'  Llegada! marker_w={marker_w:.0f}px >= {self.CLOSE_PX}px')
            self.llegada()             # → GIRANDO_180
            return

        # Control proporcional: primero centrar, luego avanzar
        cmd           = Twist()
        cmd.angular.z = -self.KP_ANG * error_x
        if abs(error_x) < 30:
            cmd.linear.x = self.NAV_VEL_LIN
        self.pub.publish(cmd)

        self.get_logger().info(
            f'  err_x={error_x:.0f}px  marker_w={marker_w:.0f}px',
            throttle_duration_sec=0.3
        )

    def _pub_girar_180(self):
        cmd = Twist()
        cmd.angular.z = self.GIRO_VEL
        self.pub.publish(cmd)

    def _pub_retroceder(self):
        cmd = Twist()
        cmd.linear.x = self.RETRO_VEL
        self.pub.publish(cmd)

    # ─── FIN de estados temporizados ───────────────────────────────────────────

    def _fin_giro_180(self):
        self._cancel_timers()
        self.pub.publish(Twist())
        self.giro_completo()           # → RETROCEDIENDO

    def _fin_retroceso(self):
        self._cancel_timers()
        self.pub.publish(Twist())
        self.retroceso_listo()         # → DONE

    # ─── DESTRUCTOR ────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._cam_running = False
        self._cam_thread.join(timeout=2.0)
        super().destroy_node()


# ─── MAIN ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FSMArucoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
