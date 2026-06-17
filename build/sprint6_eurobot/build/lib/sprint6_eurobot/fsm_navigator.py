"""
Sprint 6 - FSM Navigator (Forma pura de transitions)
======================================================
Implementación SIN control_loop, SIN if/elif, SIN getattr.

Toda la lógica vive en los callbacks on_enter_* de transitions.
Cada estado crea sus propios timers ROS2 al entrar y los cancela al salir.
La FSM de transitions es la que gobierna TODO el flujo.

Estados: INIT → AVANZAR ⇄ GIRAR → DONE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from transitions import Machine, State


# ─── ESTADOS ───────────────────────────────────────────────────────────────────
STATES = [
    State(name='INIT'),
    State(name='AVANZAR'),
    State(name='GIRAR'),
    State(name='DONE', final=True),
]

# ─── TRANSICIONES ──────────────────────────────────────────────────────────────
TRANSITIONS = [
    {'trigger': 'arrancar',
     'source': 'INIT',    'dest': 'AVANZAR'},

    {'trigger': 'avance_listo',
     'source': 'AVANZAR', 'dest': 'GIRAR',    'conditions': ['faltan_lados']},

    {'trigger': 'avance_listo',
     'source': 'AVANZAR', 'dest': 'DONE'},

    {'trigger': 'giro_listo',
     'source': 'GIRAR',   'dest': 'AVANZAR',  'conditions': ['faltan_lados']},

    {'trigger': 'giro_listo',
     'source': 'GIRAR',   'dest': 'DONE'},
]


# ─── NODO ROS2 ─────────────────────────────────────────────────────────────────
class FSMCuadradoNode(Node):

    def __init__(self):
        super().__init__('fsm_navigator')

        # Parámetros
        self.declare_parameter('avance_vel',      0.15)
        self.declare_parameter('avance_duracion', 4.0)
        self.declare_parameter('giro_vel',        0.50)
        self.declare_parameter('giro_duracion',   3.14)

        self.AVANCE_VEL      = self.get_parameter('avance_vel').value
        self.AVANCE_DURACION = self.get_parameter('avance_duracion').value
        self.GIRO_VEL        = self.get_parameter('giro_vel').value
        self.GIRO_DURACION   = self.get_parameter('giro_duracion').value

        self.lados_completados = 0
        self.TOTAL_LADOS       = 4

        # Timers activos del estado actual (se crean/cancelan en on_enter/on_exit)
        self._pub_timer = None
        self._dur_timer = None

        # Publisher
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ── FSM con transitions ──────────────────────────────────────────────
        self.machine = Machine(
            model=self,
            states=STATES,
            transitions=TRANSITIONS,
            initial='INIT',
            ignore_invalid_triggers=True,
        )

        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║   Sprint 6 - FSM Cuadrado (I3L7)    ║')
        self.get_logger().info('╚══════════════════════════════════════╝')
        self.get_logger().info(f'  Avance : {self.AVANCE_VEL} m/s  x  {self.AVANCE_DURACION} s')
        self.get_logger().info(f'  Giro   : {self.GIRO_VEL} rad/s  x  {self.GIRO_DURACION} s')
        self.get_logger().info(f'  Namespace: {self.get_namespace()}')

        # transitions no llama on_enter para el estado inicial → arrancar manual
        self.arrancar()

    # ─── CONDICIÓN ─────────────────────────────────────────────────────────────
    @property
    def faltan_lados(self):
        """True mientras no se hayan completado los 4 lados."""
        return self.lados_completados < self.TOTAL_LADOS

    # ─── CALLBACKS on_enter — aquí vive TODA la lógica ─────────────────────────

    def on_enter_AVANZAR(self):
        lado = self.lados_completados + 1
        self.get_logger().info(
            f'[FSM] → AVANZAR | Lado {lado}/{self.TOTAL_LADOS} '
            f'({self.AVANCE_DURACION}s a {self.AVANCE_VEL} m/s)'
        )
        # Timer de publicación: publica cmd_vel a 20 Hz mientras avanza
        self._pub_timer = self.create_timer(0.05, self._pub_avanzar)
        # Timer de duración: al expirar, finaliza el avance
        self._dur_timer = self.create_timer(self.AVANCE_DURACION, self._fin_avanzar)

    def on_enter_GIRAR(self):
        self.lados_completados += 1
        self.get_logger().info(
            f'[FSM] → GIRAR | Giro #{self.lados_completados} '
            f'({self.GIRO_DURACION}s a {self.GIRO_VEL} rad/s)'
        )
        # Timer de publicación: publica cmd_vel a 20 Hz mientras gira
        self._pub_timer = self.create_timer(0.05, self._pub_girar)
        # Timer de duración: al expirar, finaliza el giro
        self._dur_timer = self.create_timer(self.GIRO_DURACION, self._fin_girar)

    def on_enter_DONE(self):
        self.pub.publish(Twist())   # parar el robot
        self.get_logger().info('╔══════════════════════════════════════╗')
        self.get_logger().info('║      🏁  CUADRADO COMPLETADO        ║')
        self.get_logger().info('╚══════════════════════════════════════╝')

    # ─── PUBLICADORES por estado ────────────────────────────────────────────────

    def _pub_avanzar(self):
        cmd = Twist()
        cmd.linear.x = self.AVANCE_VEL
        self.pub.publish(cmd)

    def _pub_girar(self):
        cmd = Twist()
        cmd.angular.z = self.GIRO_VEL
        self.pub.publish(cmd)

    # ─── FIN de cada estado → cancela timers y dispara transición ──────────────

    def _fin_avanzar(self):
        self._pub_timer.cancel()
        self._dur_timer.cancel()
        self.pub.publish(Twist())   # parar antes de girar
        self.avance_listo()         # transitions decide: → GIRAR o → DONE

    def _fin_girar(self):
        self._pub_timer.cancel()
        self._dur_timer.cancel()
        self.pub.publish(Twist())   # parar antes del siguiente lado
        self.giro_listo()           # transitions decide: → AVANZAR o → DONE


# ─── MAIN ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = FSMCuadradoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())   # seguridad: parar al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
