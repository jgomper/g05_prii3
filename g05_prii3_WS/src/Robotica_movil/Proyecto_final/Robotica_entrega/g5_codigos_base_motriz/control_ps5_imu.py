import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import serial
import time
import os
import threading
import pygame  # La librería mágica para leer el mando

class RobotControlMando(Node):
    def __init__(self):
        super().__init__('control_mando_node')

        # --- VARIABLES ---
        self.ventosa_encendida = False
        self.yaw_actual = 0.0 # Guardamos el giro aunque no lo usemos visualmente

        # --- 1. CONEXION ARDUINO ---
        os.system("sudo fuser -k /dev/ttyACM0")
        try:
            self.arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            time.sleep(2)
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            self.arduino.write(b'S') 
            
            # --- NUEVO: HILO DE ESCUCHA PARA VACIAR EL BUFFER DE LA IMU ---
            self.hilo_lectura = threading.Thread(target=self.leer_serie_arduino, daemon=True)
            self.hilo_lectura.start()
            
        except Exception as e:
            self.get_logger().error(f"No se detecto el Arduino: {e}")
            exit()

        # --- 2. CONFIGURACION BRAZO (ROS 2) ---
        self.publisher = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.POS_BUSCADOR = [0.13345632854584366, 1.541650691824862, -1.7088545977048113]
        self.POS_AGARRE = [1.535515, 0.483204, -1.98190]

    # ================================================================
    # VACIADO DEL BUFFER (IMU BNO055)
    # ================================================================
    def leer_serie_arduino(self):
        """Hilo daemon que lee constantemente el YAW del Arduino para que no se sature el puerto Serie."""
        while True:
            try:
                if self.arduino.in_waiting > 0:
                    linea = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if linea.startswith("Y:"):
                        self.yaw_actual = float(linea.split(":")[1])
            except Exception:
                pass
            time.sleep(0.005)

    def mover_brazo(self, posiciones, segundos):
        msg = JointTrajectory()
        msg.joint_names = ['Junta1', 'Junta2', 'Junta3']
        punto = JointTrajectoryPoint()
        punto.positions = posiciones
        punto.time_from_start = Duration(sec=segundos, nanosec=0)
        msg.points.append(punto)
        self.publisher.publish(msg)
        time.sleep(segundos + 0.5)

    def enviar_velocidad(self, vel_izq, vel_der):
        # Para el mando ampliamos el límite de marcha atrás a -60 para tener agilidad
        vel_izq = max(-60, min(70, int(vel_izq)))
        vel_der = max(-60, min(70, int(vel_der)))
        comando = f"M {vel_izq} {vel_der}\n"
        self.arduino.write(comando.encode())
        return vel_izq, vel_der

    def parar_motores(self):
        self.arduino.write(b'S')

    def alternar_ventosa(self):
        if self.ventosa_encendida:
            self.arduino.write(b'v')
            self.ventosa_encendida = False
            print("[VENTOSA] APAGADA")
        else:
            self.arduino.write(b'V')
            self.ventosa_encendida = True
            print("[VENTOSA] ENCENDIDA")

    def ejecutar_pick_and_place(self):
        print("\n--- INICIANDO SECUENCIA PICK & PLACE ---")
        self.parar_motores() # Frenamos el robot por seguridad
        
        print("1. Bajando a posición de agarre...")
        self.mover_brazo(self.POS_AGARRE, 2)
        
        print("2. Activando ventosa...")
        self.arduino.write(b'V')
        self.ventosa_encendida = True
        time.sleep(1.0)
        
        print("3. Subiendo a posición de transporte...")
        self.mover_brazo(self.POS_BUSCADOR, 2)
        print("--- SECUENCIA COMPLETADA. MANDO REACTIVADO ---\n")


def main():
    rclpy.init()
    robot = RobotControlMando()

    print("\n[INFO] Inicializando sistema de mando...")
    
    # --- INICIALIZACIÓN DEL MANDO (PYGAME) ---
    os.environ["SDL_VIDEODRIVER"] = "dummy" # Evita errores de interfaz gráfica en la Jetson
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[ERROR] No se ha detectado ningún mando de PS5 por Bluetooth.")
        print("Asegúrate de que está emparejado y conectado a la Jetson.")
        robot.destroy_node()
        rclpy.shutdown()
        return

    mando = pygame.joystick.Joystick(0)
    mando.init()
    print(f"\n[ÉXITO] Mando conectado: {mando.get_name()}")
    print("==================================================")
    print(" CONTROLES ACTIVADOS:")
    print(" - Joystick Izq: Conducir robot (Avanzar/Girar)")
    print(" - Botón X (Cruz): Ejecutar Pick & Place")
    print(" - Botón Cuadrado: Alternar Ventosa (Manual)")
    print(" - Botón Círculo: Freno de emergencia")
    print("==================================================\n")

    robot.mover_brazo(robot.POS_BUSCADOR, 2)

    try:
        while rclpy.ok():
            # Refrescamos los eventos del mando
            pygame.event.pump()

            # --- LECTURA DE BOTONES ---
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    boton = event.button
                    
                    if boton == 0: # Botón X (Cruz) en la mayoría de mapeos de Ubuntu
                        robot.ejecutar_pick_and_place()
                        
                    elif boton == 1: # Botón Círculo
                        print("[INFO] Freno de emergencia activado.")
                        robot.parar_motores()
                        
                    elif boton == 3: # Botón Cuadrado
                        robot.alternar_ventosa()

            # --- LECTURA DEL JOYSTICK (CONDUCCIÓN) ---
            # El eje 1 es Arriba/Abajo. En Pygame, Arriba es -1.0 y Abajo es 1.0
            # El eje 0 es Izquierda/Derecha. Izquierda es -1.0 y Derecha es 1.0
            eje_y = mando.get_axis(1) 
            eje_x = mando.get_axis(0)

            # Zona muerta: Si el joystick está casi en el centro, lo ignoramos para que el robot no "tiemble"
            if abs(eje_y) < 0.15: eje_y = 0.0
            if abs(eje_x) < 0.15: eje_x = 0.0

            # --- MATEMÁTICAS DEL CONTROL DIFERENCIAL ---
            # Invertimos el eje_y para que empujar hacia adelante (negativo) sea velocidad positiva
            velocidad_base = -eje_y * 60  
            
            # El giro reparte la potencia entre las ruedas
            giro = eje_x * 40  

            vel_izq = velocidad_base + giro
            vel_der = velocidad_base - giro

            # Solo enviamos velocidad si realmente estamos moviendo el joystick
            if abs(vel_izq) > 0 or abs(vel_der) > 0:
                v_i, v_d = robot.enviar_velocidad(vel_izq, vel_der)
                print(f"Conduciendo -> Izq: {v_i:3d} | Der: {v_d:3d} | Yaw: {robot.yaw_actual:.1f}º    ", end='\r')
            else:
                robot.parar_motores()

            rclpy.spin_once(robot, timeout_sec=0.01)
            time.sleep(0.05) # Pequeña pausa para no saturar el puerto Serie del Arduino

    except KeyboardInterrupt:
        print("\n[INFO] Desconectando mando y apagando robot...")
    finally:
        robot.parar_motores()
        if hasattr(robot, 'arduino'):
            robot.arduino.write(b'v') # Apagar ventosa
        robot.destroy_node()
        rclpy.shutdown()
        pygame.quit()

if __name__ == '__main__':
    main()
