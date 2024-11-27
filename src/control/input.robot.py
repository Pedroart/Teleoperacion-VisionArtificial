#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys
import select
import termios
import tty
import os
from geometry_msgs.msg import Point

class StateController:
    
    pointer_data = None
    def __init__(self):

        
        rospy.init_node('state_controller_node', anonymous=True)

        # Inicializar los estados
        self.start_state = False
        self.record_state = False

        # Publicador para enviar el estado actual si es necesario
         # Publicadores y suscriptores
        self.state_pub = rospy.Publisher('/state_info', String,queue_size=10)
        self.target_pub = rospy.Publisher('/xyz_target', Point,queue_size=10)
        self.pointer_sub = rospy.Subscriber('/pointer', Point, self.pointer_callback)


        rospy.loginfo("Presiona 'Enter' para cambiar entre Start/Stop.")
        rospy.loginfo("Presiona 'R' para cambiar entre Record/Stop Recording.")
        rospy.loginfo("Presiona 'Ctrl+C' para salir.")

        self.loop()

    def pointer_callback(self, data):
        """Callback para recibir datos del tópico /pointer."""
        self.pointer_data = data

    def clear_screen(self):
        """Limpia la pantalla de la terminal."""
        os.system('clear')  # 'cls' en Windows, 'clear' en UNIX/Linux/Mac

    def display_interface(self):
        """Muestra la interfaz en la consola con los estados coloreados."""
        self.clear_screen()
        # Colores ANSI
        green = "\033[92m"
        red = "\033[91m"
        reset = "\033[0m"

        # Estado de inicio/parada
        start_status = f"{green}Started{reset}" if self.start_state else f"{red}Stopped{reset}"

        # Estado de grabación
        record_status = f"{green}Recording{reset}" if self.record_state else f"{red}Not Recording{reset}"

        # Mostrar los estados
        print("===== INTERFAZ DE CONTROL =====")
        print(f"Estado general: {start_status}")
        print(f"Estado de grabación: {record_status}")
        print("===============================")

    def process_pointer(self):
        """Procesa los datos del pointer y publica en /xyz_target si está en modo 'start'."""
        if self.start_state and self.pointer_data != None :
            # Escalar los datos del pointer
            scaled_point = Point(
                x=self.pointer_data.x * (-0.1)+0.1,
                y=self.pointer_data.y * (-0.1)-0.7,
                z=self.pointer_data.z * 0.00 +0.15
            )
            # Publicar en /xyz_target
            self.target_pub.publish(scaled_point)

            

    def loop(self):
        """Loop principal para escuchar teclas y cambiar estados."""
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while not rospy.is_shutdown():
                # Mostrar la interfaz
                self.process_pointer()

                # Escucha teclas sin bloqueo
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)

                    # Manejar tecla 'Enter' para Start/Stop
                    if key == '\n':  # 'Enter' key
                        self.start_state = not self.start_state
                        state_msg = "Started" if self.start_state else "Stopped"
                        self.state_pub.publish(state_msg)
                        self.display_interface()

                    # Manejar tecla 'R' para Record/Stop Recording
                    elif key.lower() == 'r':  # 'R' or 'r'
                        self.record_state = not self.record_state
                        record_msg = "Recording Started" if self.record_state else "Recording Stopped"
                        self.state_pub.publish(record_msg)
                        self.display_interface()
        
        except KeyboardInterrupt:
            self.clear_screen()
            rospy.loginfo("Saliendo del nodo...")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

if __name__ == '__main__':
    try:
        StateController()
    except rospy.ROSInterruptException:
        pass
