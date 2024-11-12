#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class RealTimePlot:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('realtime_plot', anonymous=True)
        
        # Variables para almacenar datos
        self.x_data = []
        self.y_data = []
        self.z_data = []

        # Crear la figura y los ejes
        self.fig, (self.ax_xy, self.ax_z) = plt.subplots(2, 1, figsize=(8, 8))
        
        # Configuración del gráfico XY
        self.ax_xy.set_title("Plano XY")
        self.ax_xy.set_xlim(-1024, 1024)  # Ajusta los límites según tus datos
        self.ax_xy.set_ylim(-1024, 1024)
        self.ax_xy.set_xlabel("X")
        self.ax_xy.set_ylabel("Y")
        self.xy_scatter, = self.ax_xy.plot([], [], 'bo', markersize=8)

        # Configuración del gráfico de barras Z
        self.ax_z.set_title("Coordenada Z")
        self.ax_z.set_ylim(0, 1024)  # Ajusta los límites según tus datos
        self.ax_z.set_ylabel("Z")
        self.z_bar = self.ax_z.bar(["Z"], [0], color='blue')

        # Suscriptor al tópico
        rospy.Subscriber('pointer', Point, self.callback)

        # Inicializar la animación
        self.anim = FuncAnimation(self.fig, self.update_plot, interval=100)

    def callback(self, msg):
        """
        Callback para recibir los datos del tópico `pointer`.
        """
        # Guardar los datos recibidos
        self.x_data.append(msg.x)
        self.y_data.append(msg.y)
        self.z_data.append(msg.z)

        # Limitar los datos almacenados para evitar un uso excesivo de memoria
        if len(self.x_data) > 100:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.z_data.pop(0)

    def update_plot(self, frame):
        """
        Función de actualización para la animación.
        """
        # Actualizar el gráfico XY
        self.xy_scatter.set_data(self.x_data, self.y_data)

        # Ajustar límites dinámicamente si es necesario
        self.ax_xy.set_xlim(min(self.x_data, default=-1), max(self.x_data, default=1))
        self.ax_xy.set_ylim(min(self.y_data, default=-1), max(self.y_data, default=1))

        # Actualizar el gráfico de barras Z
        if self.z_data:
            self.z_bar[0].set_height(self.z_data[-1])  # Actualiza la barra con el último valor de Z
        else:
            self.z_bar[0].set_height(0)

        return self.xy_scatter, self.z_bar

    def run(self):
        """
        Ejecuta la visualización en tiempo real.
        """
        plt.show()

if __name__ == '__main__':
    try:
        plotter = RealTimePlot()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
