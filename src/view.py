#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib
matplotlib.use('TkAgg')  # Asegura el uso de un backend gráfico compatible
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Variables globales para los ángulos
angulos = []

def angle_callback(data):
    global angulos
    # Extrae los ángulos del mensaje recibido
    angulos = data.data

def update_plot(frame):
    global angulos
    plt.clf()  # Limpia la gráfica actual
    if angulos:
        # Crear gráfica de barras
        plt.bar(range(len(angulos)), angulos, tick_label=jnames)
        plt.xlabel('Articulaciones')
        plt.ylabel('Ángulos (radianes)')
        plt.title('Ángulos de las Articulaciones del Robot')
    else:
        plt.text(0.5, 0.5, 'Esperando datos...', fontsize=12, ha='center')

if __name__ == "__main__":
    rospy.init_node("angle_plotter")

    # Nombres de las articulaciones
    jnames = ("head_pan", "right_j0", "right_j1", "right_j2", "right_j3",
              "right_j4", "right_j5", "right_j6")

    # Crear el Subscriber para angle_topic
    rospy.Subscriber('angle_topic', Float64MultiArray, angle_callback)

    # Configurar la gráfica
    plt.ion()  # Modo interactivo
    fig = plt.figure()
    ani = FuncAnimation(fig, update_plot, interval=1000)  # Actualiza la gráfica cada 1000 ms

    plt.show()  # Muestra la ventana gráfica

    rospy.spin()  # Mantiene el nodo en ejecución
