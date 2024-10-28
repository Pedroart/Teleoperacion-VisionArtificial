#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

def publish_linear_path():
    rospy.init_node("linear_path_publisher", anonymous=True)
    
    wrist_pub = rospy.Publisher("/wrist_point", Float64MultiArray, queue_size=10)
    elbow_pub = rospy.Publisher("/elbow_point", Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # Frecuencia de publicación en Hz

    # Posiciones iniciales y finales
    start_x = 0.3
    end_x = 0.8
    num_steps = 100  # Número de pasos para el movimiento en línea recta

    # Posiciones iniciales en Y y Z
    y = 0
    z_wrist = 0.3
    z_elbow = 0.5

    # Generar una serie de puntos en línea recta de 0.3 a 0.8 en el eje X
    x_positions = np.linspace(start_x, end_x, num_steps)

    # Publicar la secuencia de puntos en los tópicos
    for x in x_positions:
        # Crear mensaje para la posición deseada de la muñeca
        wrist_msg = Float64MultiArray()
        wrist_msg.data = [x, y, z_wrist]
        wrist_pub.publish(wrist_msg)

        # Crear mensaje para la posición deseada del codo
        elbow_msg = Float64MultiArray()
        elbow_msg.data = [0.3, y, z_elbow]  # Mantener el codo fijo
        elbow_pub.publish(elbow_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_linear_path()
    except rospy.ROSInterruptException:
        pass
