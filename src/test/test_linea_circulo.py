#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

def publish_circular_path():
    rospy.init_node("circular_path_publisher", anonymous=True)
    
    wrist_pub = rospy.Publisher("/wrist_point", Float64MultiArray, queue_size=10)
    elbow_pub = rospy.Publisher("/elbow_point", Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # Frecuencia de publicación en Hz

    # Parámetros del círculo
    radius = 0.1
    center_x = 0.57
    center_y = -0.04
    z_wrist = 0.25
    z_elbow = 0.5
    num_steps = 100  # Número de pasos para la trayectoria circular

    # Generar una serie de puntos en el círculo
    theta = np.linspace(0, 2 * np.pi, num_steps)
    x_positions = center_x + radius * np.cos(theta)
    y_positions = center_y + radius * np.sin(theta)

    # Publicar la secuencia de puntos en los tópicos
    for x, y in zip(x_positions, y_positions):
        # Crear mensaje para la posición deseada de la muñeca
        wrist_msg = Float64MultiArray()
        wrist_msg.data = [x, y, z_wrist]
        wrist_pub.publish(wrist_msg)

        # Crear mensaje para la posición deseada del codo (fijo en este caso)
        elbow_msg = Float64MultiArray()
        elbow_msg.data = [0.3, center_y, z_elbow]
        elbow_pub.publish(elbow_msg)

        rate.sleep()

if __name__ == "__main__":
    try:
        publish_circular_path()
    except rospy.ROSInterruptException:
        pass
