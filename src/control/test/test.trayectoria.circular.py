#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import numpy as np

def publish_trajectory_points():
    """
    Publica una secuencia de puntos en el tópico /xyz_target para probar el seguimiento de trayectoria circular.
    """
    rospy.init_node("trajectory_test_node", anonymous=True)
    
    # Crear el publicador para el tópico /xyz_target
    trajectory_pub = rospy.Publisher("/xyz_target", Point, queue_size=10)
    rate = rospy.Rate(2)  # Frecuencia de publicación (30 Hz)
    
    # Parámetros del círculo
    center_x = 0.1  # Centro en x
    center_y = -0.7  # Centro en y
    radius = 0.1  # Radio del círculo
    num_steps = 100  # Número de puntos en la circunferencia
    z_value = 0.15121768  # Coordenada z constante

    # Calcular los puntos de la circunferencia
    theta = np.linspace(0, 2 * np.pi, num_steps)  # Ángulos de 0 a 2π
    trajectory = np.array([
        [center_x + radius * np.cos(t), center_y + radius * np.sin(t), z_value] for t in theta
    ])
    
    rospy.loginfo("Publicando trayectoria circular en /xyz_target...")
    
    while not rospy.is_shutdown():
        for point in trajectory:
            # Crear un mensaje Point para cada punto de la trayectoria
            msg = Point(x=point[0], y=point[1], z=point[2])
            rospy.loginfo(f"Publicando punto: x={msg.x}, y={msg.y}, z={msg.z}")
            trajectory_pub.publish(msg)
            rate.sleep()  # Espera antes de publicar el siguiente punto

if __name__ == "__main__":
    try:
        publish_trajectory_points()
    except rospy.ROSInterruptException:
        pass
