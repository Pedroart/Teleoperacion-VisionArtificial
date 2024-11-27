#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
import numpy as np

def publish_trajectory_points():
    """
    Publica una secuencia de puntos en el tópico /xyz_target para probar el seguimiento de trayectoria.
    """
    rospy.init_node("trajectory_test_node", anonymous=True)
    
    # Crear el publicador para el tópico /xyz_target
    trajectory_pub = rospy.Publisher("/xyz_target", Point, queue_size=10)
    rate = rospy.Rate(30)  # Frecuencia de publicación (1 Hz)
    
    # Puntos de inicio y fin de la trayectoria
    start_pose = np.array([0.1,-0.58617022,0.15121768])  # Pose inicial (x, y, z)
    end_pose = np.array([0.1,-0.68617022,0.15121768])    # Pose final (x, y, z)
    
    # Número de pasos en la trayectoria
    num_steps = 10
    trajectory = np.linspace(start_pose, end_pose, num_steps)  # Interpolar entre los puntos
    
    rospy.loginfo("Publicando trayectoria interpolada en /xyz_target...")
    
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
