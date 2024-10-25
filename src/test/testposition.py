#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def publish_fixed_trajectory():
    rospy.init_node('trajectory_publisher', anonymous=True)
    pub = rospy.Publisher('desired_position', Float64MultiArray, queue_size=10)

    # Definir tres puntos fijos que cumplen con los requisitos
    trajectory = [
        np.array([0.5, 0.3, 0.4]),
        np.array([0.5, 0.3, 0.4]),
        np.array([0.6, 0.2, 0.5]),
        np.array([0.8, 0.1, 0.6])
    ]
    orientation = np.array([1.57079633, 0, 1.57079633])  # Orientación fija

    for point in trajectory:
        # Combina la posición con la orientación
        pose = np.concatenate((point, orientation))

        # Crear y publicar el mensaje
        msg = Float64MultiArray()
        msg.data = pose
        pub.publish(msg)

        rospy.loginfo(f"Published fixed point in trajectory: {pose}")

        # Delay de 2 segundos entre puntos
        rospy.sleep(15)

if __name__ == '__main__':
    try:
        publish_fixed_trajectory()
    except rospy.ROSInterruptException:
        pass
