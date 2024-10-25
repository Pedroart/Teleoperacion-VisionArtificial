#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray

def random_step_in_sphere(current_position, max_step=0.1, radius=0.9):
    """
    Genera una nueva posición cercana a la actual, asegurándose de que
    permanece dentro de una esfera de un radio dado y que x es positiva.
    """
    while True:
        # Generar un pequeño paso aleatorio
        step = np.random.uniform(-max_step, max_step, 3)
        new_position = current_position + step

        # Asegurarse de que la posición x es positiva y está dentro de la esfera
        if new_position[0] >= 0.4 and np.linalg.norm(new_position) <= radius:
            break

    return new_position

def random_position_in_sphere(radius=0.9):
    """
    Inicializa la posición inicial aleatoriamente dentro de la esfera,
    asegurando que x sea positivo.
    """
    while True:
        position = np.random.uniform(-radius, radius, 3)
        if position[0] >= 0 and np.linalg.norm(position) <= radius:
            return position

def publish_random_positions(rate_hz=1):
    rospy.init_node('random_position_publisher', anonymous=True)
    pub = rospy.Publisher('desired_position', Float64MultiArray, queue_size=10)
    rate = rospy.Rate(rate_hz)

    # Inicializar posición aleatoria
    current_position = random_position_in_sphere()
    orientation = np.array([1.57079633, 0, 1.57079633])

    while not rospy.is_shutdown():
        # Generar una nueva posición cercana a la actual
        current_position = random_step_in_sphere(current_position)
        random_pose = np.concatenate((current_position, orientation))

        # Crear y publicar el mensaje
        msg = Float64MultiArray()
        msg.data = random_pose
        pub.publish(msg)

        rospy.loginfo(f"Published random position: {random_pose}")

        # Añadir un delay de 2 segundos
        rospy.sleep(5)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_random_positions(rate_hz=1)
    except rospy.ROSInterruptException:
        pass
