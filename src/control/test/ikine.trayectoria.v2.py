import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import sympy as sp
from PyRobotFrames import *

if __name__ == "__main__":
    rospy.init_node("joint_positions_publisher", anonymous=True)
    
    # Crear un publicador para el tópico /joint_positions
    joint_pub = rospy.Publisher("/joint_positions", Float64MultiArray, queue_size=10)

    # Parámetros de ejemplo
    theta = [sp.pi, 0, 0, 0, 0, 0]
    d = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
    a = [0, -0.42500, -0.39225, 0, 0, 0]
    alpha = [sp.pi / 2, 0, 0, sp.pi / 2, -sp.pi / 2, 0]
    q_lim = np.around(np.radians(np.array([
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-180.0, 180.0],
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-360.0, 360.0]
    ])), 4)
    kind = ['R', 'R', 'R', 'R', 'R', 'R']

    # Creación del objeto
    dh_params = dhParameters(theta, d, a, alpha, kind)
    ur5_robot = robot(dh_params, q_lim)

    # Definir la pose inicial y final
    start_pose = np.array([0.4, 0.0, 0.1, 0, 1, 0, 0])  # Pose inicial
    end_pose = np.array([0.6, 0.0, 0.1, 0, 1, 0, 0])    # Pose final
    
    # Generar una lista de puntos intermedios
    num_steps = 50  # Número de pasos en la trayectoria
    trajectory = np.linspace(start_pose, end_pose, num_steps)

    rate = rospy.Rate(100)  # Frecuencia de publicación en Hz

    for pose in trajectory:
        # Resolver la cinemática inversa para cada punto en la trayectoria
        q = ur5_robot.ikine_task(pose)

        if q is None:
            rospy.logwarn("No se pudo resolver la cinemática inversa para la pose: {}".format(pose))
            continue

        # Crear un mensaje Float64MultiArray
        joint_positions_msg = Float64MultiArray()
        joint_positions_msg.data = q  # Asignar posiciones de articulaciones
        
        # Publicar el mensaje
        joint_pub.publish(joint_positions_msg)
        rospy.loginfo(f"Publicando posiciones articulares: {q}")
        rate.sleep()
