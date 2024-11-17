import rospy
from sensor_msgs.msg import JointState
import sympy as sp
import numpy as np
from PyRobotFrames import *

if __name__ == "__main__":
    rospy.init_node("joint_state_publisher", anonymous=True)
    
    # Crear un publicador para el tópico /joint_states
    joint_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)

    # Parámetros de ejemplo
    theta = np.deg2rad([0, 270, 0, 180, 0, 180, 270])
    d = [0.317, 0.1925, 0.4, 0.1685, 0.4, 0.1363, 0.11]
    a = [0.081, 0, 0, 0, 0, 0, 8.08e-07]
    alpha = [-sp.pi/2, -sp.pi/2, -sp.pi/2, -sp.pi/2, -sp.pi/2, -sp.pi/2, 0]


    q_lim = np.around(np.radians(np.array([
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-360.0, 360.0],
        [-360.0, 360.0]
    ])), 4)
    kind = ['R', 'R', 'R', 'R', 'R', 'R', 'R']

    # Creación del objeto
    dh_params = dhParameters(theta, d, a, alpha, kind)
    ur5_robot = robot(dh_params, q_lim)

    # Crear un mensaje JointState
    joint_state_msg = JointState()
    joint_state_msg.name = [
        "head_pan", "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"
    ]
    
    # Definir la pose inicial y final
    start_pose = np.array([0.5, 0.0 , 0.0, 0, 1, 0, 0])  # Pose inicial
    end_pose = np.array([0.6, 0.0, 0.0, 0, 1, 0, 0])    # Pose final
    
    # Generar una lista de puntos intermedios
    num_steps = 100  # Número de pasos en la trayectoria
    trajectory = np.linspace(start_pose, end_pose, num_steps)

    rate = rospy.Rate(10)  # Frecuencia de publicación en Hz

    while not rospy.is_shutdown():
        for pose in trajectory:
            # Resolver la cinemática inversa para cada punto en la trayectoria
            q = [0,0,0,0,0,0,0,0]
            q[1:] = ur5_robot.ikine_task(pose)

            if q is None:
                rospy.logwarn("No se pudo resolver la cinemática inversa para la pose: {}".format(pose))
                continue

            # Llenar el mensaje con datos
            joint_state_msg.header.stamp = rospy.Time.now()  # Marca de tiempo actual
            joint_state_msg.position = q  # Asignar posiciones de articulaciones
            
            # Publicar el mensaje
            joint_pub.publish(joint_state_msg)
            rate.sleep()
