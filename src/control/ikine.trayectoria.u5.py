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

    # Crear un mensaje JointState
    joint_state_msg = JointState()
    joint_state_msg.name = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint", 
        "elbow_joint", 
        "wrist_1_joint", 
        "wrist_2_joint", 
        "wrist_3_joint"
    ]
    
    # Definir la pose inicial y final
    start_pose = np.array([0.4, 0.0, 0.1, 0, 1, 0, 0])  # Pose inicial
    end_pose = np.array([0.6, 0.0, 0.1, 0, 1, 0, 0])    # Pose final
    
    # Generar una lista de puntos intermedios
    num_steps = 100  # Número de pasos en la trayectoria
    trajectory = np.linspace(start_pose, end_pose, num_steps)

    rate = rospy.Rate(10)  # Frecuencia de publicación en Hz

    while not rospy.is_shutdown():
        for pose in trajectory:
            # Resolver la cinemática inversa para cada punto en la trayectoria
            q = ur5_robot.ikine_task(pose)

            if q is None:
                rospy.logwarn("No se pudo resolver la cinemática inversa para la pose: {}".format(pose))
                continue

            # Llenar el mensaje con datos
            joint_state_msg.header.stamp = rospy.Time.now()  # Marca de tiempo actual
            joint_state_msg.position = q  # Asignar posiciones de articulaciones
            
            # Publicar el mensaje
            joint_pub.publish(joint_state_msg)
            rate.sleep()
