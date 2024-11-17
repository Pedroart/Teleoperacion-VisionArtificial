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
    theta   = [sp.pi, 0, 0, 0, 0, 0]
    d       = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
    a       = [0, -0.42500, -0.39225, 0, 0, 0]
    alpha   = [sp.pi / 2, 0, 0, sp.pi / 2, -sp.pi / 2, 0]
    q_lim = np.around(np.radians(np.array([[-360.0, 360.0],[-360.0, 360.0],[-180.0, 180.0],[-360.0, 360.0],[-360.0, 360.0],[-360.0, 360.0]])),4)
    kind    = ['R', 'R', 'R', 'R', 'R', 'R']

    # Creación del objeto
    dh_params = dhParameters(theta, d, a, alpha, kind)
    ur5_robot = robot(dh_params,q_lim)
    
    
    q = ur5_robot.ikine_task(np.array([0.6, 0.0, 0.1, 0, 1, 0, 0]))

    # Crear un mensaje JointState
    joint_state_msg = JointState()
    
    # Llenar el mensaje con datos
    joint_state_msg.header.stamp = rospy.Time.now()  # Marca de tiempo actual
    joint_state_msg.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]  # Cambiar según los nombres de tus articulaciones
    joint_state_msg.position = q  # Asignar posiciones de articulaciones
    
    # Publicar el mensaje en un bucle
    rate = rospy.Rate(10)  # Frecuencia de publicación en Hz
    while not rospy.is_shutdown():
        joint_state_msg.header.stamp = rospy.Time.now()  # Actualizar la marca de tiempo
        joint_pub.publish(joint_state_msg)
        rate.sleep()