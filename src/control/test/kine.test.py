import rospy
from sensor_msgs.msg import JointState
import sympy as sp
import numpy as np
from PyRobotFrames import *

def joint_state_callback(msg, robot:robot):
    # Obtener valores de las articulaciones desde el mensaje
    joint_positions = msg.position[:6]  # Asegúrate de que el UR5 tiene 6 articulaciones
    print(joint_positions)
    robot._q = np.array(joint_positions)
    robot.update()

    # Calcular y mostrar la cinemática directa
    efector_pose = robot.pose
    print(f"Pose del efector final:\n{efector_pose}")


if __name__ == "__main__":
    rospy.init_node("ur5_kinematics_tester")
    
    # Parámetros de ejemplo
    theta   = [0, 0, 0, 0, 0, 0]
    d       = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]
    a       = [0, -0.42500, -0.39225, 0, 0, 0]
    alpha   = [sp.pi / 2, 0, 0, sp.pi / 2, -sp.pi / 2, 0]
    q_lim = np.around(np.radians(np.array([[-360.0, 360.0],[-360.0, 360.0],[-180.0, 180.0],[-360.0, 360.0],[-360.0, 360.0],[-360.0, 360.0]])),4)
    kind    = ['R', 'R', 'R', 'R', 'R', 'R']

    # Creación del objeto
    dh_params = dhParameters(theta, d, a, alpha, kind)
    ur5_robot = robot(dh_params,q_lim)
    
    # Suscribirse al tópico /joint_states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback, ur5_robot)

    print("Esperando datos de /joint_states...")
    rospy.spin()