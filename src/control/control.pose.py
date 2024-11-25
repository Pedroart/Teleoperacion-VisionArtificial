#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import sympy as sp
from PyRobotFrames import *

joint_positions_actual = np.array([0,0,0,0,0,0])

def shortest_angular_distances(theta_array1, theta_array2):
    """
    Calcula la diferencia angular más corta entre múltiples pares de ángulos en radianes usando numpy.

    Args:
        theta_array1 (numpy.ndarray): Array de ángulos iniciales en radianes.
        theta_array2 (numpy.ndarray): Array de ángulos finales en radianes.

    Returns:
        numpy.ndarray: Array de diferencias angulares más cortas para cada par en radianes.
    """
    if theta_array1.shape != theta_array2.shape:
        raise ValueError("Los arrays de ángulos deben tener la misma forma.")
    
    delta_thetas = (theta_array2 - theta_array1 + np.pi) % (2 * np.pi) - np.pi
    return delta_thetas


def joint_states_callback(msg):
    global joint_positions_actual
    # Convierte las posiciones de las articulaciones en un numpy array
    joint_positions_actual = np.array(msg.position)

def sendq(q):
    global ur5_robot, joint_pub, joint_positions_actual
    if q is None:
        rospy.logwarn(f"No se pudo resolver la cinemática inversa para XYZ ")
        return


    # Crear y publicar el mensaje de posiciones articulares
    joint_positions_msg = Float64MultiArray()
    joint_positions_msg.data = q  # Asignar las posiciones articulares
    joint_pub.publish(joint_positions_msg)

    rospy.loginfo(f"XYZ recibido, posiciones articulares enviadas: {q}")

def xyz_callback(msg):
    """
    Callback que recibe coordenadas XYZ, calcula los valores articulares q y los publica.
    """
    global ur5_robot, joint_pub, joint_positions_actual

    # Crear la pose completa a partir de XYZ
    xyz = np.array([msg.x, msg.y, msg.z])
    pose = np.hstack((xyz, [0, 1, 0, 0]))  # Agrega orientación por defecto (eje z hacia arriba)

    ur5_robot._q = joint_positions_actual

    # Resolver la cinemática inversa
    q = ur5_robot.ikine_task(xyz,sendq)



if __name__ == "__main__":
    rospy.init_node("xyz_to_joint_positions", anonymous=True)

    # Crear un publicador para el tópico /joint_positions
    joint_pub = rospy.Publisher("/joint_positions", Float64MultiArray, queue_size=10)
    
    # Crear un suscriptor al tópico /xyz_target
    rospy.Subscriber("/xyz_target", Point, xyz_callback)
    rospy.Subscriber('/ur5_robot/joint_states', JointState, joint_states_callback)
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

    # Creación del objeto robot
    dh_params = dhParameters(theta, d, a, alpha, kind)
    ur5_robot = robot(dh_params, q_lim)

    rospy.loginfo("Nodo activo, esperando coordenadas XYZ en /xyz_target")

    # Mantener el nodo en ejecución
    rospy.spin()
