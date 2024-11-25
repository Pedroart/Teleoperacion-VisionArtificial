#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import numpy as np
import sympy as sp
from PyRobotFrames import *

def xyz_callback(msg):
    """
    Callback que recibe coordenadas XYZ, calcula los valores articulares q y los publica.
    """
    global ur5_robot, joint_pub

    # Crear la pose completa a partir de XYZ
    xyz = np.array([msg.x, msg.y, msg.z])
    pose = np.hstack((xyz, [0, 1, 0, 0]))  # Agrega orientación por defecto (eje z hacia arriba)

    # Resolver la cinemática inversa
    q = ur5_robot.ikine_task(pose)

    if q is None:
        rospy.logwarn(f"No se pudo resolver la cinemática inversa para XYZ: {xyz}")
        return

    # Crear y publicar el mensaje de posiciones articulares
    joint_positions_msg = Float64MultiArray()
    joint_positions_msg.data = q  # Asignar las posiciones articulares
    joint_pub.publish(joint_positions_msg)

    rospy.loginfo(f"XYZ recibido: {xyz}, posiciones articulares enviadas: {q}")

if __name__ == "__main__":
    rospy.init_node("xyz_to_joint_positions", anonymous=True)

    # Crear un publicador para el tópico /joint_positions
    joint_pub = rospy.Publisher("/joint_positions", Float64MultiArray, queue_size=10)

    # Crear un suscriptor al tópico /xyz_target
    rospy.Subscriber("/xyz_target", Point, xyz_callback)

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
