#!/usr/bin/env python3

import rospy
import actionlib
import numpy as np
from std_msgs.msg import Float64MultiArray
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Configuración inicial
joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

def send_joint_positions(data):
    """
    Callback para el tópico que recibe configuraciones articulares.
    """
    try:
        # Leer posiciones articulares del mensaje recibido
        joint_positions = data.data

        '''if robot_client.get_state() in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
            rospy.loginfo("Cancelando la meta actual...")
            robot_client.cancel_goal()'''

        # Crear y enviar una meta al controlador
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joint_names
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joint_positions, time_from_start=rospy.Duration(0.0001))
        ]

        robot_client.send_goal(g)
        robot_client.wait_for_result()
    except Exception as e:
        rospy.logerr(f"Error al enviar configuraciones articulares: {e}")

if __name__ == '__main__':
    # Inicializar nodo
    rospy.init_node("joint_position_control", disable_signals=True)

    # Crear cliente de acción
    robot_client = actionlib.SimpleActionClient('/ur5_robot/pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.loginfo("Waiting for server...")
    robot_client.wait_for_server()
    rospy.loginfo("Connected to server")

    # Suscribirse al tópico para recibir configuraciones articulares
    rospy.Subscriber('/joint_positions', Float64MultiArray, send_joint_positions,queue_size=2)

    rospy.loginfo("Nodo activo y esperando configuraciones articulares en /joint_positions")

    # Mantener el nodo vivo
    rospy.spin()
