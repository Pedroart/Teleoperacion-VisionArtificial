#!/usr/bin/env python3
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
    rospy.init_node("test1", disable_signals=True)

    # Crear cliente de acción
    robot_client = actionlib.SimpleActionClient('pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    print("Waiting for server...")
    robot_client.wait_for_server()
    print("Connected to server")

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    # Posición inicial
    Q0 = [0.0, -1.0, 1.7, -2.2, -1.6, 0.0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names

    # Definir posición inicial
    g.trajectory.points = [
        JointTrajectoryPoint(
            positions=Q0,
            velocities=[0] * 6,
            time_from_start=rospy.Duration(2.0)
        )
    ]

    robot_client.send_goal(g)
    robot_client.wait_for_result()
    rospy.sleep(1)

    # Bucle para modificar la posición del robot
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # Crear nueva posición basada en Q0
        new_Q = Q0[:]
        new_Q[0] -= 0.005  # Modificar solo la primera articulación

        # Crear nueva meta
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = joint_names
        g.trajectory.points = [
            JointTrajectoryPoint(
                positions=new_Q,
                velocities=[0] * 6,
                time_from_start=rospy.Duration(0.5)  # Tiempo suficiente para la trayectoria
            )
        ]

        # Enviar nueva meta y esperar resultado
        robot_client.send_goal(g)
        robot_client.wait_for_result()

        # Actualizar Q0 con la nueva posición
        Q0 = new_Q[:]
        rate.sleep()

    # Cancelar cualquier meta pendiente al salir
    robot_client.cancel_goal()
