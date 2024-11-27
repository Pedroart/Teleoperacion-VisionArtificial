#!/usr/bin/env python3
import rospy
import actionlib

from PyRobotFrames import *

from std_msgs.msg import String
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState


class ur5control():

    # Parámetros del robot
    joint_names = [
        'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
    ]
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

    dh_params = dhParameters(theta, d, a, alpha, kind)
    

    #Configuracion articular inicial
    q0 = np.radians(np.array([-90.0, -90.0, 90.0, -90.0, -90.0, 0.0]))
    xyz = np.array([0, -0.4869, 0.431859])
    flagSend = True
    joint_positions_actual = q0

    #Ros Parámetros
    freq = 500
    t = 1
    dt = 1.0/freq

    def  __init__(self):
        
        rospy.init_node("UR5Control", anonymous=True)
        
        self.robot_client = actionlib.SimpleActionClient('/ur5_robot/pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        print("Waiting for server...")
        self.robot_client.wait_for_server()
        print("Connected to server")

        # Suscriptores
        rospy.Subscriber("/xyz_target", Point, self.xyz_callback)
        #rospy.Subscriber('/ur5_robot/joint_states', JointState, self.joint_states_callback)

        self.robot = robot(self.dh_params, self.q_lim)
        self.robot._q = self.q0
        self.robot.update()
        
        self.g = FollowJointTrajectoryGoal()
        self.g.trajectory = JointTrajectory()
        self.g.trajectory.joint_names = self.joint_names
        
        self.rate = rospy.Rate(self.freq)
        
        self.sendq()
        rospy.sleep(1)
    
    @timeit
    def spin(self):
        while not rospy.is_shutdown():
            if self.flagSend: 
                rospy.loginfo(f"Coordenadas XYZ recibidas: {self.xyz}")
                
                self.flagSend = False    
                
                q = self.robot.ikine_task(xdes=self.xyz)
                self.robot.limit_joint_pos()

                self.sendq()
                #rospy.loginfo(f"Coordenadas XYZ Actual: {self.robot.tLWrist}")
                #rospy.loginfo(f"Coordenadas XYZ Efector: {self.robot.tLEndEfector}")

            self.rate.sleep()
    
    @timeit
    def sendq(self):
        self.g.trajectory.points = [JointTrajectoryPoint(positions=self.robot._q.tolist(),time_from_start=rospy.Duration(self.t))]
        self.robot_client.send_goal(self.g)
        self.robot_client.wait_for_result()
        self.t += self.dt

    def joint_states_callback(self, msg):
        """Callback para recibir los estados de las articulaciones."""
        self.joint_positions_actual = np.array(msg.position)
        #rospy.loginfo(f"Coordenadas QRobot recibidas: {self.joint_positions_actual}")

    def xyz_callback(self, msg):
        """Callback para recibir coordenadas XYZ."""
        self.xyz = np.array([msg.x, msg.y, msg.z])
        self.flagSend = True
        #rospy.loginfo(f"Coordenadas XYZ recibidas: {self.xyz}")



if __name__ == '__main__':
    try:
        node = ur5control()
        node.spin()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido.")