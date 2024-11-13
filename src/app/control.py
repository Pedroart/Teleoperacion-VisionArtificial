#!/usr/bin/env python3
import time
#import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
import rospy
import sympy as sp
import numpy as np
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray

def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Execution time in milliseconds
        print(f"{func.__name__}: {execution_time:.2f} ms")
        return result
    return wrapper

class Robot:
    def __init__(self):
        self.q = sp.symbols('q1 q2 q3 q4 q5 q6')
        self.params_dh = [
            [self.q[0]+sp.pi, 0.089159, 0, sp.pi / 2],       # Joint 1
            [self.q[1], 0, -0.42500, 0],    # Joint 2
            [self.q[2], 0, -0.39225, 0],              # Joint 3
            [self.q[3], 0.10915, 0, sp.pi / 2],  # Joint 4
            [self.q[4], 0.09465, 0, -sp.pi / 2],       # Joint 5
            [self.q[5], 0.0823, 0, 0]                 # Joint 6
        ]

        self._tWrist_symbolic = self.cinematica_directa(self.params_dh)
        self._jWrist_symbolic = self.jacobiano(self.params_dh)

        self.tWrist_func = sp.lambdify(self.q, self._tWrist_symbolic, "numpy")
        self.jWrist_func = sp.lambdify(self.q, self._jWrist_symbolic, "numpy")

        self.q_values = np.array([0,0,0,0,0,0])
        self.update()

    #@timeit
    def update(self):
        self.tWrist = self.tWrist_func(*self.q_values)
        self.jWrist = self.jWrist_func(*self.q_values)

    def set_q_values(self, q_array):
        if len(q_array) != 6:
            raise ValueError("Input array must have exactly 6 elements.")
        self.q_values = q_array
        self.update()
    
    def cinematica_directa(self, params_dh):
        T_total = sp.eye(4)
        for param in params_dh:
            theta, d, a, alpha = param
            T_total = T_total @ self.dh_matrix(theta, d, a, alpha)
        return T_total
    
    def dh_matrix(self, theta, d, a, alpha):
        return sp.Matrix([
            [sp.cos(theta), -sp.sin(theta) * sp.cos(alpha), sp.sin(theta) * sp.sin(alpha), a * sp.cos(theta)],
            [sp.sin(theta), sp.cos(theta) * sp.cos(alpha), -sp.cos(theta) * sp.sin(alpha), a * sp.sin(theta)],
            [0, sp.sin(alpha), sp.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def get_q_values(self):
        return np.array(self.q_values)

    def jacobiano(self, params_dh):
        T_total = sp.eye(4)
        p_n = []
        T_list = []
        
        for param in params_dh:
            theta, d, a, alpha = param
            T_i = self.dh_matrix(theta, d, a, alpha)
            T_total = T_total * T_i
            T_list.append(T_total)
        
        p_n = T_total[:3, 3]
        J_linear = []
        J_angular = []
        z = [sp.Matrix([0, 0, 1])]

        for i in range(len(params_dh)):
            Jv_i = p_n.diff(self.q[i])
            J_linear.append(Jv_i)
            z.append(T_total[:3, 2])
        
        J_angular = z[:len(params_dh)]
        return sp.Matrix.vstack(sp.Matrix.hstack(*J_linear), sp.Matrix.hstack(*J_angular))

robot = Robot()

def joint_state_callback(msg):
    global robot
    robot.set_q_values(msg.position)
    
def regularized_pseudoinverse(J, u=np.sqrt(0.001)):
    return J.T @ np.linalg.inv(J @ J.T + u**2 * np.eye(J.shape[0]))

def generalized_task_augmentation(q, J_tasks, errors, deltaT=1, u=np.sqrt(0.001)):
    P = np.eye(len(q))
    q_dot = np.zeros(len(q))
    
    for J, r_dot in zip(J_tasks, errors):
        Ji_hash = regularized_pseudoinverse(J, u)
        #Ji_hash = np.linalg.pinv(J)
        q_dot += P @ Ji_hash @ r_dot
        P = P @ (np.eye(len(q)) - Ji_hash @ J)

    return q + q_dot * deltaT, q_dot

def create_transformation_matrix(x, y, z):
    T = np.array([
        [0.0, -1.0, 0.0, x],
        [-1.0, 0.0, 0.0, y],
        [0.0, 0.0, -1.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return T

def calculate_pose_error(T_actual, T_desired):
    position_actual = T_actual[:3, 3]
    position_desired = T_desired[:3, 3]
    position_error = position_desired - position_actual
    R_actual = T_actual[:3, :3]
    R_desired = T_desired[:3, :3]
    R_error = R_desired @ R_actual.T  
    rotation_vector = R.from_matrix(R_error).as_rotvec()
    return np.concatenate((position_error, rotation_vector))

def joint_control_callback(msg):
    print(msg)

if __name__ == '__main__':
    
    rospy.init_node("test1", disable_signals=True)

    robot_client = actionlib.SimpleActionClient('pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    rospy.Subscriber('/joint_positions', Float64MultiArray, joint_control_callback)

    print("Waiting for server...")
    robot_client.wait_for_server()
    print("Connected to server")

    joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    Q0 = [0.0, 0.0, 0.0, -1.57, -1.57, 0.0]

    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = joint_names

    # Initial position
    g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6,time_from_start=rospy.Duration(2.0))]
    robot_client.send_goal(g)
    robot_client.wait_for_result()
    
    
    print(robot.tWrist)
    rospy.spin()

    '''rate = rospy.Rate(10)
    T_desired = create_transformation_matrix(0.25,0.0,0.0)
    while not rospy.is_shutdown():
        robot_client.cancel_goal()
        
        print(np.round(robot.tWrist, 3))
        e1= calculate_pose_error(robot.tWrist , T_desired)
        
        if np.linalg.norm(e1) > 0.01 :
            J_tasks = [robot.jWrist[0:3]]
            errors = [-e1[0:3]]

            q, _ = generalized_task_augmentation(robot.get_q_values(), J_tasks, errors, deltaT=0.01)
            
            Q0 = q.tolist()
            g.trajectory.points = [ JointTrajectoryPoint(positions=Q0, velocities=[0]*6, time_from_start=rospy.Duration(0.001))]
            robot_client.send_goal(g)
            robot_client.wait_for_result()
            
        rate.sleep()

    robot_client.cancel_goal()'''