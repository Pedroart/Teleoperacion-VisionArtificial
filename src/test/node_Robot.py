#!/usr/bin/env python3
import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        return result
    return wrapper


class Robot:
    def __init__(self):
        # Define symbolic variables and DH parameters
        self.q = sp.symbols('q1 q2 q3 q4 q5 q6 q7')
        self.params_dh = [
            [self.q[0], 0.317, 0.081, -sp.pi / 2],
            [self.q[1] - sp.pi / 2, 0.1925, 0, -sp.pi / 2],
            [self.q[2], 0.4, 0, -sp.pi / 2],
            [-self.q[3] + sp.pi, 0.1685, 0, -sp.pi / 2],
            [self.q[4], 0.4, 0, -sp.pi / 2],
            [self.q[5] + sp.pi, 0.1363, 0, -sp.pi / 2],
            [self.q[6] - sp.pi / 2, 0.11, 8.08E-07, 0]
        ]
        

        
        # Precompute symbolic forward kinematics and Jacobians
        self._tWrist_symbolic = self.cinematica_directa(self.params_dh)
        self._jWrist_symbolic = self.jacobiano(self.params_dh)

        self._tElbow_symbolic = self.cinematica_directa(self.params_dh[:4])
        self._jElbow_symbolic = self.jacobiano(self.params_dh[:4])

        # Convert symbolic expressions to fast NumPy functions
        self.tWrist_func = sp.lambdify(self.q, self._tWrist_symbolic, "numpy")
        self.jWrist_func = sp.lambdify(self.q, self._jWrist_symbolic, "numpy")
        
        self.tElbow_func = sp.lambdify(self.q, self._tElbow_symbolic, "numpy")
        self.jElbow_func = sp.lambdify(self.q, self._jElbow_symbolic, "numpy")

        # Initial q_values
        self.q_values = [0, 0, 0, 0, 0, 0, 0]

        # Update numeric matrices
        self.update()

    @timeit
    def update(self):
        # Evaluate matrices using lambdify functions
        self.tWrist = self.tWrist_func(*self.q_values)
        self.jWrist = self.jWrist_func(*self.q_values)

        self.tElbow = self.tElbow_func(*self.q_values)
        self.jElbow = self.jElbow_func(*self.q_values)

        self.xWrist_actual = self.get_pose(self.tWrist)
        self.xElbow_actual = self.get_pose(self.tElbow)

    @timeit
    def get_pose(self, T):
        t_actual = T[:3, 3]
        R_actual = T[:3, :3]
        r_actual = R.from_matrix(R_actual).as_euler('xyz')
        return np.concatenate((t_actual, r_actual))

    def set_q_values(self, q_array):
        if len(q_array) != 7:
            raise ValueError("Input array must have exactly 7 elements.")
        self.q_values = q_array
        self.update()

    def get_q_values(self):
        return np.array(self.q_values)

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

    def jacobiano(self, params_dh):
        T_total = sp.eye(4)
        p = [sp.Matrix([0, 0, 0])]
        z = [sp.Matrix([0, 0, 1])]
        for param in params_dh:
            theta, d, a, alpha = param
            T_total = T_total * self.dh_matrix(theta, d, a, alpha)
            p.append(T_total[:3, 3])
            z.append(T_total[:3, 2])
        p_n = p[-1]
        J_linear = [z[i].cross(p_n - p[i]) for i in range(len(params_dh))]
        J_angular = z[:len(params_dh)]
        return sp.Matrix.vstack(sp.Matrix.hstack(*J_linear), sp.Matrix.hstack(*J_angular))



def calculate_omega(phi_r, phi_p, phi_dot_r, phi_dot_p, phi_dot_y):
    transformation_matrix = np.array([
        [0, -np.sin(phi_r), np.cos(phi_r) * np.cos(phi_p)],
        [0, np.cos(phi_r), np.sin(phi_r) * np.cos(phi_p)],
        [1, 0, -np.sin(phi_p)]
    ])
    phi_dot_vector = np.array([phi_dot_r, phi_dot_p, phi_dot_y])
    return np.dot(transformation_matrix, phi_dot_vector)


def limitar_angulo(q):
    return (q + np.pi) % (2 * np.pi) - np.pi  # Keep q between [-pi, pi]


def regularized_pseudoinverse(J, k):
    return J.T @ np.linalg.inv(J @ J.T + (k ** 2) * np.eye(J.shape[0]))


class RobotController:
    def __init__(self):
        rospy.init_node('angle_publisher', anonymous=True)
        self.robot = Robot()
        self.x_des = None
        self.pub = rospy.Publisher('angle_topic', Float64MultiArray, queue_size=10)
        rospy.Subscriber('desired_position', Float64MultiArray, self.desired_position_callback)
        self.rate = rospy.Rate(1000)

    def desired_position_callback(self, msg):
        self.x_des = np.array(msg.data)

    def control_loop(self):
        max_error = 0.5
        while not rospy.is_shutdown():
            if self.x_des is None:
                rospy.loginfo("Waiting for desired position...")
                self.rate.sleep()
                continue
            
            x_actual = self.robot.xWrist_actual
            e = x_actual - self.x_des
            e = np.clip(e, -max_error, max_error) 
            e[3:] = calculate_omega(x_actual[3], x_actual[4], *e[3:])

            e1, e2 = e.copy(), e.copy()
            e2[:3], e1[3:] = 0, 0

            if np.linalg.norm(e[:3]) > 1e-1:
                
                J = self.robot.jWrist
                J1, J2 = J.copy(), J.copy()
                J2[0:3], J1[3:] = 0, 0

                Ji1 = regularized_pseudoinverse(J1, 1e-1)
                P1 = (np.eye(7) - Ji1 @ J1)

                dq = Ji1 @ (-e1.T) + regularized_pseudoinverse(J2 @ P1, 0.1) @ (-e2.T + J2 @ Ji1 @ e1.T)
                q_new = self.robot.get_q_values() + dq * 0.01
                self.robot.set_q_values(q_new)

                msg = Float64MultiArray()
                msg.data = q_new
                self.pub.publish(msg)
                self.rate.sleep()
            else:
                rospy.loginfo("Reached desired position")

if __name__ == '__main__':
    controller = RobotController()
    controller.control_loop()
