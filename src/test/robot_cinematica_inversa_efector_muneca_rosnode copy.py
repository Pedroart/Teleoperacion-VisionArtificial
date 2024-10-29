#!/usr/bin/env python3

import rospy
import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Float64MultiArray
from sensor_msgs.msg import JointState

import time

# Decorator to measure execution time
def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Execution time in milliseconds
        return result
    return wrapper

class Robot:
    def __init__(self):
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
        self._tWrist_symbolic = self.cinematica_directa(self.params_dh[:5])
        self._jWrist_symbolic = self.jacobiano(self.params_dh[:5])

        self._tScalpel_symbolic = self.cinematica_directa(self.params_dh)
        self._jScalpel_symbolic = self.jacobiano(self.params_dh)
        
        self._tElbow_symbolic = self.cinematica_directa(self.params_dh[:4])
        self._jElbow_symbolic = self.jacobiano(self.params_dh[:4])

        self.tWrist_func = sp.lambdify(self.q, self._tWrist_symbolic, "numpy")
        self.jWrist_func = sp.lambdify(self.q, self._jWrist_symbolic, "numpy")

        self.tScalpel_func = sp.lambdify(self.q, self._tScalpel_symbolic, "numpy")
        self.jScalpel_func = sp.lambdify(self.q, self._jScalpel_symbolic, "numpy")
        
        self.tElbow_func = sp.lambdify(self.q, self._tElbow_symbolic, "numpy")
        self.jElbow_func = sp.lambdify(self.q, self._jElbow_symbolic, "numpy")

        self.q_values = [0] * 7
        self.update()

    @timeit
    def update(self):
        self.tWrist = self.tWrist_func(*self.q_values)
        zeros = np.zeros((6, 2))
        self.jWrist = np.hstack((self.jWrist_func(*self.q_values), zeros))

        self.tScalpel = self.tScalpel_func(*self.q_values)
        self.jScalpel = self.jScalpel_func(*self.q_values)

        self.tElbow = self.tElbow_func(*self.q_values)
        zeros = np.zeros((6, 3))
        self.jElbow = np.hstack((self.jElbow_func(*self.q_values), zeros))

    def set_q_values(self, q_array):
        if len(q_array) != 7:
            raise ValueError("Input array must have exactly 7 elements.")
        self.q_values = q_array
        self.update()

    def get_q_values(self):
        return np.array(self.q_values)
    
    def get_publish_q_values(self):
        return np.concatenate(([0], self.q_values))

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

def create_marker(position, marker_id=0):
    marker = Marker()
    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "end_effector"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    marker.scale.x = marker.scale.y = marker.scale.z = 0.05
    marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)

    return marker

def regularized_pseudoinverse(J, u=np.sqrt(0.001)):
    return J.T @ np.linalg.inv(J @ J.T + u**2 * np.eye(J.shape[0]))

def generalized_task_augmentation(q, J_tasks, errors, deltaT=1, u=np.sqrt(0.001)):
    P = np.eye(len(q))
    q_dot = np.zeros(len(q))
    
    for J, r_dot in zip(J_tasks, errors):
        Ji_hash = regularized_pseudoinverse(J, u)
        q_dot += P @ Ji_hash @ r_dot
        P = P @ (np.eye(len(q)) - Ji_hash @ J)

    return q + q_dot * deltaT, q_dot

point_Wrist = np.array([0.8, 0.3, 0.1])
point_elbow = np.array([0.3, 0.3, 0.5])

def callback_wrist(msg):
    global point_Wrist
    escala = 0.5
    a = np.array([msg.data[2]*escala+0.3,-msg.data[1]*escala,msg.data[0]*0+0.2])
    point_Wrist = np.array(a)

def callback_elbow(msg):
    global point_elbow
    point_elbow = np.array(msg.data)

# Initialize ROS node
rospy.init_node("kinematics_visualizer")

# Initialize Robot object
robot = Robot()

# Publishers
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)

# Subscribers
rospy.Subscriber("wrist_point", Float64MultiArray, callback_wrist)
rospy.Subscriber("elbow_point", Float64MultiArray, callback_elbow)

# Main loop
rate = rospy.Rate(100)
joint_names = ["head_pan", "right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
desired_direction = np.array([0, 0, 3])
kpp = 10
kpr = 0.1
kpe = 1

def create_transformation_matrix(x, y, z):
    T = np.array([
        [0.0, 0.0, 1.0, x],
        [1.0, -0.0, 0.0, y],
        [0.0, 1.0, -0.0, z],
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

def calculate_orientation_error(T_link, desired_direction, link_axis=np.array([0, 1, 0])):
    """
    Calcula el error de orientación entre el eje deseado de un eslabón y una dirección en el espacio.
    
    Parámetros:
    - T_link (np.ndarray): Matriz de transformación 4x4 del eslabón.
    - desired_direction (np.ndarray): Vector de dirección deseada en el espacio (por ejemplo, [0, 1, 0]).
    - link_axis (np.ndarray): Eje del eslabón que debería apuntar hacia la dirección deseada (por defecto, el eje Z).
    
    Retorna:
    - orientation_error (np.ndarray): Vector de error de orientación como un vector rotacional.
    """
    # Extraer la rotación del eslabón en el espacio
    R_link = T_link[:3, :3]
    # Calcular el eje actual del eslabón proyectado en el espacio global
    current_direction = R_link @ link_axis
    # Error de orientación como el producto vectorial entre la dirección deseada y la actual
    orientation_error = np.cross(current_direction, desired_direction)
    
    return orientation_error

while not rospy.is_shutdown():
    T_desired = create_transformation_matrix(*point_Wrist)
    T_Elbow = create_transformation_matrix(*point_elbow)

    e = calculate_pose_error(robot.tWrist , T_desired)
    e1 = e/kpp

    e3 = calculate_pose_error(robot.tElbow, T_Elbow) / kpe
    e3[3:] = e3[3:]*0  

    J_tasks = [robot.jWrist, robot.jElbow]
    errors = [e1,e3]

    #print(J2)
    
    q, _ = generalized_task_augmentation(robot.get_q_values(), J_tasks, errors, deltaT=0.01)
    robot.set_q_values(q)

    end_effector_position = T_desired[:3, 3]
    marker_pub.publish(create_marker(end_effector_position))

    joint_state_msg = JointState()
    joint_state_msg.name = joint_names
    joint_state_msg.position = robot.get_publish_q_values().tolist()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_pub.publish(joint_state_msg)

    rate.sleep()
