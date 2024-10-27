#!/usr/bin/env python3

import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

'''
# Decorator to measure execution time
def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Execution time in milliseconds
        return result
    return wrapper'''


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

    #@timeit
    def update(self):
        # Evaluate matrices using lambdify functions
        self.tWrist = self.tWrist_func(*self.q_values)
        self.jWrist = self.jWrist_func(*self.q_values)

        self.tElbow = self.tElbow_func(*self.q_values)
        self.jElbow = self.jElbow_func(*self.q_values)

        self.xWrist_actual = self.get_pose(self.tWrist)
        self.xElbow_actual = self.get_pose(self.tElbow)

    #@timeit
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


# Inicializar el nodo de ROS y configurar la clase de robot
rospy.init_node("kinematics_visualizer")
robot = Robot()

# Publicador para el marcador en RViz
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
rate = rospy.Rate(10)  # Frecuencia de publicación en Hz

joint_names = ["head_pan", "right_j0", "right_j1", "right_j2", "right_j3",
               "right_j4", "right_j5", "right_j6"]

def create_marker(position, marker_id=0):
    """
    Crea un marcador de RViz para visualizar el efector final.

    Parámetros:
    - position: Posición [x, y, z] del marcador.
    - marker_id: ID único para el marcador.

    Retorna:
    - Marker: Mensaje de marcador configurado.
    """
    marker = Marker()
    marker.header.frame_id = "base"  # Cambia esto al marco de referencia base de tu robot
    marker.header.stamp = rospy.Time.now()
    marker.ns = "end_effector"
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    # Configurar la posición del marcador
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]

    # Configuración del marcador: escala y color
    marker.scale.x = 0.05  # Tamaño del marcador
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Verde con opacidad completa

    return marker

while not rospy.is_shutdown():
    random_q_values = np.random.uniform(np.pi, -np.pi, 7)
    # Ejecutar set_q_values con los valores generados
    robot.set_q_values(random_q_values)
    # Obtener la posición del efector final a partir de la cinemática directa
    T_end_effector = robot.tWrist  # Usamos la pose calculada en `update`
    end_effector_position = T_end_effector[:3, 3]

    # Crear y publicar el marcador de posición
    marker = create_marker(end_effector_position)
    marker_pub.publish(marker)

    # Crear el mensaje JointState
    joint_state_msg = JointState()
    joint_state_msg.name = joint_names
    joint_state_msg.position = robot.get_publish_q_values()#.tolist()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_pub.publish(joint_state_msg)

    print(joint_state_msg)
    rospy.sleep(5)