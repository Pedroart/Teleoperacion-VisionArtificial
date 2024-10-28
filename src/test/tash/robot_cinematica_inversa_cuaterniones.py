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
        zeros = np.zeros((6, 3))
        self.jElbow = np.hstack((self.jElbow_func(*self.q_values) , zeros))

        #self.xWrist_actual = self.get_pose(self.tWrist)
        #self.xElbow_actual = self.get_pose(self.tElbow)

    '''    
    #@timeit
    def get_pose(self, T):
        t_actual = T[:3, 3]
        R_actual = T[:3, :3]
        r_actual = R.from_matrix(R_actual).as_euler('xyz')
        return np.concatenate((t_actual, r_actual))
    '''

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

    '''def jacobiano(self, params_dh):
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
        '''
    def jacobiano(self, params_dh):
        # Configuración inicial
        T_total = sp.eye(4)  # Matriz identidad para la transformación total
        p_n = []  # Lista para almacenar la posición del efector final
        T_list = []  # Lista para almacenar la matriz de transformación acumulada para cada articulación

        # Calcular las matrices de transformación individuales y la posición del efector
        for param in params_dh:
            theta, d, a, alpha = param
            T_i = self.dh_matrix(theta, d, a, alpha)
            T_total = T_total * T_i
            T_list.append(T_total)
        
        p_n = T_total[:3, 3]  # Posición del efector final en el espacio

        # Convertir la orientación del efector en cuaterniones
        R_efector = T_total[:3, :3]  # Matriz de rotación final del efector
        q_efector = self.rotation_to_quaternion(R_efector)  # Cuaternión de la orientación del efector final

        # Inicializar listas para Jacobiano lineal y Jacobiano angular
        J_linear = []
        J_angular = []

        # Cálculo del Jacobiano utilizando derivadas parciales
        for i in range(len(params_dh)):
            # Derivada parcial de la posición del efector con respecto a q_i (Jacobiano lineal)
            Jv_i = p_n.diff(self.q[i])  # Derivada parcial de la posición con respecto a q_i
            J_linear.append(Jv_i)
            
            # Derivada parcial del cuaternión con respecto a q_i (Jacobiano angular)
            R_i = T_list[i][:3, :3]  # Extrae la matriz de rotación hasta la articulación i
            q_i = self.rotation_to_quaternion(R_i)  # Cuaternión de la articulación i
            dq_i = [q_efector[j].diff(self.q[i]) for j in range(4)]  # Derivadas parciales de cada componente del cuaternión
            J_angular.append(sp.Matrix(dq_i))  # Añadir las derivadas parciales al Jacobiano angular

        # Concatenar el Jacobiano lineal y angular
        return sp.Matrix.vstack(sp.Matrix.hstack(*J_linear), sp.Matrix.hstack(*J_angular))

    # Función auxiliar para convertir matriz de rotación a cuaternión
    def rotation_to_quaternion(self, R):
        """
        Convierte una matriz de rotación 3x3 en un cuaternión.
        
        Parámetros:
        R: Matriz de rotación 3x3
        
        Retorna:
        sp.Matrix: Cuaternión [q0, q1, q2, q3] (q0 es el término escalar)
        """
        # Elementos de la matriz de rotación
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = sp.sqrt(trace + 1.0) * 2  # Escala para el cuaternión
            qw = 0.25 * s
            qx = (R[2, 1] - R[1, 2]) / s
            qy = (R[0, 2] - R[2, 0]) / s
            qz = (R[1, 0] - R[0, 1]) / s
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            s = sp.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = sp.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = sp.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s

        return sp.Matrix([qw, qx, qy, qz])


def transformation_to_pose(T):
    """
    Convierte una matriz de transformación homogénea (4x4) en una pose
    con posición y orientación en cuaterniones.

    Parámetros:
    T (np.ndarray): Matriz de transformación 4x4.

    Retorna:
    tuple: (x, y, z, qw, qx, qy, qz), donde (x, y, z) es la posición y 
           (qw, qx, qy, qz) representa la orientación en cuaterniones.
    """
    # Extraer la posición (x, y, z) desde la matriz de transformación
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    
    # Extraer la matriz de rotación 3x3
    R_mat = T[:3, :3]
    
    # Convertir la matriz de rotación a cuaterniones usando scipy
    rotation = R.from_matrix(R_mat)
    qw, qx, qy, qz = rotation.as_quat()  # Nota: Devuelve en formato [qx, qy, qz, qw]
    
    return (x, y, z, qw, qx, qy, qz)

# Inicializar el nodo de ROS y configurar la clase de robot
rospy.init_node("kinematics_visualizer")
robot = Robot()

# Publicador para el marcador en RViz
marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
rate = rospy.Rate(100)  # Frecuencia de publicación en Hz

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

def calculate_pose_error(T_actual, T_desired):
    """
    Calcula el error de pose entre la pose actual y la deseada.

    Parámetros:
    T_actual (np.ndarray): Matriz de transformación 4x4 actual del efector.
    T_desired (np.ndarray): Matriz de transformación 4x4 deseada del efector.

    Retorna:
    np.ndarray: Vector de error de posición y velocidad angular [dx, dy, dz, wx, wy, wz].
    """
    # Obtener la posición actual y deseada
    pose_actual = transformation_to_pose(T_actual)
    pose_desired = transformation_to_pose(T_desired)
    
    
    return pose_desired - pose_actual

def create_transformation_matrix(x, y, z):
    """
    Crea una matriz de transformación 4x4 con los valores de traslación especificados.

    Parámetros:
    x (float): Valor de traslación en el eje X.
    y (float): Valor de traslación en el eje Y.
    z (float): Valor de traslación en el eje Z.

    Retorna:
    np.ndarray: Matriz de transformación 4x4.
    """
    T = np.array([
        [0.0, 0.0, 1.0, x],
        [1.0, -0.0, 0.0, y],
        [0.0, 1.0, -0.0, z],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return T

def regularized_pseudoinverse(J, u=np.sqrt(0.001)):
    """
    Calcula la pseudo-inversa regularizada de una matriz Jacobiana J.
    
    Parámetros:
    J (np.ndarray): La matriz Jacobiana.
    u (float): Parámetro de regularización.
    
    Retorna:
    np.ndarray: La pseudo-inversa regularizada de J.
    """
    # Calcular la pseudo-inversa regularizada
    J_hash = J.T @ np.linalg.inv(J @ J.T + u**2 * np.eye(J.shape[0]))
    return J_hash



def generalized_task_augmentation(q, J_tasks, errors, deltaT=1, u=np.sqrt(0.001)):
    """
    Calcula la actualización de las articulaciones para múltiples tareas utilizando pseudo-inversas regularizadas.

    Parámetros:
    - q (np.ndarray): Valores actuales de las articulaciones.
    - J_tasks (list of np.ndarray): Lista de Jacobianos, uno para cada tarea.
    - errors (list of np.ndarray): Lista de vectores de error para cada tarea.
    - deltaT (float): Paso de tiempo.
    - u (float): Parámetro de regularización.

    Retorna:
    - q_new (np.ndarray): Nueva configuración de las articulaciones.
    - q_dot (np.ndarray): Velocidades de las articulaciones.
    """
    # Inicializar matrices de proyección y q_dot
    P = np.eye(len(q))  # Matriz de proyección inicial (identidad)
    q_dot = np.zeros(len(q))  # Inicializar q_dot a ceros
    
    # Iterar sobre cada tarea
    for J, r_dot in zip(J_tasks, errors):
        # Calcular la pseudo-inversa regularizada de J
        Ji_hash = J.T @ np.linalg.inv(J @ J.T + u**2 * np.eye(J.shape[0]))
        
        # Actualizar q_dot con la contribución de la tarea actual
        q_dot += P @ Ji_hash @ r_dot
        
        # Actualizar la matriz de proyección para la próxima tarea
        P = P @ (np.eye(len(q)) - Ji_hash @ J)

    # Calcular la nueva configuración de las articulaciones
    q_new = q + q_dot * deltaT
    return q_new, q_dot

T_desired = create_transformation_matrix(0.8, 0.3, 0.3)

T_Elbow = create_transformation_matrix(0.8, 0.3, 0.6)

kpp = 0.1;
kpr = 0.1;

while not rospy.is_shutdown():
    

    # Obtener la posición del efector final a partir de la cinemática directa
    T_end_effector = robot.tWrist  # Usamos la pose calculada en `update`
    
    e = calculate_pose_error(T_end_effector, T_desired)

    e1 = e[0:3]/kpp
    e2 = e[3:]/kpr

    J = robot.jWrist

    
    J1 = J[0:3]
    J2 = J[3:]

    J_tasks = [J1,J2]
    errors = [e1,e2]

    q = robot.get_q_values()
    
    q= generalized_task_augmentation(q, J_tasks, errors, deltaT=0.01, u=np.sqrt(0.001))

    robot.set_q_values(q[0])

    end_effector_position = T_desired[:3, 3]

    # Crear y publicar el marcador de posición
    marker = create_marker(end_effector_position)
    marker_pub.publish(marker)

    # Crear el mensaje JointState
    joint_state_msg = JointState()
    joint_state_msg.name = joint_names
    joint_state_msg.position = robot.get_publish_q_values().tolist()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_pub.publish(joint_state_msg)
    #rospy.sleep(1)
    rate.sleep()