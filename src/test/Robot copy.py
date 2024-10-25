import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray


# Decorador para medir el tiempo de ejecución
def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Tiempo en milisegundos
        #print(f"Función '{func.__name__}' ejecutada en: {execution_time:.2f} ms")
        return result
    return wrapper

class Robot:
    def __init__(self):
        # Definir variables simbólicas y parámetros DH
        self.q = sp.symbols('q1 q2 q3 q4 q5 q6 q7')
        self.params_dh = [
            [self.q[0],         0.317, 0.081, -sp.pi/2],
            [self.q[1]-sp.pi/2, 0.1925, 0, -sp.pi/2],
            [self.q[2],         0.4, 0, -sp.pi/2],
            [self.q[3]+sp.pi,   0.1685, 0, -sp.pi/2],
            [self.q[4],         0.4, 0, -sp.pi/2],
            [self.q[5]+sp.pi,   0.1363, 0, -sp.pi/2],
            [self.q[6]-sp.pi/2, 0.13375, 0, 0]
        ]
        # Precalcular cinemática directa simbólica y jacobiano simbólico
        self._tWrist_symbolic = self.cinematica_directa(self.params_dh)
        self._jWrist_symbolic = self.jacobiano(self.params_dh)

        self._tElbow_symbolic = self.cinematica_directa(self.params_dh[0:4])
        self._jElbow_symbolic = self.jacobiano(self.params_dh[0:4])

        # Convertir expresiones simbólicas a funciones rápidas de NumPy
        self.tWrist_func = sp.lambdify(self.q, self._tWrist_symbolic, "numpy")
        self.jWrist_func = sp.lambdify(self.q, self._jWrist_symbolic, "numpy")
        
        self.tElbow_func = sp.lambdify(self.q, self._tElbow_symbolic, "numpy")
        self.jElbow_func = sp.lambdify(self.q, self._jElbow_symbolic, "numpy")

        # Valores iniciales de q_values
        #self.q_values = [0.0, 4.71238898038469, 0.0, 3.141592653589793, 0.0, 3.141592653589793, 4.71238898038469]
        self.q_values = [0, 0, 0, 0, 0, 0, 0]

        # Actualizar las matrices numéricas
        self.update()

    @timeit  # Decorador para medir tiempo
    def update(self):
        # Usar funciones lambdify para evaluar matrices
        self.tWrist = self.tWrist_func(*self.q_values)
        self.jWrist = self.jWrist_func(*self.q_values)

        self.tElbow = self.tElbow_func(*self.q_values)
        self.jElbow = self.jElbow_func(*self.q_values)

        self.xWrist_actual = self.get_pose(self.tWrist)
        self.xElbow_actual = self.get_pose(self.tElbow)

    @timeit  # Decorador para medir tiempo
    def get_pose(self,T):
        t_actual = T[:3, 3]
        R_actual = T[:3, :3]
        r_actual = R.from_matrix(R_actual).as_euler('xyz')
        return np.concatenate((t_actual, r_actual))

    def set_q_values(self, q_array):
        if len(q_array) != 7:
            raise ValueError("El array de entrada debe tener exactamente 7 elementos.")
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

# Medir el tiempo desde la inicialización del robot
start_time = time.time()

# Inicialización del robot
SawyerRobot = Robot()

rospy.init_node('angle_publisher', anonymous=True)
pub = rospy.Publisher('angle_topic', Float64MultiArray, queue_size=10)

rate = rospy.Rate(1000) # 20hz

while pub.get_num_connections() == 0:
    rospy.loginfo("Esperando conexiones al topic 'angle_topic'...")
    rospy.sleep(1)

while not rospy.is_shutdown():
    init_time = time.time()  # Medir tiempo después de la inicialización

    # Objetivo deseado en el espacio operacional (traslación y rotación)
    x_des = np.array([1.01475,0.1603,0.317,1.57079633,0.0,1.57079633])

    # Obtener la pose actual del end-effector (se actualiza en cada llamada a update())
    x_actual = SawyerRobot.xWrist_actual
    print(x_actual)
    #print(SawyerRobot.tWrist)

    # Calcular el error
    e = x_des - x_actual
    print(e)
    if(np.linalg.norm(e) <1e-3):
        break
    # Obtener el Jacobiano del robot
    J = SawyerRobot.jWrist

    # Calcular la pseudoinversa del Jacobiano
    Ji = np.linalg.pinv(J)

    # Calcular el cambio en las velocidades articulares
    dq = Ji @ e
    
    # Actualizar los valores de q
    q_new = SawyerRobot.get_q_values() + dq * 0.01
    SawyerRobot.set_q_values(q_new)

    # Medir el tiempo hasta este punto
    end_time = time.time()

    msg = Float64MultiArray()

    msg.data = q_new

    # Publica el mensaje
    pub.publish(msg)

    # Calcular el tiempo total transcurrido
    execution_time = end_time - init_time

    # Mostrar el tiempo transcurrido en milisegundos y los nuevos valores articulares
    print(f"Nuevos valores de las articulaciones: {q_new}")
    #print(f"Tiempo de ejecución total: {execution_time * 1000} ms")
    rate.sleep()
