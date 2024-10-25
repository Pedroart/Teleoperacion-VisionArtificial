import sympy as sp
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


# Decorador para medir el tiempo de ejecución
def timeit(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = (end_time - start_time) * 1000  # Tiempo en milisegundos
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
            [-self.q[3]+sp.pi,   0.1685, 0, -sp.pi/2],
            [self.q[4],         0.4, 0, -sp.pi/2],
            [self.q[5]+sp.pi,   0.1363, 0, -sp.pi/2],
            [self.q[6]-sp.pi/2, 0.11, 8.08E-07, 0]
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

# Inicialización del robot
SawyerRobot = Robot()

rospy.init_node('angle_publisher', anonymous=True)

pub = rospy.Publisher('angle_topic', Float64MultiArray, queue_size=10)

rate = rospy.Rate(1000) # 20hz

while pub.get_num_connections() == 0:
    rospy.loginfo("Esperando conexiones al topic 'angle_topic'...")
    rospy.sleep(1)


'''
q_test_values = [0, 0, 0, , 0, 0, 0]

# Asignar los valores de q al robot
SawyerRobot.set_q_values(q_test_values)

# Calcular la pose actual del efector final (end-effector) a partir de la cinemática directa
xWrist_actual = SawyerRobot.xWrist_actual

msg = Float64MultiArray()
msg.data = q_test_values
pub.publish(msg)

rate.sleep()

# Mostrar la pose del efector final (posición y orientación en Euler xyz)
print("Valores de las articulaciones (q):", q_test_values)
print("Pose actual del efector final (end-effector):")
print("Posición [x, y, z]:", xWrist_actual[:3])
print("Orientación [r, p, y] (Euler angles):", xWrist_actual[3:])



'''

def calculate_omega(phi_r, phi_p, phi_dot_r, phi_dot_p, phi_dot_y):
    """
    Calcula el vector de velocidades angulares (omega) dado los ángulos y sus derivadas.

    Parámetros:
    phi_r: Ángulo de rotación roll (en radianes).
    phi_p: Ángulo de rotación pitch (en radianes).
    phi_dot_r: Derivada del ángulo roll (velocidad angular en roll).
    phi_dot_p: Derivada del ángulo pitch (velocidad angular en pitch).
    phi_dot_y: Derivada del ángulo yaw (velocidad angular en yaw).

    Retorna:
    omega: Vector de velocidades angulares.
    """
    # Matriz de transformación
    transformation_matrix = np.array([
        [0, -np.sin(phi_r), np.cos(phi_r) * np.cos(phi_p)],
        [0, np.cos(phi_r), np.sin(phi_r) * np.cos(phi_p)],
        [1, 0, -np.sin(phi_p)]
    ])

    # Vector de velocidades angulares (derivadas de los ángulos)
    phi_dot_vector = np.array([phi_dot_r, phi_dot_p, phi_dot_y])

    # Producto matriz-vector para obtener las velocidades angulares
    omega = np.dot(transformation_matrix, phi_dot_vector)

    return omega


def limitar_angulo(q):
    return (q + np.pi) % (2 * np.pi) - np.pi  # Modulo para mantener q entre [-pi, pi]



while not rospy.is_shutdown():
    init_time = time.time()  # Medir tiempo después de la inicialización

    # Objetivo deseado en el espacio operacional (traslación y rotación)
    x_des = np.array([0.8,0.4,0.4 
                      ,1.57079633,0,1.57079633])
    #x_des = np.array([0.4 ,0.4 ,0.4 ,1.57079633,0.0,1.57079633])
    # Obtener la pose actual del end-effector (se actualiza en cada llamada a update())
    x_actual = SawyerRobot.xWrist_actual

    #roll = r_actual[0]  # Ángulo de roll (en radianes)
    #pitch = r_actual[1]  # Ángulo de pitch (en radianes)
    #yaw = r_actual[2]    # Ángulo de yaw (en radianes)

    # Calcular el error
    e =    x_actual - x_des
    e[3:] =calculate_omega(x_actual[3],x_actual[4],*e[3:])

    if np.linalg.norm(e[0:3]) < 1e-2 and np.linalg.norm(e[3:]) < 1:
        break

    # Obtener el Jacobiano del robot
    J = SawyerRobot.jWrist
    



    # Calcular la pseudoinversa del Jacobiano
    #Ji = np.linalg.pinv(J)
    k = 0.1
    Ji = J.T @ np.linalg.inv(( J @ J.T + (k**2) * np.eye(6) ) )
   
    # Calcular el cambio en las velocidades articulares
    #dq = (Ji @ e) * (-0.2)
    dq = Ji @ (-e)

    print(dq)
    # Actualizar los valores de q
    print(SawyerRobot.get_q_values())
 
    q_new = SawyerRobot.get_q_values() + dq * 0.001
    #q_new = np.array([limitar_angulo(qi) for qi in q_new])
    #print(e,x_actual,q_new)
    
    print(init_time - time.time())

    SawyerRobot.set_q_values(q_new)

    # Publicar los nuevos valores articulares
    msg = Float64MultiArray()
    msg.data = q_new
    pub.publish(msg)

    rate.sleep()

