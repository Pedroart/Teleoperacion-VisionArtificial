import rospy
from sensor_msgs.msg import JointState
import sympy as sp
import numpy as np

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

    def get_efector_pose(self):
        return self.tWrist

def joint_state_callback(msg, robot):
    # Obtener valores de las articulaciones desde el mensaje
    joint_positions = msg.position[:6]  # Asegúrate de que el UR5 tiene 6 articulaciones
    print(joint_positions)
    robot.set_q_values(np.array(joint_positions))

    # Calcular y mostrar la cinemática directa
    efector_pose = robot.get_efector_pose()
    print(f"Pose del efector final:\n{efector_pose}")

if __name__ == "__main__":
    rospy.init_node("ur5_kinematics_tester")
    
    # Crear instancia del robot
    ur5_robot = Robot()

    # Suscribirse al tópico /joint_states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback, ur5_robot)

    print("Esperando datos de /joint_states...")
    rospy.spin()
