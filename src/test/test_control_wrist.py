import rospy
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import sympy as sp
import numpy as np
from std_msgs.msg import Header

class Robot:
    def __init__(self):
        self.q = sp.symbols('q1 q2 q3 q4 q5 q6')
        self.params_dh = [
            [self.q[0] + sp.pi, 0.089159, 0, sp.pi / 2],  # Joint 1
            [self.q[1], 0, -0.42500, 0],                 # Joint 2
            [self.q[2], 0, -0.39225, 0],                 # Joint 3
            [self.q[3], 0.10915, 0, sp.pi / 2],          # Joint 4
            [self.q[4], 0.09465, 0, -sp.pi / 2],         # Joint 5
            [self.q[5], 0.0823, 0, 0]                    # Joint 6
        ]

        self._tWrist_symbolic = self.cinematica_directa(self.params_dh)
        self._jWrist_symbolic = self.jacobiano(self.params_dh)

        self.tWrist_func = sp.lambdify(self.q, self._tWrist_symbolic, "numpy")
        self.jWrist_func = sp.lambdify(self.q, self._jWrist_symbolic, "numpy")

        self.q_values = np.array([0, 0, 0, 0, 0, 0])
        self.update()

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

    def get_efector_pose(self):
        return self.tWrist

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


# Callback para recibir y actualizar el estado de las articulaciones
def joint_state_callback(msg, robot):
    joint_positions = msg.position[:6]
    robot.set_q_values(np.array(joint_positions))

# Publicar nuevas posiciones articulares para simular el control en RViz
def publish_joint_states(pub, robot):
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        q_current = robot.q_values

        # Crear mensaje JointState
        joint_msg = JointState()
        joint_msg.header = Header()
        joint_msg.header.stamp = rospy.Time.now()
        joint_msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                          'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        joint_msg.position = q_current

        pub.publish(joint_msg)
        

def calculate_pose_error(T_actual, T_desired):
    position_actual = T_actual[:3, 3]
    position_desired = T_desired[:3, 3]
    position_error = position_desired - position_actual
    R_actual = T_actual[:3, :3]
    R_desired = T_desired[:3, :3]
    R_error = R_desired @ R_actual.T  
    rotation_vector = R.from_matrix(R_error).as_rotvec()
    return np.concatenate((position_error, rotation_vector))

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

# Control de pose hacia un objetivo
def control_pose(robot, target_pose, pub):
    max_iter = 2100
    epsilon = 1e-6

    for i in range(max_iter):
        # Pose actual
        current_pose = robot.get_efector_pose()
        error = calculate_pose_error(current_pose, target_pose)
        print(error)

        print(f"Iteración {i}: Error = {np.linalg.norm(error)}")
        if np.linalg.norm(error) < epsilon:
            print("Pose alcanzada.")
            break

        # Calcular Jacobiano y actualizar q
        J = robot.jWrist
        q_delta = np.linalg.pinv(J) @ error  # Sin orientación en este caso
        robot.set_q_values(robot.q_values + q_delta)

    # Publicar nueva configuración
    print(robot.tWrist)
    publish_joint_states(pub, robot)

if __name__ == "__main__":
    rospy.init_node("pose_control_rviz")

    # Crear robot
    ur5_robot = Robot()

    # Suscribirse al estado actual de las articulaciones
    rospy.Subscriber("/joint_states", JointState, joint_state_callback, ur5_robot)

    # Publicador de estado articular
    pub = rospy.Publisher("/joint_states", JointState, queue_size=60)

    # Objetivo deseado (pose)
    target_pose = np.array([
        [0.0, -1.0, 0.0, 0],
        [-1.0, 0.0, 0.0, 0],
        [0.0, 0.0, -1.0, 0],
        [0.0, 0.0, 0.0, 1.0]
    ])


    target_pose[:3, 3] = [0.25, 0.0, 0.3]  # Cambiar la posición deseada

    control_pose(ur5_robot, target_pose, pub)
