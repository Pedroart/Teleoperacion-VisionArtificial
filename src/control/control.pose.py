#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import sympy as sp
from PyRobotFrames import *
import threading

class XYZToJointPositions:
    def __init__(self):
        rospy.init_node("xyz_to_joint_positions", anonymous=True)

        # Crear un publicador para el tópico /joint_positions
        self.joint_pub = rospy.Publisher("/joint_positions", Float64MultiArray, queue_size=10)
        
        # Suscriptores
        rospy.Subscriber("/xyz_target", Point, self.xyz_callback)
        rospy.Subscriber('/ur5_robot/joint_states', JointState, self.joint_states_callback)

        # Inicializar variables
        self.joint_positions_actual = np.zeros(6)  # Ejemplo con 6 DOF
        self.xyz_queue = []  # Cola para almacenar coordenadas XYZ

        # Parámetros del robot
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

        # Crear el robot usando los parámetros DH
        dh_params = dhParameters(theta, d, a, alpha, kind)
        self.robot = robot(dh_params, q_lim)

        # Iniciar el hilo para procesar la cola de coordenadas XYZ
        self.processing_thread = threading.Thread(target=self.process_queue)
        self.processing_thread.daemon = True  # Hilo como daemon para que se detenga al cerrar el programa
        self.processing_thread.start()

        rospy.loginfo("Nodo activo, esperando coordenadas XYZ en /xyz_target")

    def joint_states_callback(self, msg):
        """Callback para recibir los estados de las articulaciones."""
        self.joint_positions_actual = np.array(msg.position)

    def xyz_callback(self, msg):
        """Callback para recibir coordenadas XYZ."""
        xyz = np.array([msg.x, msg.y, msg.z])
        rospy.loginfo(f"Coordenadas XYZ recibidas: {xyz}")
        self.xyz_queue.append(xyz)

    def sendq(self, q):
        """Enviar posiciones articulares al tópico /joint_positions."""
        if q is None:
            rospy.logwarn("No se pudo resolver la cinemática inversa para XYZ")
            return

        joint_positions_msg = Float64MultiArray()
        joint_positions_msg.data = q
        self.joint_pub.publish(joint_positions_msg)

        rospy.loginfo(f"Posiciones articulares enviadas: {q}")

    def process_queue(self):
        """Procesar coordenadas XYZ de la cola y enviar posiciones articulares."""
        while not rospy.is_shutdown():
            if self.xyz_queue:
                xyz = self.xyz_queue.pop(0)  # Obtener el siguiente XYZ
                rospy.loginfo(f"Procesando XYZ: {xyz}")
                
                # Resolver la cinemática inversa para obtener las posiciones articulares
                self.robot._q = 
                q = self.robot.ikine_task(xyz)
                self.sendq(q)

if __name__ == "__main__":
    try:
        node = XYZToJointPositions()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido.")
