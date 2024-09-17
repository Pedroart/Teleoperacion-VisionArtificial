#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
import time
import numpy as np

import plot_pose_live
import fun_espaciales as fe
import concurrent.futures

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import configuracion_articular as ca


class imagen_postura:
    def __init__(self) -> None:
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose

        # For webcam input:
        self.pose =  self.mp_pose.Pose(
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        self.LANDMARK_GROUPS = [
            #[8, 6, 5, 4, 0, 1, 2, 3, 7],   # eyes
            #[10, 9],                       # mouth
            [11, 13, 15, 17, 19, 15, 21],  # left arm 0
            [11, 23, 25, 27, 29, 31, 27],  # left body side 1
            #[12, 14, 16, 18, 20, 16, 22],  # right arm
            #[12, 24, 26, 28, 30, 32, 28],  # right body side 
            [11, 12],                      # shoulder 2
            [23, 24],                      # waist 3
            [11],                       # lett shoulder 4
        ]
        
        self.rot_columan = np.identity(3)
        self.rot_carera = np.identity(3)


    def plot_world_landmarks(self ,ax, landmarks):
        landmark_groups=self.LANDMARK_GROUPS

        # skip when no landmarks are detected
        if landmarks is None:
            return

        ax.cla()

        # had to flip the z axis
        ax.set_xlim3d(-1, 1)
        ax.set_ylim3d(-1, 1)
        ax.set_zlim3d(1, -1)

        rotated_points = self.obtener_puntos_brazo_izquierdo(landmarks)

        # Obtener coordenadas rotadas
        rotated_plotX = rotated_points[:, 0]
        rotated_plotY = rotated_points[:, 1]
        rotated_plotZ = rotated_points[:, 2]

        # this can be changed according to your camera
        ax.plot(rotated_plotX, rotated_plotZ, rotated_plotY)

        plt.pause(.001)
        return

    def calculo_columna(self, landmarks):
        
        if landmarks is not None:

            hombros_coords = np.array([
                [landmarks.landmark[i].x, landmarks.landmark[i].y, landmarks.landmark[i].z] 
                for i in self.LANDMARK_GROUPS[2]
            ])
            hombros = np.mean(hombros_coords, axis=0)
            


            centro_cadera = np.array([0, 0, 0])  # Asumiendo que cadera es el origen según la documentación
            columna = fe.unitario(hombros, centro_cadera)
            rot_columan = fe.matriz_rotacion([columna], [[0, -1, 0]])
            rotated_points = np.dot(rot_columan.T, hombros)
            i = 11
            caderaizquierda = np.array(
                [landmarks.landmark[i].x, landmarks.landmark[i].y, landmarks.landmark[i].z] 
            )
            caderaizquierda = fe.unitario(caderaizquierda, hombros)
            caderaizquierda[1] = 0
            
            rot_cadera = fe.matriz_rotacion([caderaizquierda], [[1, 0, 0]])
            
            self.rot_columan =rot_columan
            self.rot_carera  =rot_cadera

        else:
            self.rot_columan = np.identity(3)
            self.rot_carera = np.identity(3)    
    
    def obtener_puntos_brazo_izquierdo(self,landmarks):
        indices_brazo_izquierdo = [11, 13, 15, 17, 19, 15, 21]
        puntos_brazo_izquierdo_np = np.array([
            np.dot(self.rot_carera.T, np.dot(self.rot_columan.T, 
                np.array([landmarks.landmark[i].x, landmarks.landmark[i].y, landmarks.landmark[i].z]).T
            ))
            for i in indices_brazo_izquierdo
        ])
        
        
        return puntos_brazo_izquierdo_np

def talker():
    #cap = cv2.VideoCapture('/home/art/dev_ws/src/Teleoperacion-VisionArtificial/src/obtener_postura/test.mp4') 
    cap = cv2.VideoCapture(2)
    img_postura = imagen_postura()
    
    fig = plt.figure()
    cx = fig.add_subplot(1,1,1, projection='3d')
    
    rospy.init_node('angle_publisher', anonymous=True)
    pub = rospy.Publisher('angle_topic', Float64MultiArray, queue_size=10)
    
    rate = rospy.Rate(20) # 20hz

    while not rospy.is_shutdown() and cap.isOpened():
        inicio = time.time()
        success, image = cap.read()
        if not success:
            rospy.logwarn("Ignoring empty camera frame.")
            break
        
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results_pose = img_postura.pose.process(image)

        # draw 3D pose landmarks live
        if results_pose.pose_world_landmarks is not None:
            img_postura.calculo_columna(results_pose.pose_world_landmarks)
            
            braso = img_postura.obtener_puntos_brazo_izquierdo(results_pose.pose_world_landmarks)
            angulos=ca.procesamiento(braso)
            plt.pause(.00001)
            plt.draw
            msg = Float64MultiArray()
            msg.data = angulos
            # Publica el mensaje
            pub.publish(msg)
        img_postura.plot_world_landmarks(cx, results_pose.pose_world_landmarks)
        fin = time.time()
        rospy.loginfo(fin-inicio)
        
        

        


        rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



