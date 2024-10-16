#!/usr/bin/env python3
import cv2
import mediapipe as mp
import numpy as np
import controller.fun_espaciales as fe

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class pose:
    '''
    (1,0.5,0.5) 56ms
    (1,0.8,0.5) 59ms
    (1,0.8,0.8) 63ms
    
    (2,0.5,0.5) medio 147 - 209ms
    (2,0.5,0.5) medio 147 - 209ms
    
    '''

    estado = False

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
            [11, 23],  # left body side 1
            #[12, 14, 16, 18, 20, 16, 22],  # right arm
            #[12, 24, 26, 28, 30, 32, 28],  # right body side 
            [11, 12],                      # shoulder 2
            [23, 24],                      # waist 3
            #[11],                       # lett shoulder 4
        ]
        self.results_pose = None

    def set_pose(self,imagen):
        self.results_pose = self.pose.process(imagen)
        #print(self.results_pose)
        # draw 3D pose landmarks live
        if self.results_pose.pose_landmarks is not None:
            return True
        return False

    def son_puntos_visibles(self, umbral_visibilidad=0.5):
        # Indices de los puntos del brazo izquierdo
        puntos_brazo_izquierdo = [11, 13, 15, 17, 19, 21]

        # Verificar si pose_landmarks está disponible
        if self.results_pose is None or self.results_pose.pose_landmarks is None:
            self.estado = False
            return False

        # Verificar la visibilidad de los puntos
        for idx in puntos_brazo_izquierdo:
            if self.results_pose.pose_landmarks.landmark[idx].visibility < umbral_visibilidad:
                self.estado = False
                return False
            
        self.estado = True
        return True

    def get_draw_pose(self,imagen):
        if self.results_pose is not None and self.results_pose.pose_landmarks is not None:
            self.mp_drawing.draw_landmarks(
                imagen,
                self.results_pose.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                )
        '''
        else:
            print('landmark no se observan')
        '''
        return imagen
    
    def normalizacion(self):
        if self.results_pose.pose_world_landmarks is not None:

            self.hombros_coords = np.mean(np.array([
                [
                    self.results_pose.pose_world_landmarks.landmark[i].x,
                    self.results_pose.pose_world_landmarks.landmark[i].y,
                    self.results_pose.pose_world_landmarks.landmark[i].z
                ] 
                for i in self.LANDMARK_GROUPS[2]
            ]), axis=0)
            
            centro_hombros = fe.unitario(self.hombros_coords)
            self.rot_columan = fe.matriz_rotacion([centro_hombros], [[0, -1, 0]])

            i = 11
            hombro_izquierda_coords = np.array(
                [
                    self.results_pose.pose_world_landmarks.landmark[i].x,
                    self.results_pose.pose_world_landmarks.landmark[i].y,
                    self.results_pose.pose_world_landmarks.landmark[i].z
                ]
            )

            hombro_izquierdo = fe.unitario(hombro_izquierda_coords, self.hombros_coords)
            hombro_izquierdo[1] = 0
            self.rotar_hombro = fe.matriz_rotacion([hombro_izquierdo], [[1, 0, 0]])

    def get_angulos(self):
        
        coords_rotadas = self.aplicar_normalizacion_(self.LANDMARK_GROUPS[0])
        coords_rotadas -= coords_rotadas[:,0][:, np.newaxis]
        codo = fe.unitario(coords_rotadas[:,1])
        #print("code: ",coords_rotadas[:,1])
        q2 = np.arcsin(codo[2])

        # Ahora resolvemos q1
        q1 = -np.arctan2(codo[0], codo[1])
        
        
        # Calculo de q3
        AC = coords_rotadas[:,0] - coords_rotadas[:,1]
        BC = coords_rotadas[:,2] - coords_rotadas[:,1]
        #print(coords_rotadas[:,2])
        # Calculamos el ángulo entre los vectores AC y BC usando el producto punto
        cos_theta = np.dot(AC, BC) / (np.linalg.norm(AC) * np.linalg.norm(BC))
        q4 = np.arccos(cos_theta)-np.pi/2
        q3 = np.arccos(fe.unitario(np.cross(AC, BC))[2]/cos_q2) - np.pi/2
        
        

        '''muneca = fe.unitario(coords_rotadas[:,2],coords_rotadas[:,1]) + codo
        #print(muneca)
        cos_q3 = (np.sin(q2) * (np.sin(q4)-1) + muneca[1]) / (np.cos(q2) * np.cos(q4))
        cos_q3 = np.clip(cos_q3, -1, 1)
        q3 = np.arccos(cos_q3)
        '''
        return np.array([q1,q2,q3,q4,0,0,0])
        #print(q1*180/3.14,q2*180/3.14,q4*180/3.14,q3*180/3.14)
        
        

    def aplicar_normalizacion_(self,group):
        landmarks = self.results_pose.pose_world_landmarks
        # Crear matriz de coordenadas (3xN) donde N es el número de puntos en el grupo
        coords = np.array([
            [landmarks.landmark[i].x, landmarks.landmark[i].y, landmarks.landmark[i].z]
            for i in group
        ]).T  # Transponer para obtener una matriz 3xN (X, Y, Z en columnas)

        # Restar las coordenadas de los hombros (traslación)
        coords -= self.hombros_coords[:, np.newaxis]

        # Aplicar las dos matrices de rotación a todo el conjunto de coordenadas
        return np.dot(self.rotar_hombro.T, np.dot(self.rot_columan.T, coords))
    
    def plot_world_landmarks(self, ax):
    # Verificar si no hay landmarks detectados
        if self.results_pose.pose_world_landmarks is None:
            return

        #ax.cla()
        ax.clear()
        # Configurar los límites de los ejes 3D
        ax.set_xlim3d(-0.5, 0.5)
        ax.set_ylim3d(-0.5, 0.5)
        ax.set_zlim3d(0.5, -0.5)

        # Obtener los landmarks
        landmarks = self.results_pose.pose_world_landmarks

        for group in self.LANDMARK_GROUPS:
            coords_rotadas = self.aplicar_normalizacion_(group)

            # Extraer las coordenadas rotadas
            plotX_rot = coords_rotadas[0, :]
            plotY_rot = coords_rotadas[1, :]
            plotZ_rot = coords_rotadas[2, :]

            # Graficar los puntos rotados (cambia Z e Y para ajustarse a la cámara)
            ax.plot(plotX_rot, plotZ_rot, plotY_rot, color='gray')

        # Pausar para actualizar la gráfica
        
        return




    

        
            

