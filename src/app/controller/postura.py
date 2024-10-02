#!/usr/bin/env python3
import cv2
import mediapipe as mp
import numpy as np

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



class postura:
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


        
    def set_postura(self,imagen):
        results_pose = self.pose.process(imagen)
        # draw 3D pose landmarks live
        if results_pose.pose_landmarks is not None:
            self.mp_drawing.draw_landmarks(
                imagen,
                results_pose.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                )
            
            return  imagen
        
        return imagen

