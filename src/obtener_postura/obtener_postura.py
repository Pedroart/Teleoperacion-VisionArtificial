#!/usr/bin/env python3
import cv2
import mediapipe as mp
import concurrent.futures
import rospy
import plot_pose_live
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time

class imagen_postura:
    def __init__(self) -> None:
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_pose = mp.solutions.pose
        self.mp_hands = mp.solutions.hands

        # For webcam input:
        self.pose =  self.mp_pose.Pose(
            model_complexity=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)

        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)


def talker():
    #setup plot
    #fig = plt.figure()
    #ax = fig.add_subplot(111, projection="3d")

    cap = cv2.VideoCapture('/home/art/dev_ws/src/Teleoperacion-VisionArtificial/src/obtener_postura/test.mp4') 
    img_postura = imagen_postura()
    while cap.isOpened():
        inicio = time.time()
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            break
        
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results_pose = img_postura.pose.process(image)

        #plot_pose_live.plot_world_landmarks(ax, results_pose.pose_world_landmarks)
        
        fin = time.time()
        print(fin-inicio)
    cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



