#!/usr/bin/env python3
import cv2
import mediapipe as mp
import concurrent.futures
import rospy
import plot_pose_live
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
from std_msgs.msg import String

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
            [8, 6, 5, 4, 0, 1, 2, 3, 7],   # eyes
            [10, 9],                       # mouth
            [11, 13, 15, 17, 19, 15, 21],  # left arm
            [11, 23, 25, 27, 29, 31, 27],  # left body side
            [12, 14, 16, 18, 20, 16, 22],  # right arm
            [12, 24, 26, 28, 30, 32, 28],  # right body side
            [11, 12],                      # shoulder
            [23, 24],                      # waist
        ]

def talker():
    cap = cv2.VideoCapture('/home/art/dev_ws/src/Teleoperacion-VisionArtificial/src/obtener_postura/test.mp4') 
    img_postura = imagen_postura()
    
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
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

        fin = time.time()
        #rospy.loginfo(fin-inicio)
        pub.publish(str(fin-inicio))
        rate.sleep()
    cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



