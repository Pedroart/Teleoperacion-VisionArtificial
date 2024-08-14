#!/usr/bin/env python3
import cv2
import mediapipe as mp
import concurrent.futures
import rospy
import plot_pose_live
import matplotlib.pyplot as plt

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
    # setup plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    cap = cv2.VideoCapture('test.mp4') 
    img_postura = imagen_postura()
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            # If loading a video, use 'break' instead of 'continue'.
            break
        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Enviar las tareas al ejecutor
            futuro_algoritmo1 = executor.submit(img_postura.pose.process, image)
            futuro_algoritmo2 = executor.submit(img_postura.hands.process, image)

            # Esperar a que se completen las tareas y obtener los resultados
            results_pose = futuro_algoritmo1.result()
            results_hands = futuro_algoritmo2.result()
            #concurrent.futures.wait([futuro_algoritmo1, futuro_algoritmo2])
        
        img_postura.mp_drawing.draw_landmarks(
            image,
            results_pose.pose_landmarks,
            img_postura.mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=img_postura.mp_drawing_styles.get_default_pose_landmarks_style())
        
        plot_pose_live.plot_world_landmarks(ax, results_pose.pose_world_landmarks)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if results_hands.multi_hand_landmarks:
            for hand_landmarks in results_hands.multi_hand_landmarks:
                img_postura.mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    img_postura.mp_hands.HAND_CONNECTIONS,
                    img_postura.mp_drawing_styles.get_default_hand_landmarks_style(),
                    img_postura.mp_drawing_styles.get_default_hand_connections_style())

        # Flip the image horizontally for a selfie-view display.
        
        image = cv2.resize(image, (640, 480))
        cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
        if cv2.waitKey(1) & 0xFF == 27:
            break
    cap.release()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



