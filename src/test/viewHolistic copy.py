import cv2
import mediapipe as mp
import numpy as np
from copy import deepcopy
import argparse
import opencv_cam
import struct
import time

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_pose = mp.solutions.pose
mp_hand = mp.solutions.hands
mp_holistic = mp.solutions.holistic

def visibilityToColour(vis):
    if vis < 0.5:
        return (0, 0, 255)  # Red - low visibility
    elif vis < 0.75:
        return (0, 255, 255)  # Yellow - medium visibility
    else:
        return (0, 255, 0)  # Green - high visibility

def landmark_to_np(landmark):
    # Create a numpy array of the landmark positions
    return np.array([landmark.x, landmark.y, landmark.z])

def drawDebugViews(results, hand_points, hcp, hncp, hand_points_norm, pitchmode):
    # Create images for the 3 planar projection views
    window_size = 256
    xaxis = np.zeros((window_size, window_size, 3), np.uint8)
    xaxis[:] = (0, 0, 64)
    yaxis = np.zeros((window_size, window_size, 3), np.uint8)
    yaxis[:] = (0, 64, 0)
    zaxis = np.zeros((window_size, window_size, 3), np.uint8)
    zaxis[:] = (64, 0, 0)

    # Draw planar projection views for debugging
    if results.pose_world_landmarks is not None:
        last = None
        names = ['Wrist', 'Elbow', 'RSho', 'RHip', 'LHip', 'LSho']
        joints = [mp_pose.PoseLandmark.RIGHT_WRIST, mp_pose.PoseLandmark.RIGHT_ELBOW, mp_pose.PoseLandmark.RIGHT_SHOULDER,
                  mp_pose.PoseLandmark.RIGHT_HIP, mp_pose.PoseLandmark.LEFT_HIP, mp_pose.PoseLandmark.LEFT_SHOULDER]

        # Put all the world landmark positions for the joints into numpy array
        world_landmarks = np.array(
            [[results.pose_world_landmarks.landmark[i].x, results.pose_world_landmarks.landmark[i].y, results.pose_world_landmarks.landmark[i].z] for i in joints])

        # Center the landmarks in the window
        world_landmarks += 0.5

        # Scale the landmarks to fit in the window
        world_landmarks *= window_size

        # Estimate center of torso
        cp = (world_landmarks[2] + world_landmarks[4]) / 2.0

        # Compute the normal to the center of the torso
        normal = np.cross(world_landmarks[3] - world_landmarks[2], world_landmarks[4] - world_landmarks[2])
        normal /= np.linalg.norm(normal)
        ncp = cp + (normal * 20.0)

        # To bump the rendering down a bit to use the window better
        yoffset = int(window_size * .25)

        # To integers
        world_landmarks = world_landmarks.astype(int)
        cp = cp.astype(int)
        ncp = ncp.astype(int)

        for idx in range(len(world_landmarks)):
            landmark = world_landmarks[idx]
            cv2.circle(zaxis, (landmark[0], landmark[1] + yoffset), 2, (255, 255, 255), -1)
            cv2.circle(yaxis, (landmark[0], landmark[2] + yoffset), 2, (255, 255, 255), -1)
            cv2.circle(xaxis, (landmark[2], landmark[1] + yoffset), 2, (255, 255, 255), -1)
            cv2.putText(zaxis, names[idx], (landmark[0], landmark[1] + yoffset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(yaxis, names[idx], (landmark[0], landmark[2] + yoffset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(xaxis, names[idx], (landmark[2], landmark[1] + yoffset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            if last is not None:
                cv2.line(zaxis, (landmark[0], landmark[1] + yoffset), (last[0], last[1] + yoffset), (255, 255, 255), 1)
                cv2.line(yaxis, (landmark[0], landmark[2] + yoffset), (last[0], last[2] + yoffset), (255, 255, 255), 1)
                cv2.line(xaxis, (landmark[2], landmark[1] + yoffset), (last[2], last[1] + yoffset), (255, 255, 255), 1)
            last = landmark

        # Draw torso center in each view
        cv2.circle(zaxis, (cp[0], cp[1] + yoffset), 2, (255, 255, 0), -1)
        cv2.circle(yaxis, (cp[0], cp[2] + yoffset), 2, (255, 255, 0), -1)
        cv2.circle(xaxis, (cp[2], cp[1] + yoffset), 2, (255, 255, 0), -1)

        # Draw normal line
        cv2.line(zaxis, (cp[0], cp[1] + yoffset), (ncp[0], ncp[1] + yoffset), (255, 255, 0), 2)
        cv2.line(yaxis, (cp[0], cp[2] + yoffset), (ncp[0], ncp[2] + yoffset), (255, 255, 0), 2)
        cv2.line(xaxis, (cp[2], cp[1] + yoffset), (ncp[2], ncp[1] + yoffset), (255, 255, 0), 2)

    # Show the debug views
    cv2.imshow('YZ Plane (Side View)', xaxis)
    cv2.imshow('XZ Plane (Top View)', yaxis)
    cv2.imshow('XY Plane (Front View)', zaxis)

    # Move the windows over to the left side of the main window and stack them vertically
    if not hasattr(drawDebugViews, "views_moved"):
        cv2.moveWindow('YZ Plane (Side View)', 0, 512)
        cv2.moveWindow('XZ Plane (Top View)', 0, 256)
        cv2.moveWindow('XY Plane (Front View)', 0, 0)
        drawDebugViews.views_moved = True

def unitario(fin,inicio = np.array([0, 0, 0])):
    vector_ = fin-inicio
    return vector_/np.linalg.norm(vector_)

def angle(a, b, c):
    # a, b y c: puntos como np.array([x, y, z])
    ba = a - b
    bc = c - b
    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    return angle


def calculo_angulos(results):
    
    if results.pose_world_landmarks is not None:
        last = None
        names = ['Wrist', 'Elbow', 'Shoulder']
        joints = [mp_pose.PoseLandmark.RIGHT_WRIST, mp_pose.PoseLandmark.RIGHT_ELBOW, mp_pose.PoseLandmark.RIGHT_SHOULDER]

        # Put all the world landmark positions for the joints into numpy array
        world_landmarks = np.array(
            [[-results.pose_world_landmarks.landmark[i].z, results.pose_world_landmarks.landmark[i].x, -results.pose_world_landmarks.landmark[i].y] for i in joints])

        # Extraer las coordenadas del hombro y el codo
        hombro_derecho = world_landmarks[2]  # Coordenadas del hombro derecho
        codo_derecho = world_landmarks[1]    # Coordenadas del codo derecho
        muneca_derecha = world_landmarks[0]  # Coordenadas de la muneca derecha

        print(muneca_derecha)
        return muneca_derecha

# Main code

rospy.init_node('angle_publisher', anonymous=True)
pub = rospy.Publisher('desired_position', Float64MultiArray, queue_size=10)

rate = rospy.Rate(100) # 20hz


parser = argparse.ArgumentParser()
parser.add_argument('--nodebug', action='store_true', help='Disable debug views')
args = parser.parse_args()
show_debug_views = not args.nodebug

cvcam = opencv_cam.OpenCVCam(cam_id=2,width=1920, height=1080)  # Use webcam by default

if cvcam.start() is False:
    print("Failed to start video capture - exiting.")
    exit()

# Process the video stream
with mp_holistic.Holistic(
    static_image_mode=False,
    model_complexity=2,
    enable_segmentation=True,
    smooth_segmentation=True,
    refine_face_landmarks=False,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75) as holistic:
    while cvcam.is_opened():
        success, image = cvcam.read_frame()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = holistic.process(image)

        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_holistic.POSE_CONNECTIONS,
                                  landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())
        mp_drawing.draw_landmarks(image, results.right_hand_landmarks, mp_holistic.HAND_CONNECTIONS,
                                  landmark_drawing_spec=mp_drawing_styles.get_default_hand_landmarks_style())

        image = cv2.resize(image, (1280, 720))

        if results.pose_world_landmarks:
            drawDebugViews(results, None, None, None, None, None)
            angulo = np.array([1.57079633, 0, 1.57079633])
            pose = np.concatenate((calculo_angulos(results),angulo))
            msg = Float64MultiArray()

            msg.data = pose
            
            # Publica el mensaje
            pub.publish(msg)


        flipped_image = cv2.flip(image, 1)
        cv2.imshow('MediaPipe Pose', flipped_image)

        if cv2.waitKey(5) & 0xFF == 27:
            break

cvcam.stop()
cv2.destroyAllWindows()
