import cv2
import mediapipe as mp

mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic

# Webcam input
cap = cv2.VideoCapture(1)
with mp_holistic.Holistic(
    static_image_mode=False,
    model_complexity=2,
    enable_segmentation=True,
    smooth_segmentation=True,
    refine_face_landmarks=False,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75) as holistic:
    
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue

        # Optimize performance by marking the image as non-writable
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = holistic.process(image)

        # Draw landmark annotation on the image
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        mp_drawing.draw_landmarks(
            image,
            results.face_landmarks,
            mp_holistic.FACEMESH_CONTOURS,
            landmark_drawing_spec=None,
            connection_drawing_spec=mp_drawing_styles.get_default_face_mesh_contours_style())
        
        mp_drawing.draw_landmarks(
            image,
            results.pose_landmarks,
            mp_holistic.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style())

        # Flip the image horizontally for a selfie-view display
        cv2.imshow('MediaPipe Holistic', cv2.flip(image, 1))
        
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
