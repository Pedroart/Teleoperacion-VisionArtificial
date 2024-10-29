import rospy
from geometry_msgs.msg import PointStamped
import cv2
import mediapipe as mp
import numpy as np
from filterpy.kalman import KalmanFilter

class MediaPipeHandPublisher:

    indice_camara = 2
    mano = 1  # Derecha = 0, Izquierda = 1

    def __init__(self):
        rospy.init_node('mediapipe_landmark_publisher', anonymous=True)
        
        self.pub_wrist = rospy.Publisher('landmark_wrist', PointStamped, queue_size=10)
        self.pub_elbow = rospy.Publisher('landmark_elbow', PointStamped, queue_size=10)

        self.cap = cv2.VideoCapture(self.indice_camara)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=2,
            enable_segmentation=True,
            smooth_segmentation=True,
            refine_face_landmarks=False,
            min_detection_confidence=0.75,
            min_tracking_confidence=0.75
        )

        # Configuración de los filtros de Kalman para la muñeca y el codo
        self.kf_wrist = self.create_kalman_filter()
        self.kf_elbow = self.create_kalman_filter()

    def create_kalman_filter(self):
        # Crea un filtro de Kalman básico para 3D (posición en x, y, z)
        kf = KalmanFilter(dim_x=6, dim_z=3)  # 6 estados (posición y velocidad) y 3 medidas (x, y, z)
        
        dt = 0.01  # intervalo de tiempo entre mediciones
        kf.F = np.array([[1, dt, 0,  0,  0,  0],
                         [0,  1, 0,  0,  0,  0],
                         [0,  0, 1, dt,  0,  0],
                         [0,  0, 0,  1,  0,  0],
                         [0,  0, 0,  0,  1, dt],
                         [0,  0, 0,  0,  0,  1]])
        
        kf.H = np.array([[1, 0, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0],
                         [0, 0, 0, 0, 1, 0]])

        kf.P *= 10  # incertidumbre inicial
        kf.R *= 0.1  # ruido de medida
        kf.Q *= 0.1  # ruido del proceso
        
        return kf

    def run(self):
        rate = rospy.Rate(60)

        while not rospy.is_shutdown() and self.cap.isOpened():
            success, image = self.cap.read()
            if not success:
                rospy.loginfo("No se pudo capturar el frame de la cámara.")
                continue

            image.flags.writeable = False
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = self.holistic.process(image_rgb)

            joints = []
            if results.pose_world_landmarks is not None:
                
                if self.mano == 0 and results.right_hand_landmarks:  # Mano derecha
                    
                    joints = [self.mp_holistic.PoseLandmark.RIGHT_WRIST, self.mp_holistic.PoseLandmark.RIGHT_ELBOW]

                elif self.mano == 1 and results.left_hand_landmarks:  # Mano izquierda
                    
                    joints = [self.mp_holistic.PoseLandmark.LEFT_WRIST, self.mp_holistic.PoseLandmark.LEFT_ELBOW]

                world_landmarks = np.array(
                    [[-results.pose_world_landmarks.landmark[i].z, 
                    results.pose_world_landmarks.landmark[i].x, 
                    -results.pose_world_landmarks.landmark[i].y] for i in joints])

                if len(joints) > 0:
                    # Aplicar el filtro de Kalman a la muñeca
                    self.kf_wrist.predict()
                    self.kf_wrist.update(world_landmarks[1])
                    wrist_filtered = self.kf_wrist.x[:3]  # Coordenadas x, y, z filtradas
                    
                    # Publicar el punto de la muñeca
                    p_wrist = PointStamped()
                    p_wrist.header.stamp = rospy.Time.now()
                    p_wrist.header.frame_id = "world"
                    p_wrist.point.x, p_wrist.point.y, p_wrist.point.z = wrist_filtered
                    self.pub_wrist.publish(p_wrist)

                    # Aplicar el filtro de Kalman al codo
                    self.kf_elbow.predict()
                    self.kf_elbow.update(world_landmarks[0])
                    elbow_filtered = self.kf_elbow.x[:3]  # Coordenadas x, y, z filtradas

                    # Publicar el punto del codo
                    p_elbow = PointStamped()
                    p_elbow.header.stamp = rospy.Time.now()
                    p_elbow.header.frame_id = "world"
                    p_elbow.point.x, p_elbow.point.y, p_elbow.point.z = elbow_filtered
                    self.pub_elbow.publish(p_elbow)

            image.flags.writeable = True
            image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)

            cv2.imshow('MediaPipe Holistic', cv2.flip(image_bgr, 1))
            if cv2.waitKey(1) & 0xFF == 27:  # Presionar ESC para salir
                break

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

    def __del__(self):
        self.holistic.close()
        self.cap.release()

if __name__ == '__main__':
    try:
        nodo = MediaPipeHandPublisher()
        nodo.run()
    except rospy.ROSInterruptException:
        pass
