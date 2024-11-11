import rospy
from std_msgs.msg import String

import cv2
import mediapipe as mp
    

'''
Enviador datos:
    Posicion: (array float)
    Acciones:
'''

'''
Todo:
    - Detectar si los dedos estan estirados o no
    - Detectar gestos como:
        - Dedo indice y medio como punteros
        - pausar cuando solo este el dedo indice
'''
class Hand:

    def __init__(self):
        self.left_hand = []
        self.right_hand = []
        self.left_fingers = [0, 0, 0, 0, 0]  
        self.right_fingers = [0, 0, 0, 0, 0]

    def classify_hands(self, handedness, landmark):
        if handedness == "Left":
            self.left_hand = landmark
            self.left_fingers = self.detect_fingers(handedness,landmark)
        elif handedness == "Right":
            self.right_hand = landmark
            self.right_fingers = self.detect_fingers(handedness,landmark)
        else:
            rospy.logwarn("Advertencia: Mano no identificada correctamente.")
    
    def detect_fingers(self, handedness,landmarks):
        """
        Detecta qué dedos están levantados basándose en los landmarks.

        :param landmarks: Lista de landmarks de la mano.
        :return: Lista con el estado de los dedos (0: abajo, 1: levantado).
        """
        fingers = [0, 0, 0, 0, 0] 
        tip_ids = [4, 8, 12, 16, 20]  # Índices de las puntas de los dedos

        # Pulgar (comparación en eje x)
        if handedness == "Left":
            # Mano izquierda: el pulgar apunta hacia la izquierda si está levantado
            if landmarks[tip_ids[0]].x > landmarks[tip_ids[0] - 1].x:
                fingers[0] = 1
        else:
            # Mano derecha: el pulgar apunta hacia la derecha si está levantado
            if landmarks[tip_ids[0]].x < landmarks[tip_ids[0] - 1].x:
                fingers[0] = 1

        # Índice, medio, anular y meñique (comparación en eje y)
        for i in range(1, 5):
            if landmarks[tip_ids[i]].y < landmarks[tip_ids[i] - 2].y:
                fingers[i] = 1

        return fingers

class Interface:

    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands

    def __init__(self):
        self.pub = rospy.Publisher('interface', String, queue_size=10)
        rospy.init_node('pointer', anonymous=True)
        self.rate = rospy.Rate(60)

        self.cap = cv2.VideoCapture(index=0)

        self.hand = Hand()  # Instancia de la clase Hand

    def run(self):
        with self.mp_hands.Hands(
                model_complexity=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as hands:
            while not rospy.is_shutdown() and self.cap.isOpened():
                success, image = self.cap.read()
                if not success:
                    rospy.logwarn("Advertencia: No se detectó la cámara.")
                    continue
                
                image.flags.writeable = False
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = hands.process(image)
                
                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

                if results.multi_hand_landmarks and results.multi_handedness:
                    for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):

                        handedness = results.multi_handedness[idx].classification[0].label
                         
                        self.hand.classify_hands(handedness, hand_landmarks.landmark)

                        # Dibujar los landmarks en la imagen
                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())

                print(self.hand.right_fingers)


                cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                    break
                self.rate.sleep()
        self.cap.release()

if __name__ == '__main__':
    try:
        app = Interface()
        app.run()
    except rospy.ROSInterruptException:
        pass
    

    