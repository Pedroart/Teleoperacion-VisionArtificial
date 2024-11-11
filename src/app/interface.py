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
class hand:
    infer = [0, #pulgar
             0, #indice
             0, #medio
             0, #anular
             0  #menique
            ]
    
    def __init__(self):
        pass

    

class interface:

    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands

    def __init__(self):
        self.pub = rospy.Publisher('interface', String, queue_size=10)
        rospy.init_node('pointer', anonymous=True)
        self.rate = rospy.Rate(60)

        self.cap = cv2.VideoCapture(index=0)

        desired_fps = 60
        self.cap.set(cv2.CAP_PROP_FPS, desired_fps)
        
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"FPS configurados: {60}, FPS reales: {actual_fps}")

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
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        self.mp_drawing.draw_landmarks(
                            image,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            self.mp_drawing_styles.get_default_hand_landmarks_style(),
                            self.mp_drawing_styles.get_default_hand_connections_style())

                cv2.imshow('MediaPipe Hands', cv2.flip(image, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                    break
                self.rate.sleep()
        self.cap.release()

if __name__ == '__main__':
    try:
        app = interface()
        app.run()
    except rospy.ROSInterruptException:
        pass
    

    