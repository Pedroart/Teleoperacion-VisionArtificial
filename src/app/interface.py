import rospy
from geometry_msgs.msg import Point

import cv2
import mediapipe as mp
    

'''
Enviador datos:
    Posicion: (array float)
    Acciones:
'''

'''
Todo:
    - Detectar si los dedos estan estirados o no (si)
    - Detectar gestos como:
        - Dedo indice y medio como punteros
        - pausar cuando solo este el dedo indice
    - Ver el caso de los dedos cuando una mano no se ve
'''

class Gesture:
    def __init__(self):
        self.gestures = {}
    
    def add_gesture(self, gesture_name, finger_state, action):
        self.gestures[gesture_name] = {
            "finger_state": finger_state,
            "action": action
        }

    def detect_gesture(self, finger_state):
        for gesture_name, gesture_info in self.gestures.items():
            if gesture_info["finger_state"] == finger_state:
                # Ejecuta la acción asociada al gesto
                gesture_info["action"]()
                return gesture_name
        return None

class Hand:

    def __init__(self):
        self.left_hand = []
        self.right_hand = []
        self.left_fingers = [0, 0, 0, 0, 0]  
        self.right_fingers = [0, 0, 0, 0, 0]

    def classify_hands(self, handedness, landmark):
        if handedness == "Left":
            self.left_fingers = self.detect_fingers(handedness,landmark)
            self.left_hand = landmark
        elif handedness == "Right":
            self.right_fingers = self.detect_fingers(handedness,landmark)
            self.right_hand = landmark
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

    def get_hand_data(self, handedness):
        if handedness == "Left":
            return (self.left_hand,self. left_fingers)
        elif handedness == "Right":
            return (self.right_hand, self.right_fingers)
        else:
            rospy.logwarn("Handedness no válido. Debe ser 'Left' o 'Right'.")
            return (None,None)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Interface:

    mp_drawing = mp.solutions.drawing_utils
    mp_drawing_styles = mp.solutions.drawing_styles
    mp_hands = mp.solutions.hands

    scx, scy, scz = (0,0,0)

    def __init__(self):
        self.pub = rospy.Publisher('pointer', Point, queue_size=10)
        
        rospy.init_node('interface', anonymous=True)
        self.rate = rospy.Rate(60)

        self.cap = cv2.VideoCapture(index=2)

        self.hand_control = 'Right'
        self.hand = Hand()  # Instancia de la clase Hand
        
        self.gesture = Gesture()
        self.gesture.add_gesture("Saludo", [1, 1, 0, 0, 1], self.changer_hand_control)
        self.gesture.add_gesture("Puntero", [1, 1, 1, 0, 0], self.punto)
        self.gesture.add_gesture("Ajuste", [0, 1, 1, 0, 0], self.offset)

        # Configurar la gráfica 3D
        #self.fig = plt.figure()
        #self.ax = self.fig.add_subplot(111, projection='3d')
        #plt.ion()  # Activar el modo interactivo

    def changer_hand_control(self):
        #print(self.hand_control)
        if self.hand_control == 'Left':
            self.hand_control = 'Right'
        elif self.hand_control == 'Right':
            self.hand_control = 'Left'
            print(self.hand_control)
    
    def offset(self):
        """
        Dibuja un círculo entre las puntas del dedo índice y medio.

        :param self.image: self.imagen donde se dibujará el círculo (formato OpenCV).
        :param landmarks: Lista de landmarks de la mano detectada.
        """
        # Índices de las puntas del dedo índice y medio
        index_tip = 8
        middle_tip = 12

        # Coordenadas 3D de las puntas del dedo índice y medio
        x1, y1, z1 = (
            int(self.landmarks[index_tip].x * self.image.shape[1]), 
            int(self.landmarks[index_tip].y * self.image.shape[0]), 
            self.landmarks[index_tip].z * self.image.shape[1]
        )
        x2, y2, z2 = (
            int(self.landmarks[middle_tip].x * self.image.shape[1]), 
            int(self.landmarks[middle_tip].y * self.image.shape[0]), 
            self.landmarks[middle_tip].z * self.image.shape[1]
        )

        # Calcular el punto medio entre las coordenadas 3D de ambos dedos
        self.scx, self.scy, self.scz = (x1 + x2) // 2, (y1 + y2) // 2, (z1 + z2) / 2


        #self.publish_position(self.scx,self.scy,self.scz)

        # Dibujar el círculo en el punto medio
        cv2.circle(self.image, (self.scx, self.scy), 10, (155, 0, 0), cv2.FILLED)     

    def punto(self):
        """
        Dibuja un círculo entre las puntas del dedo índice y medio.

        :param self.image: self.imagen donde se dibujará el círculo (formato OpenCV).
        :param landmarks: Lista de landmarks de la mano detectada.
        """
        # Índices de las puntas del dedo índice y medio
        index_tip = 8
        middle_tip = 12

        # Coordenadas 3D de las puntas del dedo índice y medio
        x1, y1, z1 = (
            int(self.landmarks[index_tip].x * self.image.shape[1]), 
            int(self.landmarks[index_tip].y * self.image.shape[0]), 
            self.landmarks[index_tip].z * self.image.shape[1]
        )
        x2, y2, z2 = (
            int(self.landmarks[middle_tip].x * self.image.shape[1]), 
            int(self.landmarks[middle_tip].y * self.image.shape[0]), 
            self.landmarks[middle_tip].z * self.image.shape[1]
        )

        # Calcular el punto medio entre las coordenadas 3D de ambos dedos
        cx, cy, cz = (x1 + x2) // 2, (y1 + y2) // 2, (z1 + z2) / 2


        self.publish_position(cx-self.scx,cy-self.scy,cz-self.scz)

        # Dibujar el círculo en el punto medio
        cv2.circle(self.image, (cx, cy), 10, (255, 0, 0), cv2.FILLED)  # Color verde con relleno

    def publish_position(self, x, y, z):
        """
        Publica la posición (x, y, z) en el tópico 'pointer'.
        """
        point = Point()
        point.x = x
        point.y = y
        point.z = z

        #rospy.loginfo(f"Publicando posición: x={x}, y={y}, z={z}")
        self.pub.publish(point)

    def plot_hand_3d(self, landmarks):
        """
        Actualiza el gráfico 3D con los landmarks de la mano.

        :param landmarks: Lista de landmarks de la mano.
        """
        self.ax.clear()  # Limpia la gráfica anterior
        x = [lm.x for lm in landmarks]
        y = [lm.y for lm in landmarks]
        z = [lm.z for lm in landmarks]

        # Invertir ejes para que coincidan con el sistema de coordenadas de MediaPipe
        self.ax.scatter(x, y, z, c='b', marker='o')  # Dibujar los puntos
        self.ax.set_xlim([0, 1])
        self.ax.set_ylim([0, 1])
        self.ax.set_zlim([0, 1])
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        plt.draw()  # Dibujar la gráfica
        plt.pause(0.01)  # Pausa para actualizar

    def run(self):
        with self.mp_hands.Hands(
                model_complexity=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5) as hands:
            while not rospy.is_shutdown() and self.cap.isOpened():
                success, self.image = self.cap.read()
                if not success:
                    rospy.logwarn("Advertencia: No se detectó la cámara.")
                    continue
                
                self.image.flags.writeable = False
                self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
                results = hands.process(self.image)
                
                self.image.flags.writeable = True
                self.image = cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR)

                if results.multi_hand_landmarks and results.multi_handedness:
                    for idx, hand_landmarks in enumerate(results.multi_hand_landmarks):
                        
                        handedness = results.multi_handedness[idx].classification[0].label
                        
                        self.hand.classify_hands(handedness, hand_landmarks.landmark)

                        if self.hand_control == handedness:
                            # Dibujar los landmarks en la self.imagen
                            '''
                            self.mp_drawing.draw_landmarks(
                                self.image,
                                hand_landmarks,
                                self.mp_hands.HAND_CONNECTIONS,
                                self.mp_drawing_styles.get_default_hand_landmarks_style(),
                                self.mp_drawing_styles.get_default_hand_connections_style())

                            # Actualizar el plot 3D en tiempo real
                            self.plot_hand_3d(hand_landmarks.landmark)
                            '''
                            pass

                    self.landmarks, fingers = self.hand.get_hand_data(self.hand_control)
                    self.gesture.detect_gesture(fingers)

                cv2.imshow('MediaPipe Hands', cv2.flip(self.image, 1))
                if cv2.waitKey(5) & 0xFF == 27:
                    break
                self.rate.sleep()
        self.cap.release()
        plt.ioff()  # Desactivar modo interactivo
        plt.show()  # Mostrar la gráfica final

if __name__ == '__main__':
    try:
        app = Interface()
        app.run()
    except rospy.ROSInterruptException:
        pass
    

    