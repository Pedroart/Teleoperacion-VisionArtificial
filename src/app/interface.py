#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

import cv2
import mediapipe as mp
import time
import numpy as np

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
        self.last_detected_gesture = None
        self.gesture_start_time = None
        self.gesture_end_time = None
        self.detection_threshold = 0.2  # Tiempo mínimo en segundos para validar el gesto
        self.reset_threshold = 0.5     # Tiempo mínimo sin detección para pasar a None

    def add_gesture(self, gesture_name, finger_state, action):
        self.gestures[gesture_name] = {
            "finger_state": finger_state,
            "action": action
        }

    def detect_gesture(self, finger_state):
        current_time = time.time()

        # Verifica si algún gesto coincide con el estado de los dedos actual
        for gesture_name, gesture_info in self.gestures.items():
            if gesture_info["finger_state"] == finger_state:
                # Si el mismo gesto fue detectado previamente
                if self.last_detected_gesture == gesture_name:
                    # Calcula cuánto tiempo lleva siendo consistente
                    elapsed_time = current_time - self.gesture_start_time
                    if elapsed_time >= self.detection_threshold:
                        # Ejecuta la acción asociada al gesto
                        gesture_info["action"]()
                        return gesture_name
                else:
                    # Nuevo gesto detectado, reinicia el temporizador
                    self.last_detected_gesture = gesture_name
                    self.gesture_start_time = current_time
                self.gesture_end_time = None  # Reinicia el fin del temporizador
                return None  # No ejecuta hasta que pase el tiempo de detección

        # Si no se detecta un gesto válido, manejar el temporizador de finalización
        if self.last_detected_gesture:
            if self.gesture_end_time is None:
                self.gesture_end_time = current_time
            elif current_time - self.gesture_end_time >= self.reset_threshold:
                # Si el tiempo sin detección supera el umbral, reinicia
                self.last_detected_gesture = None
                self.gesture_start_time = None
                self.gesture_end_time = None

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
    linemax = 1
    radio_trabajo = 150

    def __init__(self):
        rospy.init_node('interface', anonymous=True)
        
        self.pub = rospy.Publisher('pointer', Point, queue_size=10)
        
        self.rate = rospy.Rate(24)

        self.cap = cv2.VideoCapture(index=0)

        self.hand_control = 'Right'
        self.hand = Hand()  # Instancia de la clase Hand
        
        self.gesture = Gesture()
        self.gesture.add_gesture("Cambiar", [1, 1, 0, 0, 1], self.changer_hand_control)
        self.gesture.add_gesture("Puntero", [1, 1, 1, 0, 0], self.punto)
        self.gesture.add_gesture("Ajuste", [0, 1, 1, 0, 0], self.offset)
        self.gesture.add_gesture("Radio", [1, 1, 0, 0, 0], self.ajuste)

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
    
    def ajuste(self):
        """
        Dibuja un punto entre las puntas del dedo índice y pulgar, y una línea entre ellos.

        :param self.image: self.imagen donde se dibujará el círculo y la línea (formato OpenCV).
        :param landmarks: Lista de landmarks de la mano detectada.
        """
        # Índices de las puntas del dedo índice y pulgar
        index_tip = 8
        thumb_tip = 4

        # Coordenadas 3D de las puntas del dedo índice y pulgar
        x1, y1, z1 = (
            int(self.landmarks[index_tip].x * self.image.shape[1]), 
            int(self.landmarks[index_tip].y * self.image.shape[0]), 
            self.landmarks[index_tip].z * self.image.shape[1]
        )
        x2, y2, z2 = (
            int(self.landmarks[thumb_tip].x * self.image.shape[1]), 
            int(self.landmarks[thumb_tip].y * self.image.shape[0]), 
            self.landmarks[thumb_tip].z * self.image.shape[1]
        )

        line_length = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
        if(line_length>self.linemax):
            self.linemax = line_length
        
        self.radio_trabajo = int(150 * line_length / self.linemax)
        

        # Dibujar la línea entre las puntas del índice y el pulgar
        cv2.line(self.image, (x1, y1), (x2, y2), (0, 255, 0), 2)  # Línea verde con grosor 2


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

    
        self.publish_position(
                                np.clip( (-(cx-self.scx)/self.radio_trabajo),-1.2, 1.2) ,
                                np.clip( (-(cy-self.scy)/self.radio_trabajo),-1.2, 1.2) ,
                                np.clip( ( (cz-self.scz)/self.radio_trabajo),-1.2, 1.2) )

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


                # Crear una capa temporal para el círculo transparente
                overlay = self.image.copy()
                cv2.circle(overlay, (self.scx, self.scy), int(self.radio_trabajo*1.2), (128, 128, 0), -1)  # Dibuja el círculo en la capa temporal
                cv2.circle(overlay, (self.scx, self.scy), self.radio_trabajo, (128, 128, 128), -1)  # Dibuja el círculo en la capa temporal


                # Mezclar la capa temporal con la imagen original para el círculo transparente
                alpha = 0.5  # Transparencia (0.5 para semitransparente)
                cv2.addWeighted(overlay, alpha, self.image, 1 - alpha, 0, self.image)


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
    

    