import mediapipe as mp
import cv2
import matplotlib.pyplot as plt
import numpy as np
from std_msgs.msg import Float64MultiArray
import rospy
from collections import deque

# Configurar MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Configurar la cámara
cap = cv2.VideoCapture(2)

# Tamaño de la ventana de media móvil
window_size = 5  # Cambia este valor según la suavidad deseada

# Deques para almacenar las últimas posiciones (ventanas deslizantes)
window_x = deque(maxlen=window_size)
window_y = deque(maxlen=window_size)
window_z = deque(maxlen=window_size)

# Listas para almacenar posiciones de la muñeca para graficar en tiempo real
wrist_positions_xy = []
wrist_positions_yz = []

# Configurar gráfico en tiempo real
plt.ion()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))
ax1.set_title('Wrist Trajectory - XY Plane')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_xlim(-1, 1)
ax1.set_ylim(-1, 1)
ax2.set_title('Wrist Trajectory - YZ Plane')
ax2.set_xlabel('Y')
ax2.set_ylabel('Z')
ax2.set_xlim(-1, 1)
ax2.set_ylim(-1, 1)
line_xy, = ax1.plot([], [], 'b-')
line_yz, = ax2.plot([], [], 'g-')

# Definir ganancia para el eje X
ganancia_x = 2

# Inicializar nodo de ROS y publicadores
rospy.init_node('pose_publisher', anonymous=True)
wrist_pub = rospy.Publisher("/wrist_point", Float64MultiArray, queue_size=10)
elbow_pub = rospy.Publisher("/elbow_point", Float64MultiArray, queue_size=10)

def calculo_posiciones(results):
    if results.pose_world_landmarks is not None:
        joints = [mp_pose.PoseLandmark.RIGHT_WRIST, mp_pose.PoseLandmark.RIGHT_ELBOW]
        world_landmarks = np.array(
            [[-results.pose_world_landmarks.landmark[i].z, 
              results.pose_world_landmarks.landmark[i].x, 
              -results.pose_world_landmarks.landmark[i].y] for i in joints])

        wrist_position = world_landmarks[0]
        elbow_position = world_landmarks[1]
        return wrist_position, elbow_position
    return None, None

with mp_pose.Pose(
    min_tracking_confidence=0.7,
    min_detection_confidence=0.7,
    model_complexity=2,
    smooth_landmarks=True,
) as pose:
    while cap.isOpened() and not rospy.is_shutdown():
        success, image = cap.read()
        if not success:
            break

        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        wrist_position, elbow_position = calculo_posiciones(results)

        if wrist_position is not None:
            # Aplicar ganancia al eje X
            x_pos = wrist_position[0] * ganancia_x
            y_pos = wrist_position[1]
            z_pos = wrist_position[2]

            # Agregar posiciones actuales a las ventanas deslizantes
            window_x.append(x_pos)
            window_y.append(y_pos)
            window_z.append(z_pos)

            # Calcular las medias de las ventanas deslizantes
            x_filtrado = np.mean(window_x)
            y_filtrado = np.mean(window_y)
            z_filtrado = np.mean(window_z)

            # Publicar posiciones filtradas de la muñeca y el codo
            wrist_msg = Float64MultiArray()
            wrist_msg.data = [x_filtrado, y_filtrado, z_filtrado]
            wrist_pub.publish(wrist_msg)

            elbow_msg = Float64MultiArray()
            elbow_msg.data = elbow_position.tolist()
            elbow_pub.publish(elbow_msg)

            # Agregar posiciones filtradas a las listas para graficar
            wrist_positions_xy.append((x_filtrado, y_filtrado))  # X, Y
            wrist_positions_yz.append((y_filtrado, z_filtrado))  # Y, Z

            if len(wrist_positions_xy) > 50:
                wrist_positions_xy.pop(0)
            if len(wrist_positions_yz) > 50:
                wrist_positions_yz.pop(0)

            # Actualizar líneas en el gráfico
            line_xy.set_data([p[0] for p in wrist_positions_xy], [p[1] for p in wrist_positions_xy])
            line_yz.set_data([p[0] for p in wrist_positions_yz], [p[1] for p in wrist_positions_yz])

            # Redibujar solo las líneas actualizadas
            ax1.draw_artist(line_xy)
            ax2.draw_artist(line_yz)
            fig.canvas.blit(ax1.bbox)
            fig.canvas.blit(ax2.bbox)
            fig.canvas.flush_events()

        # Mostrar imagen de la cámara en vivo
        cv2.imshow("MediaPipe Pose", cv2.flip(image, 1))

        # Cerrar con 'ESC'
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
plt.ioff()
plt.show()
