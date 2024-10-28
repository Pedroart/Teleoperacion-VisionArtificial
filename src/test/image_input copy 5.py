import mediapipe as mp
import cv2
import matplotlib.pyplot as plt
import numpy as np
from filterpy.kalman import KalmanFilter

# Configurar MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Inicializar Kalman Filters para x, y, z de la muñeca
def inicializar_kalman():
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.x = np.array([[0.], [0.]])  # Posición y velocidad iniciales
    kf.F = np.array([[1., 1.], [0., 1.]])  # Matriz de transición de estado
    kf.H = np.array([[1., 0.]])  # Matriz de observación
    kf.P *= 1000.  # Error inicial en la estimación
    kf.R = 5  # Error de medición
    kf.Q = np.array([[1., 0.], [0., 1.]])  # Ruido del proceso
    return kf

# Crear tres filtros de Kalman para x, y, z
kf_x = inicializar_kalman()
kf_y = inicializar_kalman()
kf_z = inicializar_kalman()

# Configurar la cámara
cap = cv2.VideoCapture(2)

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
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            break

        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        wrist_position, _ = calculo_posiciones(results)

        if wrist_position is not None:
            # Aplicar ganancia al eje X
            x_pos = wrist_position[0] * ganancia_x
            y_pos = wrist_position[1]
            z_pos = wrist_position[2]

            # Filtrar cada coordenada usando el filtro de Kalman
            kf_x.predict()
            kf_x.update(x_pos)
            x_filtrado = kf_x.x[0, 0]

            kf_y.predict()
            kf_y.update(y_pos)
            y_filtrado = kf_y.x[0, 0]

            kf_z.predict()
            kf_z.update(z_pos)
            z_filtrado = kf_z.x[0, 0]

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
