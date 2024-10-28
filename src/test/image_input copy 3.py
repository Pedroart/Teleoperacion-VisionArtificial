import mediapipe as mp
import cv2
import matplotlib.pyplot as plt
import numpy as np

# Configurar MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Abrir cámara
cap = cv2.VideoCapture(2)  # Cambia el índice si es necesario

# Listas para almacenar posiciones de la muñeca para graficar en tiempo real
wrist_positions_xy = []
wrist_positions_yz = []

# Configurar figura y ejes para el gráfico en tiempo real
plt.ion()  # Activar modo interactivo para graficado en tiempo real
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

# Configurar los ejes inicialmente
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

# Inicializar líneas de trayectoria para actualización en tiempo real
line_xy, = ax1.plot([], [], 'b-')
line_yz, = ax2.plot([], [], 'g-')

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
    min_tracking_confidence=0.5,
    min_detection_confidence=0.5,
    model_complexity=2,
    smooth_landmarks=True,
) as pose:
    while cap.isOpened():
        # Capturar fotograma y procesarlo
        success, image = cap.read()
        if not success:
            break

        # Estimación de pose
        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        wrist_position, _ = calculo_posiciones(results)

        # Si se detecta la posición de la muñeca, actualizar la gráfica
        if wrist_position is not None:
            # Agregar nuevas posiciones a las listas
            wrist_positions_xy.append((wrist_position[0], wrist_position[1]))  # X, Y
            wrist_positions_yz.append((wrist_position[1], wrist_position[2]))  # Y, Z

            # Limitar el número de puntos en las listas para mejorar rendimiento
            if len(wrist_positions_xy) > 50:
                wrist_positions_xy.pop(0)
            if len(wrist_positions_yz) > 50:
                wrist_positions_yz.pop(0)

            # Actualizar los datos de las líneas
            line_xy.set_data([p[0] for p in wrist_positions_xy], [p[1] for p in wrist_positions_xy])
            line_yz.set_data([p[0] for p in wrist_positions_yz], [p[1] for p in wrist_positions_yz])

            # Redibujar solo las líneas actualizadas
            ax1.draw_artist(line_xy)
            ax2.draw_artist(line_yz)
            fig.canvas.blit(ax1.bbox)
            fig.canvas.blit(ax2.bbox)
            fig.canvas.flush_events()  # Actualizar gráfico sin hacer pausa larga

        # Mostrar imagen de la cámara en vivo
        cv2.imshow("MediaPipe Pose", cv2.flip(image, 1))

        # Cerrar con 'ESC'
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
plt.ioff()  # Desactivar modo interactivo
plt.show()  # Mantener el gráfico abierto después de que termine el bucle
