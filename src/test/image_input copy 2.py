import mediapipe as mp
import cv2
import matplotlib.pyplot as plt
import numpy as np

# Setup MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Open webcam
cap = cv2.VideoCapture(2)  # Cambia el índice si es necesario

# Lists to store wrist positions for real-time plotting
wrist_positions_xy = []
wrist_positions_yz = []

# Setup the figure and axes for live plotting
plt.ion()  # Enable interactive mode for real-time plotting
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
        # Capture frame and process it
        success, image = cap.read()
        if not success:
            break

        # Pose estimation
        results = pose.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        wrist_position, _ = calculo_posiciones(results)

        # If wrist position is detected, update the plot
        if wrist_position is not None:
            # Store positions for plotting
            wrist_positions_xy.append((wrist_position[0], wrist_position[1]))  # X, Y
            wrist_positions_yz.append((wrist_position[1], wrist_position[2]))  # Y, Z

            # Clear and replot each frame for smooth updating
            ax1.cla()
            ax2.cla()
            
            # Update plot titles and labels
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
            
            # Plot new data
            ax1.plot([p[0] for p in wrist_positions_xy], [p[1] for p in wrist_positions_xy], 'b-')
            ax2.plot([p[0] for p in wrist_positions_yz], [p[1] for p in wrist_positions_yz], 'g-')
            
            plt.draw()
            plt.pause(0.01)  # Small pause to update the plot

        # Display live webcam image
        cv2.imshow("MediaPipe Pose", cv2.flip(image, 1))

        # Close with 'ESC'
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
plt.ioff()  # Turn off interactive mode
plt.show()  # Keep the plot open after the loop ends
