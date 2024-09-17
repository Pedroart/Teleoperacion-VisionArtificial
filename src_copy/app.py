import pose as p
import cv2
import camara as cam
import time
import pose as po
import tkinter as tk
from tkinter import Frame
from PIL import Image, ImageTk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np  # Necesario para calcular la media


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Visualización de Pose")

        # Crear el contenedor para la imagen de la cámara
        self.frame_camera = Frame(root, width=640, height=480)
        self.frame_camera.grid(row=0, column=0)

        # Crear el contenedor para el gráfico 3D
        self.frame_3d = Frame(root, width=640, height=480)
        self.frame_3d.grid(row=0, column=1)

        # Crear el contenedor para el gráfico de barras
        self.frame_bar = Frame(root, width=640, height=200)
        self.frame_bar.grid(row=1, column=0, columnspan=2)

        # Crear el canvas para mostrar la imagen de la cámara
        self.canvas_camera = tk.Canvas(self.frame_camera, width=640, height=480)
        self.canvas_camera.pack()

        # Crear la figura de Matplotlib para el gráfico 3D
        self.fig_3d = plt.figure(figsize=(5, 4))
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')

        # Agregar la figura de Matplotlib al Tkinter Canvas para 3D
        self.canvas_3d = FigureCanvasTkAgg(self.fig_3d, self.frame_3d)
        self.canvas_3d.get_tk_widget().pack()

        # Crear la figura de Matplotlib para el gráfico de barras
        self.fig_bar, self.ax_bar = plt.subplots(figsize=(5, 2))

        # Inicializar las barras con valores cero
        self.bars = self.ax_bar.bar(range(7), [0] * 7)
        self.ax_bar.set_ylim(-180, 180)  # Limitar los ángulos a 180 grados

        # Agregar la figura de Matplotlib al Tkinter Canvas para las barras
        self.canvas_bar = FigureCanvasTkAgg(self.fig_bar, self.frame_bar)
        self.canvas_bar.get_tk_widget().pack()

        # Inicializar la cámara y la detección de poses
        self.camara = cam.camara()
        self.pose = po.pose()

        # Lista para almacenar el historial de ángulos (últimos 4 valores para cada ángulo)
        self.angulos_historial = [[] for _ in range(7)]  # Lista de listas para cada ángulo

        # Iniciar la actualización de la interfaz
        self.update()

    def media_movil(self, angulo_index, nuevo_valor):
        # Agregar el nuevo valor al historial de ese ángulo
        self.angulos_historial[angulo_index].append(nuevo_valor)
        
        # Si hay más de 4 valores, eliminar el más antiguo
        if len(self.angulos_historial[angulo_index]) > 4:
            self.angulos_historial[angulo_index].pop(0)

        # Calcular y devolver la media móvil (promedio) de los últimos 4 valores
        return np.mean(self.angulos_historial[angulo_index])

    def update(self):
        # Capturar imagen de la cámara
        self.camara.caputer()

        # Procesar la pose
        if self.pose.set_pose(self.camara.imagen):
            esqueleto = self.pose.get_draw_pose(self.camara.imagen)

            # Mostrar la imagen en el canvas de la cámara
            img = cv2.cvtColor(esqueleto, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(img)
            img_tk = ImageTk.PhotoImage(image=img)
            self.canvas_camera.create_image(0, 0, anchor=tk.NW, image=img_tk)
            self.canvas_camera.image = img_tk

            # Actualizar el gráfico 3D
            self.pose.normalizacion()
            self.pose.plot_world_landmarks(self.ax_3d)
            self.canvas_3d.draw()

            # Obtener los ángulos
            angulos = self.pose.get_angulos()  # Devuelve [q1, q2, q3, q4, q5, q6, q7]

            # Aplicar media móvil y actualizar el gráfico de barras
            angulos_filtrados = []
            for i, angulo in enumerate(angulos):
                angulo_suavizado = self.media_movil(i, angulo)
                angulos_filtrados.append(angulo_suavizado)

            # Actualizar las barras con los valores suavizados
            for i, bar in enumerate(self.bars):
                bar.set_height(angulos_filtrados[i])

            # Redibujar el gráfico de barras
            self.canvas_bar.draw()

        # Repetir cada 10 ms
        self.root.after(10, self.update)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
