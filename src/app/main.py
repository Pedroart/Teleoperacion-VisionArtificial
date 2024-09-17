import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
import threading
import cv2
from PIL import Image, ImageTk
import numpy as np
import time

from postura import *

class App(ttk.Window):
    def __init__(self):
        super().__init__(themename="flatly")
        self.title("Luzia")
        self.geometry("800x600")

        # Crear el notebook (conjunto de pestañas)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True)

        # Crear los frames para cada pestaña
        self.create_tabs()

        # Inicializar captura de cámara
        self.cap = cv2.VideoCapture(0)
        self.running = True

        # Crear un buffer para almacenar el último frame
        self.frame_buffer = None

        # Iniciar la actualización de la vista de cámara
        self.update_camera_view()
        
        # Iniciar el procesamiento en segundo plano
        self.start_background_processing()

    def create_tabs(self):
        self.main_frame = ttk.Frame(self.notebook, bootstyle=PRIMARY)
        self.create_main_content()
        self.notebook.add(self.main_frame, text='Main')

        self.postura_frame = ttk.Frame(self.notebook, bootstyle=SUCCESS)
        self.create_postura_content()
        self.notebook.add(self.postura_frame, text='Postura')

        self.controlador_frame = ttk.Frame(self.notebook, bootstyle=INFO)
        self.create_controlador_content()
        self.notebook.add(self.controlador_frame, text='Controlador')

    def create_main_content(self):
        """Configurar el contenido del frame principal."""
        self.camera_label = ttk.Label(self.main_frame)
        self.esqueleto_label = ttk.Label(self.main_frame)
        

        self.camera_label.grid(row=0, column=0, padx=10, pady=10)
        self.esqueleto_label.grid(row=0, column=1, padx=10, pady=10)

    def create_postura_content(self):
        label = ttk.Label(self.postura_frame, text="Contenido de la pestaña Postura", bootstyle=SUCCESS)
        label.pack(pady=20)

    def create_controlador_content(self):
        label = ttk.Label(self.controlador_frame, text="Contenido de la pestaña Controlador", bootstyle=INFO)
        label.pack(pady=20)

    def update_camera_view(self):
        """Actualiza la vista de la cámara en la interfaz gráfica y almacena la imagen en el buffer."""
        ret, frame = self.cap.read()
        if ret:
            # Guardar el frame en el buffer
            self.frame_buffer = frame

            # Convertir el frame a formato RGB y mostrarlo en la interfaz
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame_rgb)
            imgtk = ImageTk.PhotoImage(image=img)
            self.camera_label.imgtk = imgtk
            self.camera_label.configure(image=imgtk)

        if self.running:
            # Tiempo de espera de cada ejecucion de 10ms
            self.after(5, self.update_camera_view)

    def start_background_processing(self):
        """Inicia el procesamiento en segundo plano."""
        self.postura = postura()
        self.processing_thread = threading.Thread(target=self.background_processing)
        self.processing_thread.daemon = True  # Permite cerrar el hilo al cerrar la aplicación
        self.processing_thread.start()

    def background_processing(self):
        """Simula una tarea de procesamiento pesado en segundo plano usando el buffer de imagen."""
        while self.running:
            if self.frame_buffer is not None:
                # Procesar la imagen del buffer
                processed_frame = self.process_frame(self.frame_buffer)
                img = Image.fromarray(processed_frame)
                imgtk = ImageTk.PhotoImage(image=img)
                self.esqueleto_label.imgtk = imgtk
                self.esqueleto_label.configure(image=imgtk)
                # Puedes hacer algo con el frame procesado aquí (como guardar, analizar, etc.)
                print("Procesamiento de imagen completado")
            #time.sleep(1)  # Ajusta el tiempo de espera según sea necesario

    def process_frame(self, frame):
        """Ejemplo de procesamiento de imagen."""
        # Convertir la imagen a escala de grises como ejemplo de procesamiento
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        gray = self.postura.set_postura(gray)
        # Aquí se podrían agregar más operaciones de procesamiento usando GPU
        return gray

    def on_closing(self):
        """Limpieza al cerrar la aplicación."""
        self.running = False
        self.cap.release()
        self.destroy()

# Crear y ejecutar la aplicación
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)  # Asegura que se realice limpieza al cerrar
    app.mainloop()
