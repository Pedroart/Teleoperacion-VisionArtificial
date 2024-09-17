import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap.dialogs import Messagebox

import threading
import cv2
from PIL import Image, ImageTk
import numpy as np
import time

from postura import *
from _camara import *

class App(ttk.Window):
    def __init__(self):
        super().__init__(themename="flatly")
        self.title("Luzia")
        
        self.geometry("1280x720")

        # Crear Barra de Control
        self.buttonbar = ttk.Frame(self, style=PRIMARY)
        self.buttonbar.pack(fill=X, pady=1, side=TOP)
        self.create_buttonbar()

        # Crear el notebook (conjunto de pestañas)
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill='both', expand=True)
        # Crear los frames para cada pestaña
        self.create_tabs()
        
        self.camara = camara()
        
        '''
        # Inicializar captura de cámara
        self.cap = cv2.VideoCapture(0)
        self.running = True

        # Crear un buffer para almacenar el último frame
        self.frame_buffer = None

        # Iniciar la actualización de la vista de cámara
        self.update_camera_view()
        
        # Iniciar el procesamiento en segundo plano
        self.start_background_processing()
        '''

    def create_buttonbar(self):
        _func = lambda: Messagebox.ok(message='Adding new backup')
        btn = ttk.Button(
            master= self.buttonbar, text='\u25B6 Iniciar',
            compound=LEFT, 
            command=_func
        )
        btn.pack(side=LEFT, ipadx=5, ipady=5, padx=0, pady=1)
        _func = None
        btn = ttk.Button(
            master= self.buttonbar, text='\u21BB Reiniciar',
            compound=LEFT, 
            command=_func
        )
        btn.pack(side=LEFT, ipadx=5, ipady=5, padx=0, pady=1)
        _func = None
        btn = ttk.Button(
            master= self.buttonbar, text='\u2716 Detener',
            compound=LEFT, 
            command=_func
        )
        btn.pack(side=LEFT, ipadx=5, ipady=5, padx=0, pady=1)

        btn = ttk.Button(
            master= self.buttonbar, text='(C)',
            compound=LEFT,
            bootstyle=PRIMARY
        )
        btn.pack(side=RIGHT, ipadx=5, ipady=5, padx=0, pady=1)

        btn = ttk.Button(
            master= self.buttonbar, text='(P)',
            compound=LEFT, 
        )
        btn.pack(side=RIGHT, ipadx=5, ipady=5, padx=0, pady=1)
        

    def create_tabs(self):
        self.main_frame = ttk.Frame(self.notebook, bootstyle=None)
        self.create_main_content()
        self.notebook.add(self.main_frame, text='Main')

        self.camara_frame = ttk.Frame(self.notebook, bootstyle=None)
        self.camara_estructura()
        self.notebook.add(self.camara_frame, text='Camara')

        '''
        self.postura_frame = ttk.Frame(self.notebook, bootstyle=None)
        self.create_postura_content()
        self.notebook.add(self.postura_frame, text='Postura')

        self.controlador_frame = ttk.Frame(self.notebook, bootstyle=None)
        self.create_controlador_content()
        self.notebook.add(self.controlador_frame, text='Controlador')
        '''



    def camara_estructura(self):
        # Configuracion de las columans
        self.camara_frame.grid_columnconfigure(0, weight=1)
        self.camara_frame.grid_columnconfigure(1, weight=1)

        # Columna de Vista de camaras
        self.camara_view = ttk.Labelframe(self.camara_frame,text='Vista de Camara', padding=10)
        self.camara_view.grid(row=0, column=0, padx=10, pady=10,sticky='ns')
        
        ## Imagen del recuadro
        black_image = Image.new('RGB', (64*9, 48*9), color='black')
        self.black_image_tk= ImageTk.PhotoImage(image=black_image)
        
        self.image_camara = tk.Label(self.camara_view, image=self.black_image_tk)
        self.image_camara.grid(row=0, column=0, sticky='nsew')
        
        ## Selector de disposito
        self.dispositivo_numero = ttk.Combobox(
            master=self.camara_view,
            values=[str(i) for i in range(11)],
            bootstyle="light",
            state='readonly'
        )
        

        # Columna de Vista de Mediapipe
        self.dispositivo_numero.grid(row=1, column=0, padx=10, pady=10, sticky='nsew')
        self.dispositivo_numero.set(0)

        self.dispositivo_numero.bind('<<ComboboxSelected>>', self.cambio_numero_camara)

        self.postura_view = ttk.Labelframe(self.camara_frame,text='Postura', padding=10)
        self.postura_view.grid(row=0, column=1, padx=10, pady=10,sticky='ns')
        
        self.imagen_postura = tk.Label(self.postura_view, image=self.black_image_tk)
        self.imagen_postura.grid(row=0, column=0, sticky='nsew')
    
    def cambio_numero_camara(self,event):
        self.camara.activo = False
        selected_value = self.dispositivo_numero.get()
        self.camara.set_camara(int(selected_value))
        print(self.camara.captura())


    def actualizar_imagen_camara(self,imagen):
        imagen_formato = Image.fromarray(imagen)
        self.image_camara = tk.Label(self.camara_view, image=imagen_formato)

    def actualizar_imagen_postura(self,imagen):
        imagen_formato = Image.fromarray(imagen)
        self.image_postura = tk.Label(self.camara_view, image=imagen_formato)





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
        #self.cap.release()
        self.destroy()

# Crear y ejecutar la aplicación
if __name__ == "__main__":
    app = App()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)  # Asegura que se realice limpieza al cerrar
    app.mainloop()
