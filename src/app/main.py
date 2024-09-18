import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap.dialogs import Messagebox

import threading
import queue
import cv2
from PIL import Image, ImageTk
import numpy as np
import time

from _pose import *
from _camara import *

size = (64*9, 48*9)



def medir_tiempo(func):
    def wrapper(*args, **kwargs):
        start = time.time()  # Tiempo de inicio
        result = func(*args, **kwargs)  # Ejecuta la función original
        end = time.time()  # Tiempo de fin
        print(f"Tiempo de ejecución de {func.__name__}: {end - start:.6f} segundos")
        return result  # Retorna el resultado de la función original
    return wrapper


class App(ttk.Window):
    optimizar = 90
    running = False

    def __init__(self):
        #Variables del sistema 

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
        self.pestana = 'Main'
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_change)

        self.running = True
        self.video_buffer = queue.Queue(maxsize=5)
        self.image_pose_buffer = queue.Queue(maxsize=5)

        self.camara = camara()
        self.pose = pose()
        
        
        self.hilo_demonio = threading.Thread(target=self.procesar_video_buffer, daemon=True)
        self.hilo_demonio.start()

        self.evento_captura_imagen()
        self.evento_imagen_postura()
        '''
        # Inicializar captura de cámara
        self.cap = cv2.VideoCapture(0)
        

        # Crear un buffer para almacenar el último frame
        self.frame_buffer = None

        # Iniciar la actualización de la vista de cámara
        self.update_camera_view()
        
        # Iniciar el procesamiento en segundo plano
        self.start_background_processing()
        '''

    def on_tab_change(self, event):
        # Obtener el índice de la pestaña activa
        tab_id = self.notebook.index(self.notebook.select())
        self.pestana = self.notebook.tab(tab_id, "text")

    
    def procesar_video_buffer(self):
        # Este método se ejecutará en un hilo demonio
        while self.running:
            if not self.video_buffer.empty():
                inicio = time.time()
                # Consume una imagen del buffer y la procesa
                imagen = self.video_buffer.get()
                imagen = redimensionar_imagen_porcentaje(imagen,self.optimizar)

                if(self.pose.set_pose(imagen)):
                    #self.indicador_estado_postura.configure(bootstyle=SUCCESS)
                    try:
                        self.imagen_negra = np.zeros((self.camara.alto*self.optimizar//100, self.camara.ancho*self.optimizar//100,3),dtype=np.uint8)
                        self.image_pose_buffer.put_nowait(
                            self.pose.get_draw_pose(self.imagen_negra)
                            )
                    except:
                        pass
                    
                    fin = time.time()
                    self.indicador_estado_postura.configure(text=str( 1//(fin-inicio) ))
                else:
                    self.indicador_estado_postura.configure(text="(P)")
                    #self.indicador_estado_postura.configure(bootstyle=DANGER)
                
            else:
                pass
                #self.indicador_estado_postura.configure(bootstyle=WARNING)
    

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

        self.indicador_estado_camara = ttk.Button(
            master= self.buttonbar, text='(C)',
            compound=LEFT,
            bootstyle=PRIMARY
        )
        self.indicador_estado_camara.pack(side=RIGHT, ipadx=5, ipady=5, padx=0, pady=1)

        self.indicador_estado_postura = ttk.Button(
            master= self.buttonbar, text='(P)',
            compound=LEFT, 
        )
        self.indicador_estado_postura.pack(side=RIGHT, ipadx=5, ipady=5, padx=0, pady=1)
        

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
        black_image = Image.new('RGB', size , color='black')
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

        ## Selector de disposito
        self.optimizacion = ttk.Combobox(
            master=self.postura_view,
            values=[str(i) for i in range(50,100,10)],
            bootstyle="light",
            state='readonly'
        )
        

        # Columna de Vista de Mediapipe
        self.optimizacion.grid(row=1, column=0, padx=10, pady=10, sticky='nsew')
        self.optimizacion.set(self.optimizar)
        self.optimizacion.bind('<<ComboboxSelected>>', self.cambio_optimizacion)
        
    def cambio_optimizacion(self,event):
        self.optimizar = int(self.optimizacion.get())

    def cambio_numero_camara(self,event):
        self.camara.activo = False
        selected_value = self.dispositivo_numero.get()
        self.camara.set_camara(int(selected_value))
        self.imagen_negra = np.zeros((self.camara.alto, self.camara.ancho, 3), dtype=np.uint8)
        #print(self.camara.captura())

    
    def actualizar_imagen_camara(self,imagen):
        imagen_formato = ImageTk.PhotoImage(Image.fromarray(imagen))
        self.image_camara.imgtk = imagen_formato
        self.image_camara.configure(image=imagen_formato)

    def actualizar_imagen_postura(self,imagen):
        imagen_formato = ImageTk.PhotoImage(Image.fromarray(imagen))
        self.imagen_postura.imgtk = imagen_formato
        self.imagen_postura.configure(image=imagen_formato)

    def evento_imagen_postura(self):
        if (self.pestana == 'Camara') and (self.camara.activo): # verificamos que la pestaña este activa para ahorra recursos
            self.actualizar_imagen_camara(self.camara.imagen)
            if not self.image_pose_buffer.empty():
                imagen = self.image_pose_buffer.get()
                self.actualizar_imagen_postura(imagen)
        
        if self.running:
            # Tiempo de espera de cada ejecucion de 10ms
            self.after(200, self.evento_imagen_postura)

    def evento_captura_imagen(self):
        self.camara.captura()
        if self.camara.activo:
            self.indicador_estado_camara.configure(bootstyle=SUCCESS)
            
            if not self.video_buffer.full():
                self.video_buffer.put_nowait(self.camara.imagen)
            else:
                self.indicador_estado_camara.configure(bootstyle=WARNING)
            
            #self.camara.set_redimencionar(size) #<-verificar en que lugar colocarlo
        else:
            self.indicador_estado_camara.configure(bootstyle=DANGER)

        if self.running:
            # Tiempo de espera de cada ejecucion de 10ms
            self.after(30, self.evento_captura_imagen)

    

    def create_main_content(self):
        """Configurar el contenido del frame principal."""
        self.camera_label = ttk.Label(self.main_frame)
        self.esqueleto_label = ttk.Label(self.main_frame)
        

        self.camera_label.grid(row=0, column=0, padx=10, pady=10)
        self.esqueleto_label.grid(row=0, column=1, padx=10, pady=10)
    '''
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
            self.video_buffer = frame

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
    '''
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
