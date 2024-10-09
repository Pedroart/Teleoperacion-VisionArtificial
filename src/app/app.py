import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import queue
import threading

from controller._pose import *
from controller._camara import *

class App(ttk.Window):

    from view.estructura.barra_control import barra_control_init
    from view.estructura.tabs import tabs_init
    from view.camara.video_player import video_player_init
    from view.camara.video_evento import procesar_video_buffer
    from view.camara.video_evento import actualizar_imagen_camara
    from view.camara.video_evento import actualizar_imagen_postura
    from view.camara.video_evento import evento_captura_imagen
    from view.camara.video_evento import evento_imagen_postura
    from view.camara.postura_player import postura_player_init

    running = False

    def __init__(self):
        super().__init__(themename="flatly")
        self.title("Luzia")
        self.geometry("1280x720")

        """Definiciones de Objetos"""
        # Crear la figura de Matplotlib para el gráfico 3D
        self.fig_3d = plt.figure(figsize=(5, 4))
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')

        self.camara = camara()
        self.pose = pose()

        self.video_buffer = queue.Queue(maxsize=5)
        self.image_pose_buffer = queue.Queue(maxsize=5)

        """Definiciones de Estructura"""
        self.barra_control_init()
        self.tabs_init()

        """Definicion de Vistas"""
        self.video_player_init()
        self.postura_player_init()


        """Eventos"""
        self.running = True
        self.evento_captura_imagen()
        self.hilo_demonio = threading.Thread(target=self.procesar_video_buffer, daemon=True)
        self.hilo_demonio.start()
        self.evento_imagen_postura()

    def on_closing(self):
        """Limpieza al cerrar la aplicación."""
        self.running = False
        #self.canvas_3d.get_tk_widget().destroy() <- ver donde se usa
        plt.close(self.fig_3d)
        self.camara.destroy()
        """Limpieza de Modulos"""

        self.destroy()