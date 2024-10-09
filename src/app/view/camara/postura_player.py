import tkinter as tk
import ttkbootstrap as ttk
from PIL import Image, ImageTk

def postura_player_init(self):
    # Crear el frame de la cámara
    self.postura_frame = ttk.Labelframe(self.postura_2D, text='Postura MediaPipe', padding=10)
    self.postura_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ns')

    self.image_postura = tk.Label(self.postura_frame, image=self.black_image_tk)
    self.image_postura.grid(row=0, column=0,columnspan=3, sticky='nsew')

        
