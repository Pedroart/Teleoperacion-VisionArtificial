import tkinter as tk
import ttkbootstrap as ttk
from PIL import Image, ImageTk

def init_video_player(self):
        # Columna de Vista de camaras
        self.camara_view = ttk.Labelframe(self.camara_frame,text='Vista de Camara', padding=10)
        self.camara_view.grid(row=0, column=0, padx=10, pady=10,sticky='ns')
        
        ## Imagen del recuadro
        black_image = Image.new('RGB', self.size , color='black')
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
        
