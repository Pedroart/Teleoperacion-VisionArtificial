import tkinter as tk
import ttkbootstrap as ttk
from PIL import Image, ImageTk

def video_player_init(self):
    # Crear el frame de la cámara
    self.camara_frame = ttk.Labelframe(self.camara_view, text='Vista de Cámara', padding=10)
    self.camara_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ns')

    # Imagen del recuadro
    self.size = (64 * 9, 48 * 9)
    self.black_image = Image.new('RGB', self.size, color='black')
    self.black_image_tk = ImageTk.PhotoImage(image=self.black_image)

    self.image_camara = tk.Label(self.camara_frame, image=self.black_image_tk)
    self.image_camara.grid(row=0, column=0,columnspan=3, sticky='nsew')

    # Opciones de dispositivos
    opciones_dispositivos = ['/home/art23/dev_ws/src/Teleoperacion-VisionArtificial/src/app/test.mp4'] + [str(i) for i in range(11)]
    
    # Selector de dispositivo
    self.dispositivo_numero = ttk.Combobox(
        master=self.camara_frame,
        values=opciones_dispositivos,
        bootstyle="light",
        state='readonly'
    )
    self.dispositivo_numero.set(opciones_dispositivos[0])
    self.dispositivo_numero.grid(row=1, column=0, padx=10, pady=10, sticky='ew')

    # Botón de Play
    self.btn_play = ttk.Button(self.camara_frame, text="Play", command=
                               lambda: setattr(self.camara, 'activo', True))
    self.btn_play.grid(row=1, column=1, padx=10, pady=10, sticky='ew')

    # Botón de Stop
    self.btn_stop = ttk.Button(self.camara_frame, text="Stop", command=
                               lambda: setattr(self.camara, 'activo', False))
    self.btn_stop.grid(row=1, column=2, padx=10, pady=10, sticky='ew')

    # Ajustar columnas para que se expandan uniformemente
    self.camara_frame.grid_columnconfigure(0, weight=1)
    self.camara_frame.grid_columnconfigure(1, weight=1)
    self.camara_frame.grid_columnconfigure(2, weight=1)

    self.dispositivo_numero.bind(
                                    '<<ComboboxSelected>>', 
                                    lambda event: self.camara.set_camara(int(self.dispositivo_numero.get())) if len(self.dispositivo_numero.get()) < 2 else self.camara.set_camara(self.dispositivo_numero.get())
                                )
                                 
        
        
