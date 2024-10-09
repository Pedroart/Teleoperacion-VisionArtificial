from ttkbootstrap.constants import *
import cv2
from PIL import Image, ImageTk
import numpy as np
import time

def procesar_video_buffer(self):
        # Este método se ejecutará en un hilo demonio
        while self.running:
            if not self.video_buffer.empty():
                inicio = time.time()
                # Consume una imagen del buffer y la procesa
                imagen = self.video_buffer.get()
                #imagen = redimensionar_imagen_porcentaje(imagen,self.optimizar)

                if(self.pose.set_pose(imagen)):
                    #self.imagen_negra = np.zeros((self.camara.alto*self.optimizar//100, self.camara.ancho*self.optimizar//100,3),dtype=np.uint8)
                    
                    if(self.pose.son_puntos_visibles()):
                        self.pose.normalizacion()
                        print("Angulos:",self.pose.get_angulos())

                        self.indicador_estado_postura.configure(bootstyle=SUCCESS)
                        self.pose.plot_world_landmarks(self.ax_3d)
                        self.canvas_3d.draw()
                    else:
                        self.indicador_estado_postura.configure(bootstyle=WARNING)

                    fin = time.time()
                    self.indicador_estado_postura.configure(text=str( 1//(fin-inicio) ))
                else:
                    self.indicador_estado_postura.configure(text="(P)")
                    self.indicador_estado_postura.configure(bootstyle=DANGER)
                
            else:
                pass
                #self.indicador_estado_postura.configure(bootstyle=WARNING)

def evento_captura_imagen(self):
        if self.camara.activo:
            self.camara.captura()
            self.indicador_estado_camara.configure(bootstyle=SUCCESS)
            self.camara.set_redimencionar(self.size) #<-verificar en que lugar colocarlo
            
            if not self.video_buffer.full():
                self.video_buffer.put_nowait(self.camara.imagen)
            else:
                self.indicador_estado_camara.configure(bootstyle=WARNING)
            
        else:
            self.indicador_estado_camara.configure(bootstyle=DANGER)

        if self.running:
            # Tiempo de espera de cada ejecucion de 10ms
            self.after(30, self.evento_captura_imagen)

def evento_imagen_postura(self):
        if (self.camara.activo):
            if ('Camara' in self.pestana) : # verificamos que la pestaña este activa para ahorra recursos
                self.actualizar_imagen_camara(self.camara.imagen)
            if ('2D' in self.pestana) : # verificamos que la pestaña este activa para ahorra recursos
                self.postura_puntos = np.zeros((self.size[1], self.size[0], 3), dtype=np.uint8) 
                self.actualizar_imagen_postura(self.pose.get_draw_pose(self.postura_puntos))
                pass

        if self.running:
            # Tiempo de espera de cada ejecucion de 10ms
            self.after(200, self.evento_imagen_postura)

def actualizar_imagen_camara(self,imagen):
        imagen_formato = ImageTk.PhotoImage(Image.fromarray(imagen))
        self.image_camara.imgtk = imagen_formato
        self.image_camara.configure(image=imagen_formato)

def actualizar_imagen_postura(self,imagen):
        imagen_formato = ImageTk.PhotoImage(Image.fromarray(imagen))
        self.image_postura.imgtk = imagen_formato
        self.image_postura.configure(image=imagen_formato)