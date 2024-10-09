import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *

def tabs_init(self):
    # Crear el notebook (conjunto de pestañas)

    # Pestaña Izquieda
    self.notebook_izq = ttk.Notebook(self)
    self.notebook_izq.pack(side='left',fill='both', expand=True, padx=5, pady=5)
    # Pestaña Izquieda
    self.notebook_der = ttk.Notebook(self)
    self.notebook_der.pack(side='right',fill='both', expand=True, padx=5, pady=5)

    create_tabs(self)

    self.pestana = [
            self.notebook_izq.tab(0, "text"),
            self.notebook_der.tab(0, "text"),
        ]
    
    self.notebook_izq.bind("<<NotebookTabChanged>>", 
                               lambda event: self.pestana.__setitem__(0, self.notebook_izq.tab(self.notebook_izq.index(self.notebook_izq.select()), "text"))
                        )
    self.notebook_der.bind("<<NotebookTabChanged>>", 
                               lambda event: self.pestana.__setitem__(1, self.notebook_der.tab(self.notebook_der.index(self.notebook_der.select()), "text"))
                        )
    

def create_tabs(self):
    '''Izquierda'''
    self.postura_2D = ttk.Frame(self.notebook_izq, bootstyle=None)
    self.notebook_izq.add(self.postura_2D, text='2D')

    self.postura_3D = ttk.Frame(self.notebook_izq, bootstyle=None)
    self.notebook_izq.add(self.postura_3D, text='3D')
    
    '''Derecha'''
    self.angulos_view = ttk.Frame(self.notebook_der, bootstyle=None)
    self.notebook_der.add(self.angulos_view, text='Angulos')

    self.logs_view = ttk.Frame(self.notebook_der, bootstyle=None)
    self.notebook_der.add(self.logs_view, text='Logs')
    self.camara_view = ttk.Frame(self.notebook_der, bootstyle=None)
    self.notebook_der.add(self.camara_view, text='Camara')
    self.pose_config = ttk.Frame(self.notebook_der, bootstyle=None)
    self.notebook_der.add(self.pose_config, text='Config')
    

