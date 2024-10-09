import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap.dialogs import Messagebox

'''
TO-DO
    REALIZAR UN OPTIMIZACION DE VARIABLES
    ACCIONES DE BOTONES
'''

'''
Variables importantes de accion:
    self.indicador_estado_camara
    self.indicador_estado_postura
'''


def barra_control_init(self):
    # Crear Barra de Control
        self.buttonbar = ttk.Frame(self, style=PRIMARY)
        self.buttonbar.pack(fill=X, pady=1, side=TOP)
        create_buttonbar(self)

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
