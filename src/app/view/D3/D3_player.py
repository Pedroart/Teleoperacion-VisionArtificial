import tkinter as tk
import ttkbootstrap as ttk
from ttkbootstrap.constants import *
from ttkbootstrap.dialogs import Messagebox

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


def D3_init(self):
    
    self.postura_frame = ttk.Labelframe(self.postura_3D, text='3D Vista', padding=10)
    self.postura_frame.grid(row=0, column=0, padx=10, pady=10, sticky='ns')

    # Agregar la figura de Matplotlib al Tkinter Canvas para 3D
    self.canvas_3d = FigureCanvasTkAgg(self.fig_3d, self.postura_frame)
    self.canvas_3d.get_tk_widget().grid(row=0, column=0, padx=10, pady=10,sticky='ns')

    