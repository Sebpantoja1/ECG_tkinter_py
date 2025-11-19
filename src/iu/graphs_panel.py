import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque

class ArduinoGraphsPanel(ttk.Frame):
    """Gráficas de energía y voltaje del capacitor"""
    
    def __init__(self, parent):
        super().__init__(parent)
        
        # Buffers de datos
        self.vcap_data = deque(maxlen=1000)
        self.energy_pos_data = deque(maxlen=100)
        self.energy_neg_data = deque(maxlen=100)
        
        self.create_plots()
        
    def create_plots(self):
        # Canvas 1: Voltaje capacitor durante carga
        fig1, ax1 = plt.subplots(figsize=(6, 3))
        self.vcap_ax = ax1
        ax1.set_title("Capacitor Voltage (Charging)")
        ax1.set_xlabel("Time (ms)")
        ax1.set_ylabel("Vcap (V)")
        
        canvas1 = FigureCanvasTkAgg(fig1, self)
        canvas1.get_tk_widget().pack()
        
        # Canvas 2: Energía entregada por fase
        fig2, (ax2, ax3) = plt.subplots(1, 2, figsize=(10, 3))
        self.energy_pos_ax = ax2
        self.energy_neg_ax = ax3
        
        ax2.set_title("Energy Phase +")
        ax2.set_ylabel("Energy (J)")
        ax3.set_title("Energy Phase -")
        
        canvas2 = FigureCanvasTkAgg(fig2, self)
        canvas2.get_tk_widget().pack()
        
    def update_vcap(self, voltage: float):
        """Actualizar gráfica de Vcap"""
        self.vcap_data.append(voltage)
        self.vcap_ax.clear()
        self.vcap_ax.plot(list(self.vcap_data))
        self.vcap_ax.figure.canvas.draw()
        
    def update_energy(self, phase: str, energy: float, percent: float):
        """Actualizar gráficas de energía entregada"""
        if phase == "pos":
            self.energy_pos_data.append((energy, percent))
        else:
            self.energy_neg_data.append((energy, percent))
        # Redraw...
