"""
This module defines the AppState class, which holds the shared state of the application.
"""
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import List, Optional
import tkinter as tk

from . import config

class AppState:
    """A class to hold the shared state of the application."""
    def __init__(self):
        # --- Data Buffers ---
        self.data_lock = threading.Lock()
        self.voltage_buffer = deque(maxlen=3000)
        self.filtered_buffer = deque(maxlen=3000)
        self.time_buffer = deque(maxlen=3000)
        self.sample_count = 0

        # --- Connection Status ---
        self.serial_connected = False
        self.arduino_connected = False

        # --- MUX Control ---
        self.mux_control_lock = threading.Lock()
        self.mux_state_label = {
            0: "I DERIVADA", 
            1: "II DERIVADA", 
            2: "III DERIVADA", 
            3: "aVR"
        }
        self.current_mux_state = 0
        


        # --- UI-bound variables that affect data processing ---
        self.ecg_gain = tk.DoubleVar(value=1.0)
        self.voltage_offset = tk.DoubleVar(value=0.0)
        self.r_threshold = tk.DoubleVar(value=config.R_THRESHOLD_DEFAULT)
        self.r_distance = tk.IntVar(value=config.R_DISTANCE_DEFAULT)

        # --- UI-bound variables for display only ---
        self.window_size = tk.IntVar(value=1500)
        self.y_max = tk.DoubleVar(value=3.5)


@dataclass
class ECGDataModel:
    # Existentes...
    raw_data: deque
    filtered_data: deque
    
    # NUEVOS campos Pan-Tompkins
    r_peaks: List[int] = field(default_factory=list)
    rr_intervals: List[float] = field(default_factory=list)
    hr_instantaneous: List[float] = field(default_factory=list)
    hrv_metrics: dict = field(default_factory=dict)
    
    # Para cardioversor
    last_r_peak_time: Optional[float] = None
