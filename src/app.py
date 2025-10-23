import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

from . import config
from .data_model import AppState
from .serial_handler import SerialReader
from .filters import ECGFilters
from .peak_detection import detect_r_peaks, calculate_bpm

class ECGApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ECG Monitor")
        self.geometry("1200x800")
        self.is_running = True

        # Initialize components
        self.app_state = AppState()
        self.ecg_filters = ECGFilters()
        self.serial_reader = SerialReader(self.app_state, self.ecg_filters)

        self._create_widgets()
        self.serial_reader.start()
        self.update_gui()
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        sidebar_frame = ttk.Frame(main_frame, width=250)
        sidebar_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        sidebar_frame.pack_propagate(False)

        self._create_plots(plot_frame)
        self._create_control_panel(sidebar_frame)
        self._create_gain_panel(sidebar_frame)
        self._create_plot_control_panel(sidebar_frame)
        self._create_peak_panel(sidebar_frame)
        self._create_status_panel(sidebar_frame)

    def _create_plots(self, parent):
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._setup_plots()

    def _setup_plots(self):
        self.ax1.set_ylim(0, self.app_state.y_max.get())
        self.ax1.set_ylabel('Original Voltage (V)')
        self.ax1.set_title('ECG - Original Signal')
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_ylim(0, self.app_state.y_max.get())
        self.ax2.set_xlabel('Samples')
        self.ax2.set_ylabel('Filtered Voltage (V)')
        self.ax2.set_title(f'Filtered Signal - {"ON" if config.ENABLE_FILTERS else "OFF"}')
        self.ax2.grid(True, alpha=0.3)
        
        self.line_raw, = self.ax1.plot([], [], 'b-', linewidth=1.2, alpha=0.8)
        self.line_filtered, = self.ax2.plot([], [], 'g-', linewidth=1.5)
        self.peaks_line, = self.ax2.plot([], [], 'ro', markersize=5, alpha=0.8)
        
        self.fig.tight_layout()

    def _create_control_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Derivations Control", padding="10")
        panel.pack(fill=tk.X, pady=10)
        for i, label in self.app_state.mux_state_label.items():
            cmd = f"STATE_{i}"
            ttk.Button(panel, text=f"Manual: {label}", 
                       command=lambda c=cmd: self.serial_reader.send_command(c)).pack(fill='x', pady=2)
        ttk.Separator(panel, orient='horizontal').pack(fill='x', pady=5)
        ttk.Button(panel, text="Start Auto Mode",
                   command=lambda: self.serial_reader.send_command("START")).pack(fill='x', pady=2)
        ttk.Button(panel, text="Stop MUX",
                   command=lambda: self.serial_reader.send_command("STOP")).pack(fill='x', pady=2)

    def _create_gain_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Gain Control", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.gain_label = ttk.Label(panel, text=f"Gain: {self.app_state.ecg_gain.get():.2f}x")
        self.gain_label.pack()
        slider = ttk.Scale(panel, from_=0.1, to=5.0, orient=tk.HORIZONTAL, variable=self.app_state.ecg_gain, command=lambda v: self.gain_label.config(text=f"Gain: {float(v):.2f}x"))
        slider.pack(fill='x', pady=5)
    
    def _create_plot_control_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Plot Control", padding="10")
        panel.pack(fill=tk.X, pady=10)

        self.window_size_label = ttk.Label(panel, text=f"Window: {self.app_state.window_size.get()} samples")
        self.window_size_label.pack()
        ttk.Scale(panel, from_=500, to=3000, orient=tk.HORIZONTAL, variable=self.app_state.window_size, command=self.update_plot_settings).pack(fill='x', pady=5)

        self.y_max_label = ttk.Label(panel, text=f"Amplitude: {self.app_state.y_max.get():.2f} V")
        self.y_max_label.pack()
        ttk.Scale(panel, from_=0.5, to=5.0, orient=tk.HORIZONTAL, variable=self.app_state.y_max, command=self.update_plot_settings).pack(fill='x', pady=5)

        ttk.Separator(panel, orient='horizontal').pack(fill='x', pady=5)

        self.voltage_offset_label = ttk.Label(panel, text=f"Offset: {self.app_state.voltage_offset.get():.2f} V")
        self.voltage_offset_label.pack()
        ttk.Scale(panel, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, variable=self.app_state.voltage_offset, command=self.update_plot_settings).pack(fill='x', pady=5)

    def _create_peak_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Peak Detection", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.r_threshold_label = ttk.Label(panel, text=f"Threshold (V): {self.app_state.r_threshold.get():.2f}")
        self.r_threshold_label.pack()
        ttk.Scale(panel, from_=0.1, to=3.0, orient=tk.HORIZONTAL, variable=self.app_state.r_threshold, command=self.update_peak_settings).pack(fill='x', pady=5)
        self.r_distance_label = ttk.Label(panel, text=f"Distance (ms): {self.app_state.r_distance.get() * 1000 / config.SAMPLE_RATE:.0f}")
        self.r_distance_label.pack()
        ttk.Scale(panel, from_=20, to=1000, orient=tk.HORIZONTAL, variable=self.app_state.r_distance, command=self.update_peak_settings).pack(fill='x', pady=5)

    def _create_status_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="System Status", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.status_labels = {}
        labels = ["ECG Connection", "Trigger Port", "Samples", "BPM", "Signal Quality", "Derivation", "Electrodes"]
        for i, text in enumerate(labels):
            ttk.Label(panel, text=f"{text}:").grid(row=i, column=0, sticky="w", pady=2)
            self.status_labels[text] = ttk.Label(panel, text="N/A")
            self.status_labels[text].grid(row=i, column=1, sticky="w", pady=2)

    def update_plot_settings(self, _=None):
        self.ax1.set_ylim(0, self.app_state.y_max.get())
        self.ax2.set_ylim(0, self.app_state.y_max.get())
        self.y_max_label.config(text=f"Amplitude: {self.app_state.y_max.get():.2f} V")
        self.window_size_label.config(text=f"Window: {self.app_state.window_size.get()} samples")
        self.voltage_offset_label.config(text=f"Offset: {self.app_state.voltage_offset.get():.2f} V")

    def update_peak_settings(self, _=None):
        self.r_threshold_label.config(text=f"Threshold (V): {self.app_state.r_threshold.get():.2f}")
        distance_ms = self.app_state.r_distance.get() * 1000 / config.SAMPLE_RATE
        self.r_distance_label.config(text=f"Distance (ms): {distance_ms:.0f}")

    def update_gui(self):
        if not self.is_running: return

        with self.app_state.data_lock:
            y_raw = list(self.app_state.voltage_buffer)
            y_filtered = list(self.app_state.filtered_buffer)
            x_data = list(self.app_state.time_buffer)
        
        if y_raw:
            self.update_plots(x_data, y_raw, y_filtered)
        
        self.update_status_labels(y_raw, y_filtered)
        self.after(config.REFRESH_INTERVAL, self.update_gui)

    def update_plots(self, x_data, y_raw, y_filtered):
        win_size = self.app_state.window_size.get()
        visible_slice = slice(-win_size, None)
        y_raw_vis = y_raw[visible_slice]
        y_filtered_vis = y_filtered[visible_slice]
        x_vis = x_data[visible_slice]

        if not x_vis: return

        self.line_raw.set_data(x_vis, y_raw_vis)
        self.line_filtered.set_data(x_vis, y_filtered_vis)

        signal_for_peaks = y_filtered_vis if config.ENABLE_FILTERS else y_raw_vis
        if len(signal_for_peaks) > 10:
            peaks_idx = detect_r_peaks(signal_for_peaks, self.app_state.r_threshold.get(), self.app_state.r_distance.get())
            if peaks_idx:
                self.peaks_line.set_data([x_vis[i] for i in peaks_idx], [signal_for_peaks[i] for i in peaks_idx])
            else:
                self.peaks_line.set_data([], [])
        
        x_min = x_vis[0]
        x_max = x_vis[-1] if len(x_vis) >= win_size else x_vis[0] + win_size
        self.ax1.set_xlim(x_min, x_max)
        self.canvas.draw()

    def update_status_labels(self, y_raw, y_filtered):
        # Connection status
        self.status_labels["ECG Connection"].config(text=f"{'🟢 Connected' if self.app_state.serial_connected else '🔴 Disconnected'}")
        self.status_labels["Trigger Port"].config(text=f"{'🟢 Connected' if self.app_state.trigger_port_connected else '🔴 Disconnected'}")
        self.status_labels["Samples"].config(text=f"{self.app_state.sample_count}")

        # BPM
        win_size = self.app_state.window_size.get()
        signal_for_peaks = y_filtered[-win_size:] if config.ENABLE_FILTERS else y_raw[-win_size:]
        peaks = detect_r_peaks(signal_for_peaks, self.app_state.r_threshold.get(), self.app_state.r_distance.get())
        bpm = calculate_bpm(peaks, config.SAMPLE_RATE)
        self.status_labels["BPM"].config(text=f"{bpm:.0f} BPM" if bpm > 0 else "Calculating...")
        
        # Signal Quality
        if len(y_raw) >= 100:
            noise = np.std(y_raw[-100:])
            quality = "🟢 GOOD" if noise < 0.05 else "🟡 FAIR" if noise < 0.1 else "🔴 POOR"
            self.status_labels["Signal Quality"].config(text=f"{quality} ({noise:.3f}V)")
        else:
            self.status_labels["Signal Quality"].config(text="N/A")

        # MUX state
        with self.app_state.mux_control_lock:
            state = self.app_state.current_mux_state
            if state >= 0:
                mux_text = self.app_state.mux_state_label.get(state, "Unknown")
            elif state == -1:
                mux_text = "Auto"
            else:
                mux_text = "Stopped"
        self.status_labels["Derivation"].config(text=mux_text)
        
        # Electrodes status
        with self.app_state.data_lock:
            status_str = " ".join(["🟢" if s == 0 else "🔴" for s in self.app_state.loose_lead_status])
        self.status_labels["Electrodes"].config(text=status_str)

    def on_closing(self):
        self.is_running = False
        print("Closing application...")
        self.serial_reader.stop()
        self.destroy()
