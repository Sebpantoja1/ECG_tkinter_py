import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
from collections import deque
from typing import Optional

from . import config
from .data_model import AppState, ECGDataModel
from .serial_handler import SerialReader
from .filters import ECGFilters
from .peak_detection import PeakDetector
from .algorithms.pan_tompkins import PanTompkinsDetector
from .cardioverter.state_machine import CardioStateMachine
from .recording.session_manager import SessionManager
from .recording.data_logger import MultiStreamRecorder
from .iu.cardioverter_panel import CardioverterPanel
from .iu.graphs_panel import ArduinoGraphsPanel
from .iu.recording_panel import RecordingPanel

class ECGMonitorApp(tk.Tk):
    def __init__(self, root=None):
        if root:
            super().__init__(root)
        else:
            super().__init__()
        self.title("ECG Monitor")
        self.geometry("1200x800")
        self.is_running = True

        # Existentes...
        self.app_state = AppState()
        self.ecg_filters = ECGFilters()
        self.serial_handler = SerialReader(self.app_state, self.ecg_filters)
        
        # Inicializar ECGDataModel con deques
        self.data_model = ECGDataModel(
            raw_data=deque(maxlen=3000),
            filtered_data=deque(maxlen=3000)
        )
        
        # NUEVOS componentes
        self.pt_detector = PanTompkinsDetector(config.SAMPLE_RATE)
        self.cardio_fsm = CardioStateMachine(self.serial_handler)
        self.session_manager = SessionManager()
        self.multi_recorder: Optional[MultiStreamRecorder] = None
        
        # Control de frecuencia Pan-Tompkins (evitar bloqueos)
        self.last_pt_time = 0.0
        self.pt_interval = 1.0  # Ejecutar Pan-Tompkins máximo cada 1 segundo
        self.pt_samples_processed = 0
        
        self.create_layout()
        self.serial_handler.start()
        self.start_update_loop()
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_layout(self):
        """
        Layout mejorado:
        +-----------------------------------+
        |  ECG Plots (2 gráficas)           |
        +-----------------------------------+
        | Arduino Graphs  | Cardio Panel    |
        |                 | Recording Panel |
        +-----------------------------------+
        | Controls & Status                 |
        +-----------------------------------+
        """
        # Notebook con tabs
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True)

        # Tab 1: ECG Monitoring
        ecg_tab = ttk.Frame(self.notebook)
        self.notebook.add(ecg_tab, text="ECG Monitor")
        
        # Frame principal para organizar gráficas y controles
        main_ecg_frame = ttk.Frame(ecg_tab)
        main_ecg_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Frame para gráficas (izquierda)
        plots_frame = ttk.Frame(main_ecg_frame)
        plots_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Frame para controles (derecha)
        controls_frame = ttk.Frame(main_ecg_frame, width=250)
        controls_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        controls_frame.pack_propagate(False)
        
        # Gráficas ECG
        self._create_ecg_plots(plots_frame)
        
        # Controles ECG
        self._create_ecg_controls(controls_frame)

        # Tab 2: Cardioverter Control
        cardio_tab = ttk.Frame(self.notebook)
        self.notebook.add(cardio_tab, text="Cardioverter")
        
        # Panel izquierdo: Cardioverter
        self.cardio_panel = CardioverterPanel(cardio_tab, self.cardio_fsm)
        self.cardio_panel.pack(side="left", fill="both", expand=True, padx=5, pady=5)
        
        # Panel derecho: Arduino Graphs
        self.arduino_graphs = ArduinoGraphsPanel(cardio_tab)
        self.arduino_graphs.pack(side="right", fill="both", expand=True, padx=5, pady=5)

        # Tab 3: Recording
        recording_tab = ttk.Frame(self.notebook)
        self.notebook.add(recording_tab, text="Recording")
        
        self.recording_panel = RecordingPanel(recording_tab, self.session_manager)
        self.recording_panel.pack(fill="both", expand=True, padx=5, pady=5)
        # Guardar referencia para que el panel pueda actualizar app.multi_recorder
        recording_tab.app_ref = self

    def _create_ecg_plots(self, parent):
        """Crear gráficas ECG"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._setup_plots()

    def _setup_plots(self):
        """Configurar gráficas"""
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

    def _create_ecg_controls(self, parent):
        """Crear controles ECG en sidebar"""
        # parent ya es el controls_frame, solo agregar los paneles
        self._create_control_panel(parent)
        self._create_gain_panel(parent)
        self._create_offset_panel(parent)
        self._create_plot_control_panel(parent)
        self._create_status_panel(parent)

    def _create_control_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Derivations Control", padding="10")
        panel.pack(fill=tk.X, pady=10)
        for i, label in self.app_state.mux_state_label.items():
            cmd = f"STATE_{i}"
            ttk.Button(panel, text=f"Manual: {label}", 
                       command=lambda c=cmd: self.serial_handler.send_command(c)).pack(fill='x', pady=2)
        ttk.Separator(panel, orient='horizontal').pack(fill='x', pady=5)
        ttk.Button(panel, text="Start Auto Mode",
                   command=lambda: self.serial_handler.send_command("START")).pack(fill='x', pady=2)
        ttk.Button(panel, text="Stop MUX",
                   command=lambda: self.serial_handler.send_command("STOP")).pack(fill='x', pady=2)

    def _create_gain_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Gain Control", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.gain_label = ttk.Label(panel, text=f"Gain: {self.app_state.ecg_gain.get():.2f}x")
        self.gain_label.pack()
        slider = ttk.Scale(panel, from_=0.1, to=5.0, orient=tk.HORIZONTAL, variable=self.app_state.ecg_gain, 
                          command=lambda v: self.gain_label.config(text=f"Gain: {float(v):.2f}x"))
        slider.pack(fill='x', pady=5)

    def _create_offset_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Offset Control", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.offset_label = ttk.Label(panel, text=f"Offset: {self.app_state.voltage_offset.get():.2f} V")
        self.offset_label.pack()
        slider = ttk.Scale(panel, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, variable=self.app_state.voltage_offset, 
                          command=lambda v: self.offset_label.config(text=f"Offset: {float(v):.2f} V"))
        slider.pack(fill='x', pady=5)

    def _create_plot_control_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="Plot Control", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.window_size_label = ttk.Label(panel, text=f"Window: {self.app_state.window_size.get()} samples")
        self.window_size_label.pack()
        ttk.Scale(panel, from_=500, to=3000, orient=tk.HORIZONTAL, variable=self.app_state.window_size, 
                  command=self.update_plot_settings).pack(fill='x', pady=5)
        self.y_max_label = ttk.Label(panel, text=f"Amplitude: {self.app_state.y_max.get():.2f} V")
        self.y_max_label.pack()
        ttk.Scale(panel, from_=0.5, to=5.0, orient=tk.HORIZONTAL, variable=self.app_state.y_max, 
                  command=self.update_plot_settings).pack(fill='x', pady=5)

    def _create_status_panel(self, parent):
        panel = ttk.LabelFrame(parent, text="System Status", padding="10")
        panel.pack(fill=tk.X, pady=10)
        self.status_labels = {}
        labels = ["ECG Connection", "Arduino", "Samples", "BPM", "Signal Quality", "Derivation"]
        for i, text in enumerate(labels):
            ttk.Label(panel, text=f"{text}:").grid(row=i, column=0, sticky="w", pady=2)
            self.status_labels[text] = ttk.Label(panel, text="N/A")
            self.status_labels[text].grid(row=i, column=1, sticky="w", pady=2)

    def update_plot_settings(self, _=None):
        self.ax1.set_ylim(0, self.app_state.y_max.get())
        self.ax2.set_ylim(0, self.app_state.y_max.get())
        self.y_max_label.config(text=f"Amplitude: {self.app_state.y_max.get():.2f} V")
        self.window_size_label.config(text=f"Window: {self.app_state.window_size.get()} samples")
        self.offset_label.config(text=f"Offset: {self.app_state.voltage_offset.get():.2f} V")

    def apply_filters(self, sample: float) -> float:
        """Aplicar filtros a una muestra"""
        return self.ecg_filters.process_sample(sample)

    def start_update_loop(self):
        """Iniciar loop de actualización"""
        self.update_loop()

    def update_loop(self):
        """Loop principal de actualización"""
        if not self.is_running:
            return
            
        # 1. Leer datos del ESP32
        new_samples = self.read_ecg_samples()
        
        # Obtener también los valores filtrados del buffer (ya procesados en serial_handler)
        new_filtered = []
        try:
            with self.app_state.data_lock:
                buffer_len = len(self.app_state.filtered_buffer)
                data_len = len(self.data_model.filtered_data)
                if buffer_len > data_len:
                    new_filtered = list(self.app_state.filtered_buffer)[data_len:]
        except Exception as e:
            print(f"Error leyendo datos filtrados: {e}")
        
        # Procesar muestras en batch (más eficiente)
        for i, sample in enumerate(new_samples):
            # La ganancia ya está aplicada en serial_handler
            self.data_model.raw_data.append(sample)
            
            # El valor filtrado ya está procesado en serial_handler
            if i < len(new_filtered):
                final_filtered = new_filtered[i]
            else:
                # Fallback: procesar manualmente si no está en buffer
                filtered = self.apply_filters(sample)
                final_filtered = filtered + self.app_state.voltage_offset.get()
            
            self.data_model.filtered_data.append(final_filtered)
            
            # Grabar si está activo
            if hasattr(self.session_manager, 'recording') and self.session_manager.recording and self.multi_recorder:
                timestamp = time.time()
                self.multi_recorder.record_ecg_sample(
                    timestamp, len(self.data_model.raw_data), 
                    sample, final_filtered
                )
        
        # 2. Ejecutar Pan-Tompkins con limitación de frecuencia (evitar bloqueos)
        current_time = time.time()
        if (len(self.data_model.filtered_data) >= 500 and 
            (current_time - self.last_pt_time) >= self.pt_interval):
            
            # Usar solo los últimos 500 puntos sin crear copia completa
            window_size = min(500, len(self.data_model.filtered_data))
            window = np.array(list(self.data_model.filtered_data)[-window_size:])
            
            # Ejecutar detección (puede tardar, pero limitado por frecuencia)
            try:
                pt_result = self.pt_detector.detect_qrs(window) if hasattr(self.pt_detector, 'detect_qrs') else self.pt_detector.detect(window)
                
                # Actualizar modelo solo si hay nuevos picos
                if pt_result.get('r_peaks'):
                    # Solo agregar picos nuevos (ajustar índices al buffer completo)
                    new_peaks = [p + len(self.data_model.filtered_data) - window_size 
                                for p in pt_result['r_peaks']]
                    self.data_model.r_peaks.extend(new_peaks)
                    self.data_model.rr_intervals = pt_result.get('rr_intervals', [])
                    self.data_model.hrv_metrics = pt_result.get('hrv_metrics', {})
                    
                    # Notificar cardioversor si detecta nuevo pico R
                    self.cardio_fsm.on_r_peak_detected()
                    
                    # Grabar
                    if hasattr(self.session_manager, 'recording') and self.session_manager.recording and self.multi_recorder:
                        self.multi_recorder.record_r_peak(time.time(), pt_result)
                
                self.last_pt_time = current_time
                self.pt_samples_processed += window_size
            except Exception as e:
                print(f"Error en Pan-Tompkins: {e}")
                # Continuar sin bloquear
        
        # 3. Leer datos del Arduino
        arduino_data = self.read_arduino_messages()
        for msg in arduino_data:
            parsed = self.serial_handler.parse_arduino_data(msg)
            if parsed:
                # Actualizar gráficas
                if parsed['type'] == 'armed':
                    self.arduino_graphs.update_vcap(parsed.get('vcap', 0))
                elif parsed['type'] in ['discharge_pos_end', 'discharge_neg_end']:
                    phase = 'pos' if 'pos' in parsed['type'] else 'neg'
                    self.arduino_graphs.update_energy(
                        phase, parsed.get('energy', 0), parsed.get('percent', 0)
                    )
                    
                # Grabar
                if hasattr(self.session_manager, 'recording') and self.session_manager.recording and self.multi_recorder:
                    self.multi_recorder.record_cardioverter_event(
                        time.time(), parsed
                    )
                    
                # Notificar FSM si descarga completa
                if parsed['type'] == 'discharge_complete':
                    self.cardio_fsm.on_discharge_complete()
        
        # 4. Actualizar FSM cardioversor
        self.cardio_fsm.update()
        
        # 5. Actualizar UI
        self.cardio_panel.update_ui()
        self.update_plots()
        self.update_status_labels()
        
        # Repetir cada 50 ms
        self.after(50, self.update_loop)

    def read_ecg_samples(self):
        """Lee nuevas muestras del ESP32 desde el buffer (sin ganancia/offset, ya aplicados en serial_handler)"""
        samples = []
        try:
            with self.app_state.data_lock:
                # Leer muestras nuevas desde el buffer
                # El buffer ya tiene ganancia aplicada desde serial_handler
                buffer_len = len(self.app_state.voltage_buffer)
                data_len = len(self.data_model.raw_data)
                
                if buffer_len > data_len:
                    # Obtener solo las muestras nuevas (eficiente con deque)
                    samples = list(self.app_state.voltage_buffer)[data_len:]
        except Exception as e:
            print(f"Error leyendo muestras ECG: {e}")
        return samples

    def read_arduino_messages(self):
        """Lee mensajes del Arduino (implementación simplificada)"""
        # En una implementación real, esto leería desde un buffer de mensajes
        # Por ahora retornamos lista vacía, los mensajes se procesan en el thread
        return []

    def update_plots(self):
        """Actualizar gráficas ECG (optimizado para tiempo real)"""
        if len(self.data_model.raw_data) == 0:
            return
            
        try:
            win_size = self.app_state.window_size.get()
            data_len = len(self.data_model.raw_data)
            
            # Calcular slice sin crear copias completas
            start_idx = max(0, data_len - win_size)
            
            # Usar slicing directo de deque (más eficiente)
            y_raw = list(self.data_model.raw_data)[start_idx:]
            y_filtered = list(self.data_model.filtered_data)[start_idx:]
            x_data = list(range(start_idx, data_len))
            
            if not x_data or len(y_raw) == 0:
                return
            
            # Actualizar datos de líneas (rápido)
            self.line_raw.set_data(x_data, y_raw)
            self.line_filtered.set_data(x_data, y_filtered)
            
            # Mostrar picos R detectados (optimizado)
            if self.data_model.r_peaks and len(self.data_model.r_peaks) > 0:
                # Solo buscar picos visibles (más eficiente)
                visible_peaks = [p for p in self.data_model.r_peaks[-20:] if start_idx <= p < data_len]
                if visible_peaks:
                    # Calcular índices relativos al slice
                    peak_indices = [p - start_idx for p in visible_peaks]
                    peak_values = [y_filtered[i] if i < len(y_filtered) else 0 for i in peak_indices]
                    peak_x = [x_data[i] if i < len(x_data) else start_idx for i in peak_indices]
                    self.peaks_line.set_data(peak_x, peak_values)
                else:
                    self.peaks_line.set_data([], [])
            else:
                self.peaks_line.set_data([], [])
            
            # Actualizar límites de ejes
            x_min = x_data[0] if x_data else start_idx
            x_max = x_data[-1] if x_data else (start_idx + win_size)
            self.ax1.set_xlim(x_min, x_max)
            
            # Dibujar canvas (esto puede ser costoso, pero necesario)
            self.canvas.draw_idle()  # Usar draw_idle en lugar de draw para mejor performance
        except Exception as e:
            print(f"Error actualizando gráficas: {e}")
            # Continuar sin bloquear

    def update_status_labels(self):
        """Actualizar etiquetas de estado"""
        with self.app_state.data_lock:
            y_raw = list(self.app_state.voltage_buffer)
            y_filtered = list(self.app_state.filtered_buffer)
        
        self.status_labels["Arduino"].config(
            text=f"{'🟢 Connected' if self.app_state.arduino_connected else '🔴 Disconnected'}"
        )
        self.status_labels["Samples"].config(text=f"{len(self.data_model.raw_data)}")
        
        # BPM desde HRV metrics
        if self.data_model.hrv_metrics and 'hr' in self.data_model.hrv_metrics:
            hr = self.data_model.hrv_metrics.get('hr', 0)
            self.status_labels["BPM"].config(text=f"{hr:.0f} BPM" if hr > 0 else "Calculating...")
        else:
            self.status_labels["BPM"].config(text="Calculating...")
        
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

    def on_closing(self):
        self.is_running = False
        print("Closing application...")
        self.serial_handler.stop()
        if self.multi_recorder:
            self.multi_recorder.stop_recording()
        self.destroy()

# Mantener compatibilidad con código existente
ECGApp = ECGMonitorApp
