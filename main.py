import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import threading
from collections import deque
from scipy import signal
import tkinter as tk
from tkinter import ttk

# --- Configuración del puerto serial ---
SERIAL_PORT = "COM8"  # Cambiar según tu PC
BAUD_RATE = 115200

# --- Configuración del puerto de disparo (trigger) ---
TRIGGER_SERIAL_PORT = "COM9"  # Puerto para enviar la señal de disparo
TRIGGER_BAUD_RATE = 9600
TRIGGER_SIGNAL = b'\x01'  # Byte a enviar cuando se detecta un pico R

# --- MODO DEBUG ---
DEBUG_MODE = True  # Cambiar a False para modo normal
RAW_DATA_MODE = True  # Mostrar datos en crudo sin filtrar

# --- Configuración de filtros ---
SAMPLE_RATE = 2000  # Hz (ajustar según tu ESP32)
ENABLE_FILTERS = True  # Activar/desactivar filtros

# --- Configuración de la gráfica ---
refresh_interval = 50  # ms (velocidad de refresco)

# --- Parámetros de detección de picos ---
R_THRESHOLD_DEFAULT = 1.0 # Umbral para picos R
R_DISTANCE_DEFAULT = 200  # muestras mínimas entre picos (ej: 200ms a 1000Hz)

class ECGFilters:
    def __init__(self, app, fs=SAMPLE_RATE):
        self.app = app
        self.fs = fs
        self.setup_filters()
        self.reset_states()
        
    def setup_filters(self):
        nyquist = self.fs / 2
        self.notch_b, self.notch_a = signal.iirnotch(60, 20, self.fs)
        low_cutoff = 0.05 / nyquist
        high_cutoff = 40 / nyquist
        self.bp_b, self.bp_a = signal.butter(2, [low_cutoff, high_cutoff], btype='band')
    
    def reset_states(self):
        self.notch_zi = signal.lfilter_zi(self.notch_b, self.notch_a)
        self.bp_zi = signal.lfilter_zi(self.bp_b, self.bp_a)
        initial_condition = 1.65
        self.notch_zi *= initial_condition
        self.bp_zi *= initial_condition
    
    def process_sample(self, sample):
        if not ENABLE_FILTERS:
            return sample
        filtered_sample, self.notch_zi = signal.lfilter(self.notch_b, self.notch_a, [sample], zi=self.notch_zi)
        filtered_sample, self.bp_zi = signal.lfilter(self.bp_b, self.bp_a, filtered_sample, zi=self.bp_zi)
        filtered_value = filtered_sample[0] + self.app.voltage_offset.get()
        if filtered_value < 0:
            filtered_value = max(filtered_value, -0.5)
        elif filtered_value > 3.3:
            filtered_value = min(filtered_value, 3.8)
        return filtered_value

class SerialReader:
    def __init__(self, port, baud_rate, app_instance):
        self.port = port
        self.baud_rate = baud_rate
        self.app = app_instance
        self.ser = None
        self.trigger_ser = None # Nuevo puerto serial para el disparo
        self.running = False
        self.total_bytes_received = 0
        self.valid_packets = 0
        self.invalid_packets = 0
        self.sync_buffer = bytearray()
        
        # Buffer para detección de picos en tiempo real
        self.realtime_peak_buffer = deque(maxlen=R_DISTANCE_DEFAULT * 2)
        self.last_peak_sample_count = 0
    def connect(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=0.1)
            time.sleep(2)
            self.ser.flushInput()
            self.ser.flushOutput()
            print("Conexión establecida")
            return True
        except Exception as e:
            print(f"Error al conectar: {e}")
            return False

    def connect_trigger(self):
        """Intenta conectar al puerto serial de disparo."""
        try:
            if self.trigger_ser and self.trigger_ser.is_open:
                self.trigger_ser.close()
            self.trigger_ser = serial.Serial(TRIGGER_SERIAL_PORT, TRIGGER_BAUD_RATE, timeout=0.1)
            print(f"Puerto de disparo {TRIGGER_SERIAL_PORT} conectado.")
            return True
        except Exception as e:
            print(f"Error al conectar puerto de disparo {TRIGGER_SERIAL_PORT}: {e}")
            return False
    
    def decode_packet(self, pkt):
        if len(pkt) != 4:
            self.invalid_packets += 1
            return None
        start, lsb, msb, checksum = pkt
        if start != 0xAA:
            self.invalid_packets += 1
            return None
        expected_checksum = start ^ lsb ^ msb
        if checksum != expected_checksum:
            self.invalid_packets += 1
            return None
        val = (msb << 8) | lsb
        voltage = val * (3.3 / 4095)
        self.valid_packets += 1
        return voltage

    def check_for_r_peak_and_trigger(self, new_sample):
        """Revisa si una nueva muestra completa un pico R y envía una señal."""
        self.realtime_peak_buffer.append(new_sample)
        
        # Necesitamos al menos 3 puntos para detectar un pico (antes, pico, después)
        if len(self.realtime_peak_buffer) < 3:
            return

        # El candidato a pico es el punto del medio en una ventana de 3
        mid_idx = len(self.realtime_peak_buffer) - 2
        p_before, p_peak, p_after = self.realtime_peak_buffer[mid_idx-1], self.realtime_peak_buffer[mid_idx], self.realtime_peak_buffer[mid_idx+1]

        # Obtener valores de la GUI
        threshold = self.app.r_threshold.get()
        distance = self.app.r_distance.get()

        # Lógica de detección de pico simple
        if p_peak > threshold and p_peak > p_before and p_peak > p_after:
            if (self.app.sample_count - self.last_peak_sample_count) >= distance:
                if self.trigger_ser and self.trigger_ser.is_open:
                    self.trigger_ser.write(TRIGGER_SIGNAL)
                self.last_peak_sample_count = self.app.sample_count
    
    def read_data(self):
        print("Iniciando lectura de datos...")
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    self.app.serial_connected = False
                    if not self.connect():
                        time.sleep(1)
                        continue
                self.app.serial_connected = True

                if not self.trigger_ser or not self.trigger_ser.is_open:
                    self.app.trigger_port_connected = self.connect_trigger()
                else:
                    self.app.trigger_port_connected = True
                
                if self.ser.in_waiting > 0:
                    raw_bytes = self.ser.read(self.ser.in_waiting)
                    
                    try:
                        message = raw_bytes.decode('utf-8').strip()
                        if message.startswith("LEADS_STATE:"):
                            state_str = message.split(':')[1]
                            if len(state_str) == 4:
                                with self.app.data_lock:
                                    self.app.loose_lead_status = [int(s) for s in state_str]
                                continue
                    except (UnicodeDecodeError, IndexError, ValueError):
                        pass

                    self.sync_buffer.extend(raw_bytes)
                    
                    while len(self.sync_buffer) >= 4:
                        start_idx = self.sync_buffer.find(b'\xaa')
                        if start_idx == -1:
                            del self.sync_buffer[:]
                            break
                        
                        if start_idx > 0:
                            del self.sync_buffer[:start_idx]

                        if len(self.sync_buffer) >= 4:
                            packet = self.sync_buffer[:4]
                            voltage = self.decode_packet(packet)
                            
                            if voltage is not None:
                                voltage *= self.app.ecg_gain.get()
                                filtered_voltage = self.app.ecg_filters.process_sample(voltage)
                                self.check_for_r_peak_and_trigger(filtered_voltage) # Detección en tiempo real
                                with self.app.data_lock:
                                    self.app.voltage_buffer.append(voltage)
                                    self.app.filtered_buffer.append(filtered_voltage)
                                    self.app.time_buffer.append(self.app.sample_count)
                                    self.app.sample_count += 1
                            
                            del self.sync_buffer[:4]
                        else:
                            break
                time.sleep(0.005)
            except Exception as e:
                print(f"Error en lectura serial: {e}")
                self.app.serial_connected = False
                self.app.trigger_port_connected = False
                time.sleep(1)
    
    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.read_data, daemon=True)
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Puerto serial cerrado.")
        if self.trigger_ser and self.trigger_ser.is_open:
            self.trigger_ser.close()
            print("Puerto de disparo cerrado.")

    def send_command(self, command):
        if not self.ser or not self.ser.is_open:
            print("Error: No hay conexión serial para enviar el comando.")
            return
        with self.app.mux_control_lock:
            try:
                cmd = (command + "\n").encode('utf-8')
                self.ser.write(cmd)
                print(f"Comando enviado: {command}")
                if command.startswith("STATE_"):
                    self.app.current_mux_state = int(command[-1])
                elif command == "START":
                    self.app.current_mux_state = -1
                elif command == "STOP":
                    self.app.current_mux_state = -2
            except Exception as e:
                print(f"Error al enviar comando: {e}")

def detect_r_peaks(signal_data, threshold, distance):
    if len(signal_data) < 3: return []
    peaks = []
    last_peak = -distance
    for i in range(1, len(signal_data) - 1):
        if (signal_data[i] > threshold and 
            signal_data[i] > signal_data[i-1] and 
            signal_data[i] > signal_data[i+1]):
            if i - last_peak >= distance:
                peaks.append(i)
                last_peak = i
    return peaks

class ECGApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Monitor ECG Integrado")
        self.geometry("1200x800")

        self.is_running = True

        # Variables de estado
        self.data_lock = threading.Lock()
        self.voltage_buffer = deque(maxlen=3000)
        self.filtered_buffer = deque(maxlen=3000)
        self.time_buffer = deque(maxlen=3000)
        self.sample_count = 0
        self.serial_connected = False
        self.trigger_port_connected = False # Nuevo estado para el puerto de disparo
        self.mux_state_label = {0: "I DERIVADA", 1: "II DERIVADA", 2: "III DERIVADA", 3: "aVR"}
        self.current_mux_state = 0
        self.mux_control_lock = threading.Lock()
        self.loose_lead_status = ["Desconocido"] * 4

        self.ecg_gain = tk.DoubleVar(value=1.0)
        self.window_size = tk.IntVar(value=1500)
        self.y_max = tk.DoubleVar(value=3.5)
        self.voltage_offset = tk.DoubleVar(value=0.0)
        self.r_threshold = tk.DoubleVar(value=R_THRESHOLD_DEFAULT)
        self.r_distance = tk.IntVar(value=R_DISTANCE_DEFAULT)

        self.ecg_filters = ECGFilters(self, SAMPLE_RATE)
        
        # --- Crear la interfaz ---
        self._create_widgets()

        # --- Iniciar lector serial ---
        self.serial_reader = SerialReader(SERIAL_PORT, BAUD_RATE, self)
        self.serial_reader.start()

        # --- Iniciar actualización de GUI ---
        self.update_gui()
        
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _create_widgets(self):
        main_frame = ttk.Frame(self, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Frame para plots
        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Frame para controles y estado
        sidebar_frame = ttk.Frame(main_frame, width=250)
        sidebar_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(10, 0))
        sidebar_frame.pack_propagate(False)

        # --- Gráficas Matplotlib ---
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        self.setup_plots()

        # --- Panel de Control ---
        control_panel = ttk.LabelFrame(sidebar_frame, text="Control de Derivadas", padding="10")
        control_panel.pack(fill=tk.X, pady=10)
        
        for i in range(4):
            state_command = f"STATE_{i}"
            state_label = self.mux_state_label[i]
            ttk.Button(control_panel, text=f"Manual: {state_label}",
                       command=lambda cmd=state_command: self.serial_reader.send_command(cmd)).pack(fill='x', pady=2)
        
        ttk.Separator(control_panel, orient='horizontal').pack(fill='x', pady=5)
        
        ttk.Button(control_panel, text="Iniciar Modo Automático",
                   command=lambda: self.serial_reader.send_command("START")).pack(fill='x', pady=2)
        ttk.Button(control_panel, text="Detener MUX",
                   command=lambda: self.serial_reader.send_command("STOP")).pack(fill='x', pady=2)

        # --- Panel de Ganancia ---
        gain_panel = ttk.LabelFrame(sidebar_frame, text="Control de Ganancia", padding="10")
        gain_panel.pack(fill=tk.X, pady=10)

        self.gain_label = ttk.Label(gain_panel, text=f"Ganancia: {self.ecg_gain.get():.2f}x")
        self.gain_label.pack()

        gain_slider = ttk.Scale(gain_panel, from_=0.1, to=5.0, orient=tk.HORIZONTAL, variable=self.ecg_gain, command=self.update_gain_label)
        gain_slider.pack(fill='x', pady=5)

        # --- Panel de Gráfica ---
        plot_control_panel = ttk.LabelFrame(sidebar_frame, text="Control de Gráfica", padding="10")
        plot_control_panel.pack(fill=tk.X, pady=10)

        self.window_size_label = ttk.Label(plot_control_panel, text=f"Ventana: {self.window_size.get()} muestras")
        self.window_size_label.pack()
        window_slider = ttk.Scale(plot_control_panel, from_=500, to=3000, orient=tk.HORIZONTAL, variable=self.window_size, command=self.update_plot_settings)
        window_slider.pack(fill='x', pady=5)

        self.y_max_label = ttk.Label(plot_control_panel, text=f"Amplitud: {self.y_max.get():.2f} V")
        self.y_max_label.pack()
        y_max_slider = ttk.Scale(plot_control_panel, from_=0.5, to=5.0, orient=tk.HORIZONTAL, variable=self.y_max, command=self.update_plot_settings)
        y_max_slider.pack(fill='x', pady=5)

        ttk.Separator(plot_control_panel, orient='horizontal').pack(fill='x', pady=5)

        self.voltage_offset_label = ttk.Label(plot_control_panel, text=f"Offset: {self.voltage_offset.get():.2f} V")
        self.voltage_offset_label.pack()
        voltage_offset_slider = ttk.Scale(plot_control_panel, from_=-1.0, to=1.0, orient=tk.HORIZONTAL, variable=self.voltage_offset, command=self.update_plot_settings)
        voltage_offset_slider.pack(fill='x', pady=5)

        # --- Panel de Detección de Picos ---
        peak_panel = ttk.LabelFrame(sidebar_frame, text="Detección de Picos", padding="10")
        peak_panel.pack(fill=tk.X, pady=10)

        self.r_threshold_label = ttk.Label(peak_panel, text=f"Umbral (V): {self.r_threshold.get():.2f}")
        self.r_threshold_label.pack()
        r_threshold_slider = ttk.Scale(peak_panel, from_=0.1, to=3.0, orient=tk.HORIZONTAL, variable=self.r_threshold, command=self.update_peak_settings)
        r_threshold_slider.pack(fill='x', pady=5)

        self.r_distance_label = ttk.Label(peak_panel, text=f"Distancia (ms): {self.r_distance.get() * 1000 / SAMPLE_RATE:.0f}")
        self.r_distance_label.pack()
        r_distance_slider = ttk.Scale(peak_panel, from_=20, to=1000, orient=tk.HORIZONTAL, variable=self.r_distance, command=self.update_peak_settings)
        r_distance_slider.pack(fill='x', pady=5)

        # --- Panel de Estado ---
        status_panel = ttk.LabelFrame(sidebar_frame, text="Estado del Sistema", padding="10")
        status_panel.pack(fill=tk.X, pady=10)

        self.status_labels = {}
        labels_text = ["Conexión ECG", "Puerto Disparo", "Muestras", "BPM", "Calidad Señal", "Derivada", "Electrodos"]
        for i, text in enumerate(labels_text):
            ttk.Label(status_panel, text=f"{text}:").grid(row=i, column=0, sticky="w", pady=2)
            self.status_labels[text] = ttk.Label(status_panel, text="Desconocido")
            self.status_labels[text].grid(row=i, column=1, sticky="w", pady=2, columnspan=2)

    def update_gain_label(self, value):
        self.gain_label.config(text=f"Ganancia: {float(value):.2f}x")

    def update_plot_settings(self, _=None):
        self.ax1.set_ylim(0, self.y_max.get())
        self.ax2.set_ylim(0, self.y_max.get())
        self.y_max_label.config(text=f"Amplitud: {self.y_max.get():.2f} V")
        self.window_size_label.config(text=f"Ventana: {self.window_size.get()} muestras")
        self.voltage_offset_label.config(text=f"Offset: {self.voltage_offset.get():.2f} V")

    def update_peak_settings(self, _=None):
        self.r_threshold_label.config(text=f"Umbral (V): {self.r_threshold.get():.2f}")
        distance_ms = self.r_distance.get() * 1000 / SAMPLE_RATE
        self.r_distance_label.config(text=f"Distancia (ms): {distance_ms:.0f}")

    def setup_plots(self):
        self.ax1.set_ylim(0, self.y_max.get())
        self.ax1.set_ylabel('Voltaje Original (V)')
        self.ax1.set_title('Monitor ECG - Señal Original')
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_ylim(0, self.y_max.get())
        self.ax2.set_xlabel('Muestras')
        self.ax2.set_ylabel('Voltaje Filtrado (V)')
        self.ax2.set_title(f'Señal Filtrada - {"ACTIVO" if ENABLE_FILTERS else "INACTIVO"}')
        self.ax2.grid(True, alpha=0.3)
        
        self.line_raw, = self.ax1.plot([], [], 'b-', linewidth=1.2, alpha=0.8)
        self.line_filtered, = self.ax2.plot([], [], 'g-', linewidth=1.5)
        self.peaks_line, = self.ax2.plot([], [], 'ro', markersize=5, alpha=0.8)
        
        self.fig.tight_layout()

    def update_gui(self):
        if not self.is_running:
            return

        with self.data_lock:
            y_raw = list(self.voltage_buffer)
            y_filtered = list(self.filtered_buffer)
            x_data = list(self.time_buffer)
        
        if not y_raw:
            self.status_labels["Conexión ECG"].config(text=f"{ '🟢 Conectado' if self.serial_connected else '🔴 Desconectado'}")
            self.status_labels["Puerto Disparo"].config(text=f"{ '🟢 Conectado' if self.trigger_port_connected else '🔴 Desconectado'}")
            self.after(refresh_interval, self.update_gui)
            return

        # --- Actualizar gráficas ---
        visible_slice = slice(-self.window_size.get(), None)
        y_raw_visible = y_raw[visible_slice]
        y_filtered_visible = y_filtered[visible_slice]
        x_visible = x_data[visible_slice]

        self.line_raw.set_data(x_visible, y_raw_visible)
        self.line_filtered.set_data(x_visible, y_filtered_visible)

        signal_for_peaks = y_filtered_visible if ENABLE_FILTERS else y_raw_visible
        if len(signal_for_peaks) > 10:
            peaks_idx = detect_r_peaks(signal_for_peaks, threshold=self.r_threshold.get(), distance=self.r_distance.get())
            if peaks_idx:
                peaks_x = [x_visible[i] for i in peaks_idx]
                peaks_y = [signal_for_peaks[i] for i in peaks_idx]
                self.peaks_line.set_data(peaks_x, peaks_y)
            else:
                self.peaks_line.set_data([], [])
        
        if x_visible:
            x_min = x_visible[0]
            x_max = x_visible[-1] if len(x_visible) >= self.window_size.get() else x_visible[0] + self.window_size.get()
            self.ax1.set_xlim(x_min, x_max)
            self.ax2.set_xlim(x_min, x_max)

        self.canvas.draw()

        # --- Actualizar etiquetas de estado ---
        self.status_labels["Conexión ECG"].config(text=f"{ '🟢 Conectado' if self.serial_connected else '🔴 Desconectado'}")
        self.status_labels["Puerto Disparo"].config(text=f"{ '🟢 Conectado' if self.trigger_port_connected else '🔴 Desconectado'}")
        self.status_labels["Muestras"].config(text=f"{len(y_raw)}")

        bpm_text = "Calculando..."
        if len(signal_for_peaks) > 0:
            peaks = detect_r_peaks(signal_for_peaks, threshold=self.r_threshold.get(), distance=self.r_distance.get())
            if len(peaks) > 1:
                avg_interval = np.mean(np.diff(peaks))
                bpm = 60 / (avg_interval / SAMPLE_RATE) if avg_interval > 0 else 0
                bpm_text = f"{bpm:.0f} BPM"
        self.status_labels["BPM"].config(text=bpm_text)

        noise_level = np.std(y_raw_visible[-100:]) if len(y_raw_visible) >= 100 else 0
        signal_quality = "🟢 BUENA" if noise_level < 0.05 else "🟡 REGULAR" if noise_level < 0.1 else "🔴 POBRE"
        self.status_labels["Calidad Señal"].config(text=f"{signal_quality} ({noise_level:.3f}V)")

        with self.mux_control_lock:
            if self.current_mux_state >= 0:
                mux_text = self.mux_state_label.get(self.current_mux_state, "Desconocido")
            elif self.current_mux_state == -1:
                mux_text = "Automático"
            else:
                mux_text = "Detenido"
        self.status_labels["Derivada"].config(text=mux_text)
        
        with self.data_lock:
            lead_status_str = " ".join(["🟢" if s == 0 else "🔴" for s in self.loose_lead_status])
        self.status_labels["Electrodos"].config(text=lead_status_str)

        self.after(refresh_interval, self.update_gui)

    def on_closing(self):
        self.is_running = False
        print("Cerrando aplicación...")
        self.serial_reader.stop()
        self.destroy()

if __name__ == "__main__":
    print("=== 📊 MONITOR ECG CON INTERFAZ TKINTER ===")
    app = ECGApp()
    app.mainloop()
