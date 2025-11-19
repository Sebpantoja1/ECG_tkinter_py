import csv
from pathlib import Path
from typing import List, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from .session_manager import SessionManager

class DataRecorder:
    """Graba datos a CSV en tiempo real"""
    
    def __init__(self, filepath: Path):
        self.filepath = filepath
        self.file = None
        self.writer = None
        self.buffer = []
        self.buffer_size = 100  # Flush cada 100 muestras
        
    def start(self, headers: List[str]):
        """Abre archivo y escribe headers"""
        self.file = open(self.filepath, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(headers)
        self.file.flush()
        
    def write_row(self, data: List[Any]):
        """Escribe fila (con buffering)"""
        self.buffer.append(data)
        if len(self.buffer) >= self.buffer_size:
            self.flush()
            
    def flush(self):
        """Fuerza escritura de buffer"""
        if self.writer and self.buffer:
            self.writer.writerows(self.buffer)
            self.file.flush()
            self.buffer.clear()
            
    def stop(self):
        """Cierra archivo"""
        self.flush()
        if self.file:
            self.file.close()

class MultiStreamRecorder:
    """Coordina grabación de múltiples streams"""
    
    def __init__(self, session_manager: SessionManager):
        self.session = session_manager
        
        # Recorders individuales
        self.ecg_raw_recorder = None
        self.ecg_filtered_recorder = None
        self.pt_recorder = None  # Pan-Tompkins
        self.cardio_recorder = None  # Arduino cardioverter
        
    def start_recording(self):
        """Inicia todos los recorders"""
        self.ecg_raw_recorder = self.session.get_recorder("ecg_raw")
        self.ecg_raw_recorder.start(["timestamp", "sample_index", "voltage"])
        
        self.ecg_filtered_recorder = self.session.get_recorder("ecg_filtered")
        self.ecg_filtered_recorder.start(["timestamp", "sample_index", "voltage"])
        
        self.pt_recorder = self.session.get_recorder("pan_tompkins")
        self.pt_recorder.start([
            "timestamp", "r_peak_index", "rr_interval", "hr_bpm", 
            "sdnn", "rmssd", "pnn50"
        ])
        
        self.cardio_recorder = self.session.get_recorder("cardioverter")
        self.cardio_recorder.start([
            "timestamp", "event", "vcap", "energy_pos", "energy_neg",
            "percent_pos", "percent_neg", "total_energy"
        ])
        
    def record_ecg_sample(self, timestamp: float, index: int, 
                          raw: float, filtered: float):
        """Graba muestra de ECG"""
        self.ecg_raw_recorder.write_row([timestamp, index, raw])
        self.ecg_filtered_recorder.write_row([timestamp, index, filtered])
        
    def record_r_peak(self, timestamp: float, data: dict):
        """Graba detección de pico R con métricas HRV"""
        # Pan-Tompkins retorna múltiples picos, grabar cada uno
        r_peaks = data.get('r_peaks', [])
        rr_intervals = data.get('rr_intervals', [])
        hr = data.get('hr', 0.0)
        hrv_metrics = data.get('hrv_metrics', {})
        
        # Grabar el último pico R detectado con sus métricas
        if len(r_peaks) > 0:
            last_peak_idx = r_peaks[-1]
            last_rr = rr_intervals[-1] if len(rr_intervals) > 0 else 0.0
            
            self.pt_recorder.write_row([
                timestamp,
                last_peak_idx,
                last_rr,
                hr,
                hrv_metrics.get('sdnn', 0.0),
                hrv_metrics.get('rmssd', 0.0),
                hrv_metrics.get('pnn50', 0.0)
            ])
        
    def record_cardioverter_event(self, timestamp: float, event_data: dict):
        """Graba evento del cardioversor"""
        self.cardio_recorder.write_row([
            timestamp,
            event_data['type'],
            event_data.get('vcap', 0),
            event_data.get('energy_pos', 0),
            event_data.get('energy_neg', 0),
            event_data.get('percent_pos', 0),
            event_data.get('percent_neg', 0),
            event_data.get('total_energy', 0)
        ])
        
    def stop_recording(self):
        """Detiene todos los recorders"""
        for recorder in [self.ecg_raw_recorder, self.ecg_filtered_recorder,
                        self.pt_recorder, self.cardio_recorder]:
            if recorder:
                recorder.stop()
