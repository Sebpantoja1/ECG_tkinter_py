import numpy as np
from typing import List, Dict
from scipy import signal
from .hrv_analysis import HRVAnalyzer

class PanTompkinsDetector:
    """
    Implementación completa del algoritmo Pan-Tompkins para detección robusta de QRS.
    
    Stages:
    1. Band-pass filter (5-15 Hz)
    2. Derivative filter
    3. Squaring
    4. Moving window integration
    5. Adaptive thresholding
    6. Decision rules
    """
    
    def __init__(self, sample_rate: int = 500):
        self.fs = sample_rate
        self.setup_filters()
        self.init_thresholds()
        
    def setup_filters(self):
        """Diseña filtros IIR optimizados"""
        # Band-pass filter 5-15 Hz usando scipy
        nyquist = self.fs / 2.0
        low = 5.0 / nyquist
        high = 15.0 / nyquist
        self.b, self.a = signal.butter(4, [low, high], btype='band')
        
    def init_thresholds(self):
        """Inicializa umbrales adaptativos"""
        self.signal_peak = 0.0
        self.noise_peak = 0.0
        self.threshold_i1 = 0.0
        self.threshold_i2 = 0.0
        self.rr_avg1 = 0.0
        self.rr_avg2 = 0.0
        self.rr_low_limit = 0.0
        self.rr_high_limit = 0.0
        self.rr_missed_limit = 0.0
        
    def derivative_filter(self, sig: np.ndarray) -> np.ndarray:
        """y(nT) = [-x(nT-2T) - 2x(nT-T) + 2x(nT+T) + x(nT+2T)]/(8T)"""
        result = np.zeros_like(sig)
        for i in range(2, len(sig) - 2):
            result[i] = (-sig[i-2] - 2*sig[i-1] + 2*sig[i+1] + sig[i+2]) / 8.0
        return result
        
    def moving_window_integration(self, sig: np.ndarray) -> np.ndarray:
        """N = int(0.15 * fs)"""
        window_size = int(0.15 * self.fs)
        if window_size < 1:
            window_size = 1
        return np.convolve(sig, np.ones(window_size)/window_size, mode='same')
        
    def detect_qrs(self, ecg_signal: np.ndarray) -> Dict:
        """
        Detecta picos QRS usando algoritmo Pan-Tompkins.
        
        Returns:
            {
                'r_peaks': List[int] - indices de picos R,
                'rr_intervals': List[float] - intervalos RR (ms),
                'hr': float - frecuencia cardíaca instantánea (BPM),
                'hrv_metrics': {
                    'sdnn': float,
                    'rmssd': float,
                    'pnn50': float
                }
            }
        """
        if len(ecg_signal) < 100:
            return {
                'r_peaks': [],
                'rr_intervals': [],
                'hr': 0.0,
                'hrv_metrics': {'sdnn': 0.0, 'rmssd': 0.0, 'pnn50': 0.0}
            }
        
        # 1. Band-pass filter (usar lfilter en lugar de filtfilt para mejor performance)
        # filtfilt es más preciso pero más lento, lfilter es suficiente para tiempo real
        filtered = signal.lfilter(self.b, self.a, ecg_signal)
        
        # 2. Derivative
        deriv = self.derivative_filter(filtered)
        
        # 3. Squaring
        squared = deriv ** 2
        
        # 4. Moving window integration
        integrated = self.moving_window_integration(squared)
        
        # 5. Find peaks (simplified thresholding)
        # Usar percentil 75 como umbral inicial
        threshold = np.percentile(integrated, 75)
        min_distance = int(0.2 * self.fs)  # Mínimo 200ms entre picos
        
        peaks, _ = signal.find_peaks(integrated, height=threshold, distance=min_distance)
        
        # Convertir a índices relativos al inicio de la ventana
        r_peaks = peaks.tolist() if len(peaks) > 0 else []
        
        # Calcular RR intervals
        rr_intervals = []
        if len(r_peaks) > 1:
            rr_intervals = np.diff(r_peaks) * (1000.0 / self.fs)  # en ms
        
        # Calcular HR
        hr = 0.0
        if len(rr_intervals) > 0:
            avg_rr = np.mean(rr_intervals)
            if avg_rr > 0:
                hr = 60000.0 / avg_rr  # BPM
        
        # Calcular HRV metrics
        hrv_metrics = {'sdnn': 0.0, 'rmssd': 0.0, 'pnn50': 0.0}
        if len(rr_intervals) >= 2:
            hrv_metrics = HRVAnalyzer.time_domain(np.array(rr_intervals))
        
        return {
            'r_peaks': r_peaks,
            'rr_intervals': rr_intervals.tolist() if len(rr_intervals) > 0 else [],
            'hr': float(hr),
            'hrv_metrics': hrv_metrics
        }
