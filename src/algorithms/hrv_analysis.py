import numpy as np
from typing import Dict

class HRVAnalyzer:
    """Calcula métricas avanzadas de HRV"""
    
    @staticmethod
    def time_domain(rr_intervals: np.ndarray) -> Dict:
        """SDNN, RMSSD, pNN50, pNN20"""
        if len(rr_intervals) < 2:
            return {'sdnn': 0.0, 'rmssd': 0.0, 'pnn50': 0.0, 'pnn20': 0.0}
        
        # SDNN: desviación estándar de intervalos NN
        sdnn = float(np.std(rr_intervals))
        
        # RMSSD: raíz cuadrada media de diferencias sucesivas
        diff = np.diff(rr_intervals)
        rmssd = float(np.sqrt(np.mean(diff ** 2))) if len(diff) > 0 else 0.0
        
        # pNN50: porcentaje de diferencias > 50ms
        diff_abs = np.abs(diff)
        pnn50 = float(np.sum(diff_abs > 50) / len(diff_abs) * 100) if len(diff_abs) > 0 else 0.0
        
        # pNN20: porcentaje de diferencias > 20ms
        pnn20 = float(np.sum(diff_abs > 20) / len(diff_abs) * 100) if len(diff_abs) > 0 else 0.0
        
        return {
            'sdnn': sdnn,
            'rmssd': rmssd,
            'pnn50': pnn50,
            'pnn20': pnn20
        }
        
    @staticmethod
    def frequency_domain(rr_intervals: np.ndarray, fs: float) -> dict:
        """VLF, LF, HF, LF/HF ratio usando Welch"""
        pass
        
    @staticmethod
    def nonlinear(rr_intervals: np.ndarray) -> dict:
        """SD1, SD2 (Poincaré plot)"""
        pass
