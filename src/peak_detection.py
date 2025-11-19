# REEMPLAZAR todo el contenido con wrapper a Pan-Tompkins

import numpy as np
from .algorithms.pan_tompkins import PanTompkinsDetector


class PeakDetector:

    def __init__(self, sample_rate: int):
        self.pt_detector = PanTompkinsDetector(sample_rate)
        
    def detect(self, signal: np.ndarray) -> dict:
        return self.pt_detector.detect_qrs(signal)
