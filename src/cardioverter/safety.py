import numpy as np
from typing import List, Tuple

class SafetyValidator:
    """Validaciones de seguridad para cardioversión"""
    
    MIN_RR_INTERVAL = 300   # ms (200 BPM max)
    MAX_RR_INTERVAL = 2000  # ms (30 BPM min)
    MIN_RESERVE_PERCENT = 15.0
    
    @staticmethod
    def validate_energy_config(percent_pos: float, percent_neg: float) -> tuple[bool, str]:
        """Valida que %E+ + %E- <= 85%"""
        total = percent_pos + percent_neg
        if total > (100.0 - SafetyValidator.MIN_RESERVE_PERCENT):
            return False, f"Total energy {total}% exceeds safe limit (85%)"
        if percent_pos < 0 or percent_neg < 0:
            return False, "Negative percentages not allowed"
        return True, "OK"
        
    @staticmethod
    def validate_hr_stable(rr_intervals: List[float]) -> tuple[bool, str]:
        """Valida que HR esté estable (CV < 20%)"""
        if len(rr_intervals) < 5:
            return False, "Insufficient RR history"
        recent = rr_intervals[-10:]
        cv = np.std(recent) / np.mean(recent)
        if cv > 0.20:
            return False, f"HR unstable (CV={cv:.1%})"
        return True, "OK"
