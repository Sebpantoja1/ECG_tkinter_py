from pathlib import Path
from datetime import datetime
from typing import Optional
import json
from dataclasses import asdict
from .patient_info import PatientInfo
from .data_logger import DataRecorder

class SessionManager:
    """Gestiona sesiones de grabación"""
    
    def __init__(self, base_dir: str = "recordings"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(exist_ok=True)
        
        self.current_session: Optional[Path] = None
        self.patient_info: Optional[PatientInfo] = None
        self.recording = False
        
    def create_session(self, patient_info: PatientInfo) -> Path:
        """
        Crea carpeta de sesión:
        recordings/
          └── YYYYMMDD_HHMMSS_PatientID/
              ├── patient_info.json
              ├── ecg_raw.csv
              ├── ecg_filtered.csv
              ├── pan_tompkins.csv
              ├── cardioverter.csv
              └── session_summary.pdf
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        session_dir = self.base_dir / f"{timestamp}_{patient_info.patient_id}"
        session_dir.mkdir(exist_ok=True)
        
        # Guardar info del paciente
        with open(session_dir / "patient_info.json", "w") as f:
            json.dump(asdict(patient_info), f, indent=2, default=str)
        
        self.current_session = session_dir
        self.patient_info = patient_info
        return session_dir
        
    def get_recorder(self, data_type: str) -> 'DataRecorder':
        """Retorna recorder específico para tipo de datos"""
        if not self.current_session:
            raise ValueError("No active session")
        
        filepath = self.current_session / f"{data_type}.csv"
        return DataRecorder(filepath)
