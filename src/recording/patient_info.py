import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from datetime import datetime

@dataclass
class PatientInfo:
    """Información del paciente para sesión"""
    patient_id: str
    age: int
    gender: str
    diagnosis: str
    notes: str
    timestamp: datetime

class PatientInfoDialog(tk.Toplevel):
    """Diálogo para ingresar datos del paciente"""
    
    def __init__(self, parent):
        super().__init__(parent)
        self.title("Patient Information")
        self.result = None
        
        self.create_widgets()
        
    def create_widgets(self):
        # Campos: ID, Age, Gender, Diagnosis, Notes
        ttk.Label(self, text="Patient ID:").grid(row=0, column=0, padx=5, pady=5)
        self.id_entry = ttk.Entry(self, width=30)
        self.id_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(self, text="Age:").grid(row=1, column=0, padx=5, pady=5)
        self.age_entry = ttk.Entry(self, width=30)
        self.age_entry.grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(self, text="Gender:").grid(row=2, column=0, padx=5, pady=5)
        self.gender_var = tk.StringVar(value="M")
        gender_frame = ttk.Frame(self)
        gender_frame.grid(row=2, column=1, padx=5, pady=5)
        ttk.Radiobutton(gender_frame, text="Male", variable=self.gender_var, value="M").pack(side=tk.LEFT)
        ttk.Radiobutton(gender_frame, text="Female", variable=self.gender_var, value="F").pack(side=tk.LEFT)
        
        ttk.Label(self, text="Diagnosis:").grid(row=3, column=0, padx=5, pady=5)
        self.diagnosis_entry = ttk.Entry(self, width=30)
        self.diagnosis_entry.grid(row=3, column=1, padx=5, pady=5)
        
        ttk.Label(self, text="Notes:").grid(row=4, column=0, padx=5, pady=5)
        self.notes_text = tk.Text(self, width=30, height=4)
        self.notes_text.grid(row=4, column=1, padx=5, pady=5)
        
        ttk.Button(self, text="Start Session", 
                   command=self.on_submit).grid(row=5, column=0, columnspan=2, pady=10)
        
    def on_submit(self):
        try:
            age = int(self.age_entry.get()) if self.age_entry.get() else 0
        except ValueError:
            age = 0
            
        self.result = PatientInfo(
            patient_id=self.id_entry.get() or "UNKNOWN",
            age=age,
            gender=self.gender_var.get(),
            diagnosis=self.diagnosis_entry.get() or "",
            notes=self.notes_text.get("1.0", tk.END).strip(),
            timestamp=datetime.now()
        )
        self.destroy()
