import tkinter as tk
from tkinter import ttk, messagebox
from typing import Optional
from ..recording.session_manager import SessionManager
from ..recording.data_logger import MultiStreamRecorder
from ..recording.patient_info import PatientInfoDialog

class RecordingPanel(ttk.Frame):
    """Panel de control de grabación"""
    
    def __init__(self, parent, session_manager: SessionManager):
        super().__init__(parent)
        self.session = session_manager
        self.recorder: Optional[MultiStreamRecorder] = None
        
        self.create_widgets()
        
    def create_widgets(self):
        # Botón para crear nueva sesión
        self.new_session_btn = ttk.Button(
            self, text="📋 New Session", 
            command=self.on_new_session
        )
        self.new_session_btn.pack(fill="x", pady=2)
        
        # Estado de grabación
        self.recording_label = ttk.Label(
            self, text="⚫ Not Recording", 
            font=("Arial", 12)
        )
        self.recording_label.pack(pady=5)
        
        # Botón Start/Stop
        self.record_btn = ttk.Button(
            self, text="🔴 Start Recording",
            command=self.toggle_recording,
            state="disabled"
        )
        self.record_btn.pack(fill="x", pady=2)
        
        # Info de sesión actual
        self.session_info = ttk.Label(self, text="No active session")
        self.session_info.pack(pady=5)
        
    def on_new_session(self):
        """Abre diálogo para crear sesión"""
        dialog = PatientInfoDialog(self)
        self.wait_window(dialog)
        
        if dialog.result:
            session_dir = self.session.create_session(dialog.result)
            self.session_info.config(text=f"Session: {session_dir.name}")
            self.record_btn.config(state="normal")
            messagebox.showinfo("Session Created", 
                f"Session directory: {session_dir}")
            
    def toggle_recording(self):
        """Inicia/detiene grabación"""
        if not self.session.recording:
            # Iniciar
            self.recorder = MultiStreamRecorder(self.session)
            self.recorder.start_recording()
            self.session.recording = True
            
            # Notificar a app.py que el recorder está listo
            if hasattr(self.master, 'app_ref'):
                self.master.app_ref.multi_recorder = self.recorder
            
            self.recording_label.config(text="🔴 RECORDING")
            self.record_btn.config(text="⬛ Stop Recording")
            self.new_session_btn.config(state="disabled")
        else:
            # Detener
            self.recorder.stop_recording()
            self.session.recording = False
            
            # Limpiar referencia en app.py
            if hasattr(self.master, 'app_ref'):
                self.master.app_ref.multi_recorder = None
            
            self.recording_label.config(text="⚫ Not Recording")
            self.record_btn.config(text="🔴 Start Recording")
            self.new_session_btn.config(state="normal")
            
            messagebox.showinfo("Recording Stopped",
                f"Data saved to:\n{self.session.current_session}")
