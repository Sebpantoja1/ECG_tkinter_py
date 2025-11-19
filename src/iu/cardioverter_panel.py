import tkinter as tk
from tkinter import ttk, messagebox
from ..cardioverter.state_machine import CardioStateMachine, CardioState, DischargMode
from ..cardioverter.safety import SafetyValidator

class CardioverterPanel(ttk.Frame):
    """Panel de control del cardioversor"""
    
    def __init__(self, parent, state_machine: CardioStateMachine):
        super().__init__(parent)
        self.fsm = state_machine
        
        self.create_widgets()
        self.update_ui()
        
    def create_widgets(self):
        # === SECCIÓN 1: CONFIGURACIÓN ===
        config_frame = ttk.LabelFrame(self, text="⚙️ Configuration", padding=10)
        config_frame.grid(row=0, column=0, sticky="ew", padx=5, pady=5)
        
        # Vcap target
        ttk.Label(config_frame, text="Target Vcap (V):").grid(row=0, column=0)
        self.vcap_var = tk.DoubleVar(value=20.0)
        vcap_spinner = ttk.Spinbox(config_frame, from_=10, to=30, 
                                   textvariable=self.vcap_var, width=10)
        vcap_spinner.grid(row=0, column=1)
        
        # Energy %pos
        ttk.Label(config_frame, text="Energy Phase + (%):").grid(row=1, column=0)
        self.pos_var = tk.DoubleVar(value=50.0)
        pos_scale = ttk.Scale(config_frame, from_=0, to=85, 
                             variable=self.pos_var, orient="horizontal")
        pos_scale.grid(row=1, column=1)
        
        # Energy %neg
        ttk.Label(config_frame, text="Energy Phase - (%):").grid(row=2, column=0)
        self.neg_var = tk.DoubleVar(value=35.0)
        neg_scale = ttk.Scale(config_frame, from_=0, to=85, 
                             variable=self.neg_var, orient="horizontal")
        neg_scale.grid(row=2, column=1)
        
        # === SECCIÓN 2: ESTADO ===
        status_frame = ttk.LabelFrame(self, text="📊 Status", padding=10)
        status_frame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)
        
        self.state_label = ttk.Label(status_frame, text="State: IDLE", 
                                     font=("Arial", 14, "bold"))
        self.state_label.pack()
        
        # === SECCIÓN 3: CONTROLES ===
        controls_frame = ttk.LabelFrame(self, text="🎛️ Controls", padding=10)
        controls_frame.grid(row=2, column=0, sticky="ew", padx=5, pady=5)
        
        # Botón ARM (paso 1)
        self.arm_btn = ttk.Button(controls_frame, text="🔓 ARM CARDIOVERTER",
                                  command=self.on_arm, style="Warning.TButton")
        self.arm_btn.pack(fill="x", pady=2)
        
        # Botón CONFIRM (paso 2) - inicialmente disabled
        self.confirm_btn = ttk.Button(controls_frame, text="✓ CONFIRM ARM",
                                      command=self.on_confirm, 
                                      state="disabled", style="Danger.TButton")
        self.confirm_btn.pack(fill="x", pady=2)
        
        # Frame para botones de descarga
        discharge_frame = ttk.Frame(controls_frame)
        discharge_frame.pack(fill="x", pady=5)
        
        # Botón MANUAL
        self.manual_btn = ttk.Button(discharge_frame, text="⚡ MANUAL DISCHARGE",
                                     command=self.on_manual_discharge,
                                     state="disabled")
        self.manual_btn.pack(side="left", expand=True, fill="x", padx=2)
        
        # Botón AUTO
        self.auto_btn = ttk.Button(discharge_frame, text="🔄 AUTO DISCHARGE",
                                   command=self.on_auto_discharge,
                                   state="disabled")
        self.auto_btn.pack(side="left", expand=True, fill="x", padx=2)
        
        # Botón ABORT
        self.abort_btn = ttk.Button(controls_frame, text="❌ ABORT / RESET",
                                    command=self.on_abort, style="Danger.TButton")
        self.abort_btn.pack(fill="x", pady=2)
        
    def on_arm(self):
        """Paso 1: Armar sin confirmar"""
        if self.fsm.arm_cardioverter():
            messagebox.showwarning("ARM", 
                "Cardioverter ARMED but NOT confirmed.\n"
                "Press CONFIRM to proceed.")
            self.update_ui()
        else:
            messagebox.showerror("Error", "Cannot arm in current state")
            
    def on_confirm(self):
        """Paso 2: Doble confirmación"""
        # Validar config
        valid, msg = SafetyValidator.validate_energy_config(
            self.pos_var.get(), self.neg_var.get())
        if not valid:
            messagebox.showerror("Config Error", msg)
            return
            
        # Confirmar con diálogo
        result = messagebox.askyesno("CONFIRM ARM",
            f"CONFIRM cardioverter arm with:\n"
            f"  Vcap: {self.vcap_var.get():.1f} V\n"
            f"  Phase +: {self.pos_var.get():.1f}%\n"
            f"  Phase -: {self.neg_var.get():.1f}%\n\n"
            f"Continue?",
            icon="warning")
            
        if result:
            self.fsm.target_vcap = self.vcap_var.get()
            self.fsm.percent_pos = self.pos_var.get()
            self.fsm.percent_neg = self.neg_var.get()
            
            if self.fsm.confirm_arm():
                messagebox.showinfo("READY", 
                    "Cardioverter ARMED and READY.\n"
                    "Select discharge mode.")
                self.update_ui()
                
    def on_manual_discharge(self):
        """Descarga manual: espera próximo R peak"""
        if self.fsm.authorize_discharge(DischargMode.MANUAL):
            messagebox.showinfo("Manual Mode", 
                "Waiting for next R peak to discharge...")
            self.update_ui()
            
    def on_auto_discharge(self):
        """Descarga automática: detecta R y descarga"""
        result = messagebox.askyesno("AUTO DISCHARGE",
            "AUTO mode will discharge on NEXT R peak detected.\n"
            "Continue?",
            icon="warning")
        if result:
            if self.fsm.authorize_discharge(DischargMode.AUTO):
                self.update_ui()
                
    def on_abort(self):
        """Abortar y resetear"""
        self.fsm.reset()
        self.update_ui()
        messagebox.showinfo("Reset", "Cardioverter reset to IDLE")
        
    def update_ui(self):
        """Actualizar UI según estado FSM"""
        state = self.fsm.state
        
        # Actualizar label de estado
        self.state_label.config(text=f"State: {state.name}")
        
        # Habilitar/deshabilitar botones según estado
        if state == CardioState.IDLE:
            self.arm_btn.config(state="normal")
            self.confirm_btn.config(state="disabled")
            self.manual_btn.config(state="disabled")
            self.auto_btn.config(state="disabled")
            
        elif state == CardioState.ARMED_STANDBY:
            self.arm_btn.config(state="disabled")
            self.confirm_btn.config(state="normal")
            self.manual_btn.config(state="disabled")
            self.auto_btn.config(state="disabled")
            
        elif state == CardioState.ARMED_READY:
            self.arm_btn.config(state="disabled")
            self.confirm_btn.config(state="disabled")
            self.manual_btn.config(state="normal")
            self.auto_btn.config(state="normal")
            
        else:  # WAITING, DISCHARGING, COOLDOWN
            self.arm_btn.config(state="disabled")
            self.confirm_btn.config(state="disabled")
            self.manual_btn.config(state="disabled")
            self.auto_btn.config(state="disabled")
