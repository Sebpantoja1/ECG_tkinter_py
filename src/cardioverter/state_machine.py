from enum import Enum, auto
from typing import Optional
import time

class CardioState(Enum):
    """Estados del cardioversor desde perspectiva de la UI"""
    IDLE = auto()              # Inactivo
    ARMED_STANDBY = auto()     # Armado pero en espera (doble confirm)
    ARMED_READY = auto()       # Confirmado, listo para descarga
    WAITING_R_PEAK = auto()    # Esperando próximo pico R
    DISCHARGING = auto()       # Descargando (Arduino en control)
    COOLDOWN = auto()          # Periodo refractario post-descarga
    ERROR = auto()             # Estado de error

class DischargMode(Enum):
    MANUAL = auto()    # Usuario presiona botón, espera R peak
    AUTO = auto()      # Detecta R peak y descarga automáticamente

class CardioStateMachine:
    """
    Controla la lógica de seguridad del cardioversor.
    Restricción: SOLO puede descargar cuando el usuario lo autoriza.
    """
    
    def __init__(self, serial_handler):
        self.state = CardioState.IDLE
        self.mode = DischargMode.MANUAL
        self.serial = serial_handler
        
        # Safety flags
        self.armed_confirmed = False
        self.discharge_authorized = False
        
        # Timers
        self.arm_time: Optional[float] = None
        self.discharge_time: Optional[float] = None
        self.cooldown_duration = 5.0  # segundos
        
        # Config
        self.target_vcap = 20.0       # V
        self.percent_pos = 50.0       # %
        self.percent_neg = 35.0       # %
        
    def arm_cardioverter(self) -> bool:
        """Paso 1: Armar (sin confirmar)"""
        if self.state != CardioState.IDLE:
            return False
        self.state = CardioState.ARMED_STANDBY
        self.arm_time = time.time()
        return True
        
    def confirm_arm(self) -> bool:
        """Paso 2: Doble confirmación"""
        if self.state != CardioState.ARMED_STANDBY:
            return False
        
        # Enviar config al Arduino
        self.send_config_to_arduino()
        
        self.state = CardioState.ARMED_READY
        self.armed_confirmed = True
        return True
        
    def authorize_discharge(self, mode: DischargMode) -> bool:
        """Paso 3: Autorizar descarga (manual o auto)"""
        if self.state != CardioState.ARMED_READY:
            return False
        
        self.mode = mode
        self.discharge_authorized = True
        self.state = CardioState.WAITING_R_PEAK
        return True
        
    def on_r_peak_detected(self) -> bool:
        """Llamado cuando Pan-Tompkins detecta pico R"""
        if self.state != CardioState.WAITING_R_PEAK:
            return False
            
        if not self.discharge_authorized:
            return False
        
        # Enviar comando de trigger al Arduino
        self.serial.send_trigger()
        
        self.state = CardioState.DISCHARGING
        self.discharge_time = time.time()
        return True
        
    def on_discharge_complete(self):
        """Llamado cuando Arduino reporta descarga completa"""
        self.state = CardioState.COOLDOWN
        self.discharge_authorized = False
        self.armed_confirmed = False
        
    def update(self):
        """Llamar en loop principal"""
        if self.state == CardioState.COOLDOWN:
            if time.time() - self.discharge_time > self.cooldown_duration:
                self.reset()
                
    def reset(self):
        """Volver a IDLE"""
        self.state = CardioState.IDLE
        self.armed_confirmed = False
        self.discharge_authorized = False
        
    def send_config_to_arduino(self):
        """Envía Vcap_target, %pos, %neg al Arduino vía serial"""
        self.serial.send_config(self.target_vcap, self.percent_pos, self.percent_neg)
