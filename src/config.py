"""
Configuration constants for the ECG Monitor application.
"""

# --- Serial Port Configuration ---
SERIAL_PORT = "COM8"  # COM port for ECG data
BAUD_RATE = 115200

# --- Trigger Port Configuration ---
TRIGGER_SERIAL_PORT = "COM11"  # COM port for sending trigger signals
TRIGGER_BAUD_RATE = 9600
TRIGGER_SIGNAL = b'\x01'  # Byte to send when an R peak is detected

# --- Debug and Data Mode ---
DEBUG_MODE = True  # Set to False for normal mode
RAW_DATA_MODE = True  # Display raw data without filtering

# --- Filter Configuration ---
SAMPLE_RATE = 2000  # Hz (adjust according to your ESP32)
ENABLE_FILTERS = True  # Enable/disable filters

# --- UI and Plotting Configuration ---
REFRESH_INTERVAL = 50  # GUI refresh rate in ms

# --- Peak Detection Parameters ---
R_THRESHOLD_DEFAULT = 1.0  # Default R-peak threshold in Volts
R_DISTANCE_DEFAULT = 200  # Default minimum samples between peaks
