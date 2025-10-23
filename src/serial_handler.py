import serial
import time
import threading
from collections import deque
from . import config

class SerialReader:
    def __init__(self, app_state, ecg_filters):
        self.app_state = app_state
        self.ecg_filters = ecg_filters
        self.ser = None
        self.trigger_ser = None
        self.running = False
        self.sync_buffer = bytearray()
        
        self.realtime_peak_buffer = deque(maxlen=config.R_DISTANCE_DEFAULT * 2)
        self.last_peak_sample_count = 0

    def connect(self):
        try:
            if self.ser: self.ser.close()
            self.ser = serial.Serial(config.SERIAL_PORT, config.BAUD_RATE, timeout=0.1)
            time.sleep(2)
            self.ser.flushInput()
            self.ser.flushOutput()
            print("ECG port connected.")
            return True
        except Exception as e:
            print(f"Error connecting to ECG port: {e}")
            return False

    def connect_trigger(self):
        try:
            if self.trigger_ser and self.trigger_ser.is_open:
                self.trigger_ser.close()
            self.trigger_ser = serial.Serial(config.TRIGGER_SERIAL_PORT, config.TRIGGER_BAUD_RATE, timeout=0.1)
            print(f"Trigger port {config.TRIGGER_SERIAL_PORT} connected.")
            return True
        except Exception as e:
            print(f"Error connecting trigger port {config.TRIGGER_SERIAL_PORT}: {e}")
            return False
    
    def decode_packet(self, pkt):
        if len(pkt) != 4 or pkt[0] != 0xAA:
            return None
        
        start, lsb, msb, checksum = pkt
        expected_checksum = start ^ lsb ^ msb
        if checksum != expected_checksum:
            return None
            
        val = (msb << 8) | lsb
        voltage = val * (3.3 / 4095)
        return voltage

    def check_for_r_peak_and_trigger(self, new_sample):
        self.realtime_peak_buffer.append(new_sample)
        if len(self.realtime_peak_buffer) < 3:
            return

        mid_idx = len(self.realtime_peak_buffer) - 2
        p_before, p_peak, p_after = self.realtime_peak_buffer[mid_idx-1], self.realtime_peak_buffer[mid_idx], self.realtime_peak_buffer[mid_idx+1]

        threshold = self.app_state.r_threshold.get()
        distance = self.app_state.r_distance.get()

        if p_peak > threshold and p_peak > p_before and p_peak > p_after:
            if (self.app_state.sample_count - self.last_peak_sample_count) >= distance:
                if self.trigger_ser and self.trigger_ser.is_open:
                    try:
                        self.trigger_ser.write(config.TRIGGER_SIGNAL)
                    except Exception as e:
                        print(f"Error writing to trigger port: {e}")
                        self.app_state.trigger_port_connected = False
                self.last_peak_sample_count = self.app_state.sample_count
    
    def read_data(self):
        print("Starting data acquisition thread...")
        while self.running:
            try:
                if not self.ser or not self.ser.is_open:
                    self.app_state.serial_connected = False
                    if not self.connect():
                        time.sleep(1)
                        continue
                self.app_state.serial_connected = True

                if not self.trigger_ser or not self.trigger_ser.is_open:
                    self.app_state.trigger_port_connected = self.connect_trigger()
                else:
                    self.app_state.trigger_port_connected = True
                
                if self.ser.in_waiting > 0:
                    raw_bytes = self.ser.read(self.ser.in_waiting)
                    self.process_raw_bytes(raw_bytes)

                time.sleep(0.005)
            except Exception as e:
                print(f"Error in serial reading loop: {e}")
                self.app_state.serial_connected = False
                self.app_state.trigger_port_connected = False
                time.sleep(1)

    def process_raw_bytes(self, raw_bytes):
        # Check for lead status message first
        try:
            message = raw_bytes.decode('utf-8').strip()
            if message.startswith("LEADS_STATE:"):
                state_str = message.split(':')[1]
                if len(state_str) == 4:
                    with self.app_state.data_lock:
                        self.app_state.loose_lead_status = [int(s) for s in state_str]
                    return # Don't process this as data
        except (UnicodeDecodeError, IndexError, ValueError):
            pass # Not a valid status message, treat as data

        self.sync_buffer.extend(raw_bytes)
        
        while len(self.sync_buffer) >= 4:
            start_idx = self.sync_buffer.find(b'\xaa')
            if start_idx == -1:
                self.sync_buffer.clear()
                break
            
            if start_idx > 0:
                del self.sync_buffer[:start_idx]

            if len(self.sync_buffer) >= 4:
                packet = self.sync_buffer[:4]
                voltage = self.decode_packet(packet)
                
                if voltage is not None:
                    # Apply gain
                    voltage *= self.app_state.ecg_gain.get()
                    
                    # Filter the signal
                    filtered_voltage = self.ecg_filters.process_sample(voltage)
                    
                    # Apply offset post-filtering
                    final_filtered_voltage = filtered_voltage + self.app_state.voltage_offset.get()

                    # Real-time peak detection for trigger
                    self.check_for_r_peak_and_trigger(final_filtered_voltage)
                    
                    with self.app_state.data_lock:
                        self.app_state.voltage_buffer.append(voltage)
                        self.app_state.filtered_buffer.append(final_filtered_voltage)
                        self.app_state.time_buffer.append(self.app_state.sample_count)
                        self.app_state.sample_count += 1
                
                del self.sync_buffer[:4]
            else:
                break

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.read_data, daemon=True)
        self.thread.start()
    
    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("ECG port closed.")
        if self.trigger_ser and self.trigger_ser.is_open:
            self.trigger_ser.close()
            print("Trigger port closed.")

    def send_command(self, command):
        if not self.ser or not self.ser.is_open:
            print("Error: Serial not connected. Cannot send command.")
            return
        with self.app_state.mux_control_lock:
            try:
                cmd = (command + "\n").encode('utf-8')
                self.ser.write(cmd)
                print(f"Command sent: {command}")
                if command.startswith("STATE_"):
                    self.app_state.current_mux_state = int(command[-1])
                elif command == "START":
                    self.app_state.current_mux_state = -1 # Automatic mode
                elif command == "STOP":
                    self.app_state.current_mux_state = -2 # MUX stopped
            except Exception as e:
                print(f"Error sending command: {e}")
