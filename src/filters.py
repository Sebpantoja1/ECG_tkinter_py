"""
ECG signal processing filters.
"""
from scipy import signal
from . import config

class ECGFilters:
    """
    Manages the filtering of ECG signals.
    
    Includes a notch filter for powerline interference and a bandpass filter.
    """
    def __init__(self, fs=config.SAMPLE_RATE):
        self.fs = fs
        self.setup_filters()
        self.reset_states()

    def setup_filters(self):
        """Initializes the filter coefficients."""
        nyquist = self.fs / 2
        # Notch filter for 60Hz powerline interference
        self.notch_b, self.notch_a = signal.iirnotch(60, 20, self.fs)
        
        # Bandpass filter (0.05Hz - 40Hz)
        low_cutoff = 0.05 / nyquist
        high_cutoff = 40 / nyquist
        self.bp_b, self.bp_a = signal.butter(2, [low_cutoff, high_cutoff], btype='band')

    def reset_states(self):
        """Resets the initial conditions of the filters."""
        self.notch_zi = signal.lfilter_zi(self.notch_b, self.notch_a)
        self.bp_zi = signal.lfilter_zi(self.bp_b, self.bp_a)
        
        # Set initial condition to mid-point of ADC range (1.65V)
        # to reduce initial transient.
        initial_condition = 1.65 
        self.notch_zi *= initial_condition
        self.bp_zi *= initial_condition

    def process_sample(self, sample):
        """Processes a single data sample through the filters."""
        if not config.ENABLE_FILTERS:
            return sample
        
        filtered_sample, self.notch_zi = signal.lfilter(self.notch_b, self.notch_a, [sample], zi=self.notch_zi)
        filtered_sample, self.bp_zi = signal.lfilter(self.bp_b, self.bp_a, filtered_sample, zi=self.bp_zi)
        
        return filtered_sample[0]
