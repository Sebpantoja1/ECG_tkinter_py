"""
Functions for R-peak detection in ECG signals.
"""
import numpy as np

def detect_r_peaks(signal_data, threshold, distance):
    """
    Detects R-peaks in a signal based on a simple threshold and distance criteria.

    Args:
        signal_data (list or np.array): The ECG signal.
        threshold (float): The minimum amplitude for a peak.
        distance (int): The minimum number of samples between peaks.

    Returns:
        list: A list of indices where R-peaks are detected.
    """
    if len(signal_data) < 3:
        return []
        
    peaks = []
    last_peak = -distance
    
    # Simple peak detection: a point is a peak if it's greater than its neighbours
    for i in range(1, len(signal_data) - 1):
        if (signal_data[i] > threshold and
            signal_data[i] > signal_data[i-1] and
            signal_data[i] > signal_data[i+1]):
            
            if i - last_peak >= distance:
                peaks.append(i)
                last_peak = i
                
    return peaks

def calculate_bpm(peaks, sample_rate):
    """
    Calculates Beats Per Minute (BPM) from a list of peak indices.

    Args:
        peaks (list): Indices of the detected peaks.
        sample_rate (int): The sampling rate of the signal in Hz.

    Returns:
        float: The calculated BPM, or 0 if not enough peaks are found.
    """
    if len(peaks) < 2:
        return 0
    
    avg_interval = np.mean(np.diff(peaks))
    if avg_interval > 0:
        bpm = 60 / (avg_interval / sample_rate)
        return bpm
    
    return 0
