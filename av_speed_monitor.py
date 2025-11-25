import threading
import time
import random

# Thread-safe storage of decode CAN values
class VehicleState:
    def __init__(self):
        self._current_speed = 0.0  # Buffer of Size 1 to store speed value
        self._lock = threading.Lock()  
        self._last_updated = time.time()

    def update_speed(self, raw_can_value):
        """
        WRITER: Called by the CAN Listener (Producer).
        Decodes data and updates storage immediately.
        """
        # DECODING LOGIC: raw value might be an integer 0-255, so use 
        # conversion like 'speed = raw * 0.621' (fake conversion factor, just a placeholder!)
        decoded_speed = raw_can_value * 0.621 
        
        # MUTEX LOCKING while writing
        with self._lock:
            self._current_speed = decoded_speed
            self._last_updated = time.time()

    def get_latest_speed(self):
        # MUTEX LOCKING while reading
        with self._lock:
            speed_snapshot = self._current_speed
            timestamp = self._last_updated
        
        return speed_snapshot, timestamp