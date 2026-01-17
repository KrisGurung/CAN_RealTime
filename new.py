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

# LISTENER FUNCTION: Used for callback when detecting requested/required CAN value
def can_bus_callback(vehicle_state):
    """
    Simulates the CAN hardware waking up your code.
    In real life, the CAN library calls this function automatically.
    """
    print("[CAN HARDWARE] Listener started in background...")
    
    while True:
        # SIMULATION: Wait random time between messages (10ms - 50ms)
        time.sleep(random.uniform(0.01, 0.05))
        
        # SIMULATION: Receive random raw data (0 to 100)
        raw_data = random.randint(0, 100)
        
        # TRIGGER: Hardware calls the update function
        # This runs in a separate thread from the main loop!
        vehicle_state.update_speed(raw_data)

# ==========================================
# 3. THE "CONTROLLER" (Main Loop)
# ==========================================
def main_control_loop():
    # Initialize the shared memory
    shared_state = VehicleState()

    # Start the Listener in a background thread
    # (Daemon means it dies when the main program dies)
    listener_thread = threading.Thread(
        target=can_bus_callback, 
        args=(shared_state,), 
        daemon=True
    )
    listener_thread.start()

    print("[AV CONTROLLER] System initialized. Starting main loop...")

    # The Main Loop (Simulating the car driving)
    try:
        for i in range(10):
            # 1. Do complex AV calculations (Simulated work)
            time.sleep(0.5) 
            
            # 2. FETCH DATA: "I need the speed NOW"
            current_speed, last_time = shared_state.get_latest_speed()
            
            # Calculate how "fresh" the data is
            latency = (time.time() - last_time) * 1000 
            
            print(f"Loop {i+1}: AV Logic used Speed: {current_speed:.2f} mph "
                  f"(Data age: {latency:.1f}ms)")
            
    except KeyboardInterrupt:
        print("Stopping...")

if __name__ == "__main__":
    main_control_loop()