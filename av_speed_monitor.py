class VehicleState:
    def __init__(self):
        # DATA
        self._speed = 0.0          
        self._steering = 0.0       
        self._acceleration = 0.0   
        
        # TIMESTAMPS (To detect stale data)
        self._speed_ts = 0.0
        self._steering_ts = 0.0
        self._accel_ts = 0.0
        
        self._lock = threading.Lock() 

    # --- WRITER FUNCTIONS ---
    def update_speed(self, raw_value):
        decoded = raw_value * 0.621
        with self._lock:
            self._speed = decoded
            self._speed_ts = time.time()  # Track exact moment data arrived

    def update_steering(self, raw_value):
        decoded = raw_value - 500.0 
        with self._lock:
            self._steering = decoded
            self._steering_ts = time.time()

    def update_acceleration(self, raw_value):
        decoded = raw_value * 0.039 
        with self._lock:
            self._acceleration = decoded
            self._accel_ts = time.time()

    # --- READER FUNCTION ---
    def get_full_state(self):
        with self._lock:
            now = time.time()
            state_snapshot = {
                "speed": self._speed,
                "speed_age": self._speed_ts, # How old is this data?
                
                "steering": self._steering,
                "steering_age": self._steering_ts,
                
                "acceleration": self._acceleration,
                "accel_age": self._accel_ts
            }
        return state_snapshot

# 2) Listener (DAEMON)
def can_bus_callback(vehicle_state):


# 3) CONTROL LOOP
def main_control_loop():

    shared_state = VehicleState()

    # Starting Listener Daemon. Create new thread
    listener_thread = threading.Thread(
        target=can_bus_callback, 
        args=(shared_state,), 
        daemon=True
    )
    listener_thread.start()

    print("[AV CONTROLLER] System initialized. Starting main loop...")

    # MAIN LOOP (Placeholder for CAN detecting logic)
    try:
            
    except:

if __name__ == "__main__":
    main_control_loop()