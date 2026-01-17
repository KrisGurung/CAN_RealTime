import time
import random

def can_bus_simulator(vehicle_state):
    """
    Simulates the CAN bus hardware. 
    It generates RAW data and updates the vault.
    It does NOT talk to ROS.
    """
    print("  -> [CAN SIM] Simulator Thread Started...")
    while True:
        # Simulate RAW data (e.g., sensor voltages or integers)
        # Note: av_speed_monitor will multiply this by 0.621 to get MPH
        raw_speed_input = random.uniform(40, 60) 
        
        # Note: av_speed_monitor will subtract 500 from this
        raw_steer_input = random.uniform(495, 505) 
        
        # WRITE to the Vault (Thread-Safe)
        vehicle_state.update_speed(raw_speed_input)
        vehicle_state.update_steering(raw_steer_input)
        
        # Simulate hardware running at 100Hz
        time.sleep(0.01)