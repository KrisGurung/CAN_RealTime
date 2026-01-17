import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from av_speed_monitor import VehicleState

# ==========================================
# 1. THE ROS NODE 
# ==========================================
class DataBridgeNode(Node):
    def __init__(self):
        super().__init__('can_data_bridge')
        self.pub_speed = self.create_publisher(Float32, '/car/speed', 10)
        self.pub_steer = self.create_publisher(Float32, '/car/steering', 10)

    def broadcast_data(self, speed, steering):
        msg_speed = Float32()
        msg_speed.data = float(speed)
        
        msg_steer = Float32()
        msg_steer.data = float(steering)
        
        self.pub_speed.publish(msg_speed)
        self.pub_steer.publish(msg_steer)

# ==========================================
# 2. THREAD A: THE SIMULATOR (Writer)
# ==========================================
def can_bus_simulator(vehicle_state):
    """
    Simulates the CAN bus hardware. 
    It generates RAW data and updates the vault.
    It does NOT talk to ROS.
    """
    import random
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

# ==========================================
# 3. THREAD B: THE BRIDGE (Reader)
# ==========================================
def ros_bridge_listener(vehicle_state, ros_node):
    """
    Replaces the 'hybrid_listener'.
    It READS the safe state from the vault and broadcasts to ROS.
    """
    while True:
        # READ from the Vault (Thread-Safe)
        # This gets the dictionary with timestamps: {'speed': X, 'speed_age': Y, ...}
        current_state = vehicle_state.get_full_state()
        
        # Extract the DECODED values (Safe to use)
        safe_speed = current_state["speed"]
        safe_steer = current_state["steering"]
        
        # Broadcast to the car's software stack
        ros_node.broadcast_data(safe_speed, safe_steer)
        
        # We can publish slightly slower than the hardware (e.g., 50Hz)
        # This 'decouples' the software from the hardware speed
        time.sleep(0.02) 

# ==========================================
# 4. MAIN EXECUTION
# ==========================================
def main():
    rclpy.init()
    
    # 1. Create the ROS Node
    bridge_node = DataBridgeNode()
    
    # 2. Create the Shared Safety Vault
    shared_state = VehicleState()
    
    # 3. Start Thread A (The Hardware Simulator)
    t_sim = threading.Thread(
        target=can_bus_simulator, 
        args=(shared_state,), 
        daemon=True
    )
    t_sim.start()
    
    # 4. Start Thread B (The ROS Bridge)
    t_bridge = threading.Thread(
        target=ros_bridge_listener, 
        args=(shared_state, bridge_node), 
        daemon=True
    )
    t_bridge.start()
    
    print("System Running: Decoupled Writer (CAN) and Reader (ROS)...")
    
    # 5. Keep the ROS node alive
    rclpy.spin(bridge_node)

if __name__ == "__main__":
    main()