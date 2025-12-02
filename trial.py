import threading
import time
# 1. IMPORT ROS 2 LIBRARIES
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# ... (VehicleState Class remains the same: The Safety Vault) ...

# ==========================================
# 2. THE ROS NODE (The "Shouter")
# ==========================================
class DataBridgeNode(Node):
    def __init__(self):
        super().__init__('can_data_bridge')
        # Create "Topics" (Channels) to broadcast data to the rest of the car
        self.pub_speed = self.create_publisher(Float32, '/car/speed', 10)
        self.pub_steer = self.create_publisher(Float32, '/car/steering', 10)

    def broadcast_data(self, speed, steering):
        # Package the data into a ROS message
        msg_speed = Float32()
        msg_speed.data = float(speed)
        
        msg_steer = Float32()
        msg_steer.data = float(steering)
        
        # Send it out to the ROS network
        self.pub_speed.publish(msg_speed)
        self.pub_steer.publish(msg_steer)

# ==========================================
# 3. THE HYBRID LISTENER (The Integration Point)
# ==========================================
def hybrid_listener(vehicle_state, ros_node):
    """
    This runs in the background. It reads CAN, updates the vault, 
    AND tells ROS what happened.
    """
    import random # Simulating CAN data
    
    while True:
        # A. READ CAN (The "Spinal Cord" action)
        # In real life, this comes from the CAN bus hardware
        raw_speed = random.uniform(40, 60)
        raw_steer = random.uniform(-5, 5)
        
        # B. UPDATE SAFETY VAULT (Critical!)
        # The main control loop needs this INSTANTLY
        vehicle_state.update_speed(raw_speed)
        vehicle_state.update_steering(raw_steer)
        
        # C. UPDATE ROS 2 (Integration!)
        # We tell the ROS node to broadcast this to the HMI/Planner
        ros_node.broadcast_data(raw_speed, raw_steer)
        
        time.sleep(0.01) # Simulate CAN speed (100Hz)

# ==========================================
# 4. MAIN EXECUTION
# ==========================================
def main():
    # Initialize ROS 2
    rclpy.init()
    
    # Create the ROS Node
    bridge_node = DataBridgeNode()
    
    # Create the Safety Vault
    shared_state = VehicleState()
    
    # Start the Listener Thread (passing it BOTH the vault and the ROS node)
    t = threading.Thread(target=hybrid_listener, args=(shared_state, bridge_node))
    t.start()
    
    print("System Running: Handling CAN control + Broadcasting to ROS 2...")
    
    # Keep the ROS node alive so it can publish
    rclpy.spin(bridge_node)

if __name__ == "__main__":
    main()