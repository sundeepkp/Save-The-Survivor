import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import random

# --- Global GPIO State (Module Level Error Handling) ---
# This determines if the necessary library is even installed
try:
    import gpiod
    # RPi 5 GPIO chip is typically chip 0
    CHIP = gpiod.Chip('gpiochip0') 
    MODULE_AVAILABLE = True # Rename to MODULE_AVAILABLE to avoid internal conflict
except ImportError:
    CHIP = None
    MODULE_AVAILABLE = False
except Exception as e:
    print(f"GPIOD Chip access error at module level: {e}")
    CHIP = None
    MODULE_AVAILABLE = False


class IRSensorTestNode(Node):
    """ROS 2 Node for reading a digital IR sensor state using libgpiod."""
    
    def __init__(self):
        super().__init__('ir_sensor_test_node')
        
        # --- Configuration Parameters ---
        self.declare_parameter('ir_pin', 17)
        self.ir_pin = self.get_parameter('ir_pin').get_parameter_value().integer_value
        self.timer_period = 0.1
        
        # --- Hardware State Tracking (Instance variables replace 'global') ---
        self.sensor_line = None
        self.is_hardware_ready = False # Tracks if GPIO setup succeeded
        
        # --- libgpiod Setup ---
        if MODULE_AVAILABLE:
            try:
                # Request the line using BCM pin number (offset)
                self.sensor_line = CHIP.get_line(self.ir_pin)
                # Set as input, tell kernel who is requesting the line
                self.sensor_line.request(consumer="ir_sensor_node", type=gpiod.LINE_REQ_DIR_IN)
                
                self.is_hardware_ready = True # Setup succeeded
                self.get_logger().info(f'Real GPIO setup complete on BCM {self.ir_pin} using libgpiod.')
            except Exception as e:
                # FIX: Simply log the error and let is_hardware_ready remain False
                self.get_logger().error(f"GPIOD Line setup failed on BCM {self.ir_pin}: {e}")
                self.is_hardware_ready = False # Setup failed

        # --- ROS 2 Setup ---
        self.publisher_ = self.create_publisher(String, 'ir_sensor_status', 10)
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info(f'IR Sensor Node starting. Monitoring BCM Pin {self.ir_pin}...')

        if not self.is_hardware_ready:
            self.get_logger().warn('Hardware access failed. Publishing simulated data.')
        

    def read_ir_sensor(self):
        """Reads the digital value from the IR sensor pin."""
        if self.is_hardware_ready:
            # libgpiod read operation: get_value() returns 0 or 1
            return self.sensor_line.get_value()

        # Mocking logic (only executes if setup failed)
        if int(time.time() / 5) % 2 == 0:
            return random.choice([0, 1])
        else:
            return 1
            
    def timer_callback(self):
        """Callback executed by the timer to read the sensor and publish the state."""
        sensor_value = self.read_ir_sensor()
        
        # Logic check: Assuming 0 (LOW) is the active state when LED blinks
        if sensor_value == 0:
            status = 'OBJECT DETECTED'
        else:
            status = 'CLEAR'
        
        msg = String()
        msg.data = f'Pin {self.ir_pin}: {status}'
        
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Publishing: "{msg.data}"')

    def destroy_node(self):
        """Cleanup GPIO resources when the node is shut down."""
        if self.sensor_line is not None:
            self.sensor_line.release()
            self.get_logger().info('GPIO line released.')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    ir_sensor_node = IRSensorTestNode()
    
    try:
        rclpy.spin(ir_sensor_node)
    except KeyboardInterrupt:
        ir_sensor_node.get_logger().info('Node stopped cleanly')
    finally:
        ir_sensor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
