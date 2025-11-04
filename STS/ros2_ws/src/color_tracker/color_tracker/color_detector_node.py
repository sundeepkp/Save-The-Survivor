import rclpy
from rclpy.node import Node
# We need String for the pickup status message
from std_msgs.msg import String 
import cv2
import numpy as np
# Note: geometry_msgs.msg.Point and CvBridge are not needed for this simplified status logic

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        
        # --- Publishers and Timers ---
        self.status_pub = self.create_publisher(String, '/robot_status', 10) 
        self.timer = self.create_timer(0.1, self.camera_callback) # Run at 10 Hz 

        # 1. Initialize the Webcam using OpenCV
        self.cap = cv2.VideoCapture(0)
        
        # --- CRITICAL FIX: Check camera status IMMEDIATELY ---
        self.is_camera_open = self.cap.isOpened()
        
        if not self.is_camera_open:
            self.get_logger().error("FATAL: Failed to open camera /dev/video0! Check permissions and device mapping.")
            self.FRAME_AREA = 1 # Set minimal area if camera fails
        else:
            # 2. Get Frame Dimensions and Area (Only if camera is open)
            # Read a dummy frame to initialize dimensions
            ret, frame = self.cap.read()
            if ret:
                self.FRAME_AREA = frame.shape[0] * frame.shape[1]
                self.get_logger().info(f"Camera opened. Frame area: {self.FRAME_AREA} pixels.")
            else:
                self.get_logger().warn("Could not read frame for dimension setup. Using default area.")
                self.FRAME_AREA = 320 * 240 # Safe fallback
            
        # 3. Define the detection threshold
        self.AREA_THRESHOLD = 0.08  # 8% of the frame area for pickup trigger

        # 4. Define HSV Ranges
        self.L_RED1 = np.array([0, 100, 100])
        self.U_RED1 = np.array([10, 255, 255])
        self.L_RED2 = np.array([160, 100, 100])
        self.U_RED2 = np.array([179, 255, 255])
        self.L_GREEN = np.array([35, 100, 100])
        self.U_GREEN = np.array([85, 255, 255])
        self.L_BLUE = np.array([100, 100, 100])
        self.U_BLUE = np.array([140, 255, 255])
        self.color_map = {
            1: (self.L_RED1, self.U_RED1, self.L_RED2, self.U_RED2, "RED"), 
            2: (self.L_GREEN, self.U_GREEN, None, None, "GREEN"),
            3: (self.L_BLUE, self.U_BLUE, None, None, "BLUE")
        }


    def detect_and_publish_pickup(self, frame, hsv_frame):
        
        status_msg = String()
        
        for color_id, (l_hsv1, u_hsv1, l_hsv2, u_hsv2, label) in self.color_map.items():
            
            # 1. Create mask for the current color
            mask = cv2.inRange(hsv_frame, l_hsv1, u_hsv1)
            if l_hsv2 is not None:
                mask2 = cv2.inRange(hsv_frame, l_hsv2, u_hsv2)
                mask = mask + mask2 

            # 2. Find the largest contour
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area_pixels = cv2.contourArea(largest_contour)

                # 3. Calculate Percentage Area
                percentage = area_pixels / self.FRAME_AREA
                
                # 4. Check Pickup Threshold
                if percentage >= self.AREA_THRESHOLD:
                    
                    status_text = f"PICKUP {label}"
                    status_msg.data = status_text
                    self.status_pub.publish(status_msg)
                    
                    self.get_logger().info(f"STATUS TRIGGER: {status_text} at {percentage:.2f} coverage.")
                    
                    # Optional: Mark the detected object
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        center_x = int(M["m10"] / M["m00"])
                        center_y = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (center_x, center_y), 10, (0, 255, 0), 2)
                        
                    # Stop searching once a target is found and ready for pickup
                    return

        # If no color meets the threshold, publish a default status
        status_msg.data = "IDLE/SEARCHING"
        self.status_pub.publish(status_msg)


    def camera_callback(self):
        # Only proceed if the camera is actually open
        if not self.is_camera_open:
            return 
            
        ret, frame = self.cap.read()

        if ret:
            # Convert BGR to HSV
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            self.detect_and_publish_pickup(frame, hsv_frame) 
            
            # Display the frame 
            cv2.imshow("Detection Frame", frame)
            cv2.waitKey(1)
            
# --- MAIN FUNCTION FOR CLEAN EXIT ---
def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    try:
        rclpy.spin(color_detector)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        color_detector.get_logger().error(f"Node execution failed: {e}")
        return 1 
    finally:
        # Shutdown gracefully
        if color_detector.is_camera_open:
             color_detector.cap.release()
             cv2.destroyAllWindows()
             
        color_detector.destroy_node()
        rclpy.shutdown()
        
    return 0

if __name__ == '__main__':
    exit_status = main()
    if exit_status is not None:
        exit(exit_status)
