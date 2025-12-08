#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import math
import threading
import sys

class VisualNavigator(Node):
    def __init__(self):
        super().__init__('visual_navigator')

        # Declare parameters for frame IDs
        # Defaulting to 'aruco_1' (Robot) and 'aruco_0' (Target/Origin)
        self.declare_parameter('robot_frame', 'aruco_1')
        self.declare_parameter('target_frame', 'aruco_0')

        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        # Publisher for robot velocity
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Visual Navigator started. Target: {self.target_frame} from {self.robot_frame}')

        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

    def input_loop(self):
        """
        Thread to handle user input from terminal
        """
        print("\n--- Visual Navigator Interactive Mode ---")
        print(f"Current Target: {self.target_frame}")
        print("Enter new Target ID (number only) to change destination.")
        print("-----------------------------------------")
        
        while rclpy.ok():
            try:
                user_input = input("Enter Target ID > ")
                if user_input.strip().isdigit():
                    new_id = user_input.strip()
                    self.target_frame = f"aruco_{new_id}"
                    self.get_logger().info(f"New Target Set: {self.target_frame}")
                else:
                    print("Invalid input. Please enter a number.")
            except EOFError:
                break
            except Exception as e:
                print(f"Error reading input: {e}")

    def control_loop(self):
        try:
            # Look up transform from robot_frame TO target_frame
            t = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.target_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().warn(f'Waiting for transform {self.robot_frame} -> {self.target_frame}: {ex}', throttle_duration_sec=2.0)
            return

        msg = Twist()

        # 1. Get relative coordinates
        x = t.transform.translation.x
        y = t.transform.translation.y
        
        # 2. Calculate distance and angle
        # CORRECTION: The Aruco marker on the robot has X pointing Right and Y pointing Forward.
        # The Robot's base_footprint has X pointing Forward and Y pointing Left.
        # We need to map Marker coordinates to Robot coordinates to get the correct steering angle.
        # Marker Forward (0, Y) -> Robot Forward (X, 0) -> Angle 0
        # Marker Right (X, 0)   -> Robot Right (0, -Y)  -> Angle -90
        # Therefore, we use atan2(-x, y) instead of atan2(y, x)
        distance = math.sqrt(x**2 + y**2)
        angle_to_target = math.atan2(-x, y)

        # 3. Proportional Controller (P-Controller)
        K_linear = 0.5
        K_angular = 1.5 
        
        # Distance tolerance
        STOP_DISTANCE = 0.15
        # Alignment tolerance (radians) - ~3 degrees
        ALIGN_TOLERANCE = 0.05

        if distance > STOP_DISTANCE: 
            # Angular control: Turn towards target
            msg.angular.z = K_angular * angle_to_target
            
            # Linear control: STRICT Turn-then-Move
            # Only move forward if we are aligned with the target
            if abs(angle_to_target) < ALIGN_TOLERANCE:
                msg.linear.x = K_linear * distance
                msg.linear.x = min(msg.linear.x, 0.22) # Max speed for Turtlebot3
            else:
                # If not aligned, rotate in place
                msg.linear.x = 0.0
        else:
            # Target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            # Log prominent message
            self.get_logger().info(f'¡HE LLEGADO A LA META! ({self.target_frame})', throttle_duration_sec=2.0)

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisualNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
