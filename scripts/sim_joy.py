#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class DiffJoyControl(Node):
    def __init__(self):
        super().__init__('diff_joy_control')
        
        # Constants for velocities
        self.LINEAR_VEL = 2.0  # m/s
        self.ANGULAR_VEL = 1.5  # rad/s
        
        # Hinge position limits and step size
        self.LEFT_HINGE_MIN = 0.0      # Lower limit for left hinge
        self.LEFT_HINGE_MAX = -2.117   # Upper limit for left hinge
        self.RIGHT_HINGE_MIN = 0.0     # Lower limit for right hinge
        self.RIGHT_HINGE_MAX = 2.117   # Upper limit for right hinge
        self.HINGE_STEP = 0.1          # Step size for hinge movement
        
        # Current hinge positions
        self.left_hinge_pos = 0.0
        self.right_hinge_pos = 0.0
        
        # Create publisher for cmd_vel topic
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create publishers for hinge controllers
        self.left_hinge_pub = self.create_publisher(
            Float64MultiArray, 
            '/left_hinge_controller/commands', 
            10
        )
        self.right_hinge_pub = self.create_publisher(
            Float64MultiArray, 
            '/right_hinge_controller/commands', 
            10
        )
        
        # Create subscriber for joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Initialize messages
        self.twist_msg = Twist()
        self.left_hinge_msg = Float64MultiArray()
        self.right_hinge_msg = Float64MultiArray()
        
        # Button press tracking to prevent continuous triggering
        self.prev_button_2 = False
        self.prev_button_0 = False
        
        self.get_logger().info('Transporter Joy node initialized')
        self.get_logger().info(f'Linear velocity: {self.LINEAR_VEL} m/s')
        self.get_logger().info(f'Angular velocity: {self.ANGULAR_VEL} rad/s')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left stick Y (axis[1]): Forward/Backward')
        self.get_logger().info('  Right stick X (axis[3]): Left/Right turn')
        self.get_logger().info('  Button[2]: Raise hinges to top position')
        self.get_logger().info('  Button[0]: Lower hinges to bottom position')
        self.get_logger().info('Listening for joystick input on /joy topic...')
    
    def clamp_hinge_position(self, position, min_val, max_val):
        """Clamp hinge position within limits"""
        return max(min_val, min(max_val, position))
    
    def publish_hinge_positions(self):
        """Publish current hinge positions"""
        # Left hinge command
        self.left_hinge_msg.data = [self.left_hinge_pos]
        self.left_hinge_pub.publish(self.left_hinge_msg)
        
        # Right hinge command
        self.right_hinge_msg.data = [self.right_hinge_pos]
        self.right_hinge_pub.publish(self.right_hinge_msg)
        
        # self.get_logger().info(
        #     f'Hinge positions - Left: {self.left_hinge_pos:.3f}, Right: {self.right_hinge_pos:.3f}'
        # )
    
    def joy_callback(self, msg):
        """
        Process joystick input and publish Twist and hinge commands
        
        Axis mapping:
        - axis[1]: Linear velocity (Vx)
            > 0: +Vx (forward)
            < 0: -Vx (backward)
        - axis[3]: Angular velocity (Wz)
            > 0: +Wz (turn left)
            < 0: -Wz (turn right)
            
        Button mapping:
        - button[2]: Raise both hinges to top position
        - button[0]: Lower both hinges to bottom position
        """
        
        # Reset twist message
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        
        # Check if we have enough axes
        if len(msg.axes) > 3:
            # Process linear velocity (axis 1)
            if abs(msg.axes[1]) > 0.1:  # Dead zone
                self.twist_msg.linear.x = msg.axes[1] * self.LINEAR_VEL
            
            # Process angular velocity (axis 3)
            if abs(msg.axes[3]) > 0.1:  # Dead zone
                self.twist_msg.angular.z = msg.axes[3] * self.ANGULAR_VEL
            
            # Publish the twist message
            self.twist_pub.publish(self.twist_msg)
            
            # Log current velocities (optional - comment out if too verbose)
            if abs(self.twist_msg.linear.x) > 0.01 or abs(self.twist_msg.angular.z) > 0.01:
                self.get_logger().debug(
                    f'Publishing: Vx={self.twist_msg.linear.x:.4f} m/s, '
                    f'Wz={self.twist_msg.angular.z:.4f} rad/s'
                )
        else:
            self.get_logger().warn(
                f'Not enough axes in Joy message. Expected at least 4, got {len(msg.axes)}'
            )
        
        # Handle hinge control buttons
        if len(msg.buttons) > 2:
            # Button 2: Raise hinges to top position
            if msg.buttons[2] and not self.prev_button_2:
                # self.get_logger().info('Button 2 pressed - Raising hinges to top position')
                self.left_hinge_pos = self.LEFT_HINGE_MIN   # 0.0 (top position for left)
                self.right_hinge_pos = self.RIGHT_HINGE_MIN # 2.217 (top position for right)
                self.publish_hinge_positions()
            
            # Button 0: Lower hinges to bottom position  
            if msg.buttons[0] and not self.prev_button_0:
                # self.get_logger().info('Button 0 pressed - Lowering hinges to bottom position')
                self.left_hinge_pos = self.LEFT_HINGE_MAX   # -2.217 (bottom position for left)
                self.right_hinge_pos = self.RIGHT_HINGE_MAX # 0.0 (bottom position for right)
                self.publish_hinge_positions()
            
            # Update previous button states
            self.prev_button_2 = msg.buttons[2]
            self.prev_button_0 = msg.buttons[0]
        else:
            self.get_logger().warn(
                f'Not enough buttons in Joy message. Expected at least 3, got {len(msg.buttons)}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DiffJoyControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()