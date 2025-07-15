#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class DiffJoyControl(Node):
    def __init__(self):
        super().__init__('diff_joy_control')
        
        # Constants for velocities
        self.LINEAR_VEL = 2.0  # m/s
        self.ANGULAR_VEL = 1.5  # rad/s
        
        # Create publisher for cmd_vel topic
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscriber for joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Initialize Twist message
        self.twist_msg = Twist()
        
        self.get_logger().info('Transporter Joy node initialized')
        self.get_logger().info(f'Linear velocity: {self.LINEAR_VEL} m/s')
        self.get_logger().info(f'Angular velocity: {self.ANGULAR_VEL} rad/s')
        self.get_logger().info('Listening for joystick input on /joy topic...')
    
    def joy_callback(self, msg):
        """
        Process joystick input and publish Twist messages
        
        Axis mapping:
        - axis[6]: Angular velocity (Wz)
            > 0: +Wz (turn left)
            < 0: -Wz (turn right)
        - axis[7]: Linear velocity (Vx)
            > 0: +Vx (forward)
            < 0: -Vx (backward)
        """
        
        # Reset twist message
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        
        # Check if we have enough axes
        if len(msg.axes) > 7:
            # Process linear velocity (axis 7)
            if msg.axes[7] > 0:
                self.twist_msg.linear.x = self.LINEAR_VEL
            elif msg.axes[7] < 0:
                self.twist_msg.linear.x = -self.LINEAR_VEL
            
            # Process angular velocity (axis 6)
            if msg.axes[6] > 0:
                self.twist_msg.angular.z = self.ANGULAR_VEL
            elif msg.axes[6] < 0:
                self.twist_msg.angular.z = -self.ANGULAR_VEL
            
            # Publish the twist message
            self.twist_pub.publish(self.twist_msg)
            
            # Log current velocities (optional - comment out if too verbose)
            if self.twist_msg.linear.x != 0.0 or self.twist_msg.angular.z != 0.0:
                self.get_logger().debug(
                    f'Publishing: Vx={self.twist_msg.linear.x:.4f} m/s, '
                    f'Wz={self.twist_msg.angular.z:.4f} rad/s'
                )
        else:
            self.get_logger().warn(
                f'Not enough axes in Joy message. Expected at least 8, got {len(msg.axes)}'
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