#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class DiffJoyControl(Node):
    def __init__(self):
        super().__init__('diff_joy_control')
        
        self.declare_parameter('linear_velocity', 2.0)
        self.declare_parameter('angular_velocity', 1.5)
        
        self.linear_vel = self.get_parameter('linear_velocity').get_parameter_value().double_value
        self.angular_vel = self.get_parameter('angular_velocity').get_parameter_value().double_value
        
        self.hinge_animation_in_progress = False
        
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.hinges_up_client = self.create_client(SetBool, 'set_hinges_up')
        self.hinges_down_client = self.create_client(SetBool, 'set_hinges_down')
        
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        self.twist_msg = Twist()
        
        self.prev_button_3 = False
        self.prev_button_1 = False
        
        self.wait_for_services()
        
        self.get_logger().info('Differential Drive Joy Control initialized')
        
    def wait_for_services(self):
        while not self.hinges_up_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_hinges_up service...')
        
        while not self.hinges_down_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_hinges_down service...')
        
        self.get_logger().info('All hinge control services available')
    
    def hinges_up_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Hinges UP completed: {response.message}')
            else:
                self.get_logger().warn(f'Hinges UP failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Hinges UP service call failed: {str(e)}')
        finally:
            self.hinge_animation_in_progress = False
    
    def hinges_down_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Hinges DOWN completed: {response.message}')
            else:
                self.get_logger().warn(f'Hinges DOWN failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Hinges DOWN service call failed: {str(e)}')
        finally:
            self.hinge_animation_in_progress = False
    
    def call_hinges_up_service(self):
        if self.hinge_animation_in_progress:
            self.get_logger().info('Hinge animation in progress, ignoring UP request')
            return
        
        request = SetBool.Request()
        request.data = True
        
        self.hinge_animation_in_progress = True
        self.get_logger().info('Calling hinges UP service...')
        
        future = self.hinges_up_client.call_async(request)
        future.add_done_callback(self.hinges_up_response_callback)
    
    def call_hinges_down_service(self):
        if self.hinge_animation_in_progress:
            self.get_logger().info('Hinge animation in progress, ignoring DOWN request')
            return
        
        request = SetBool.Request()
        request.data = True
        
        self.hinge_animation_in_progress = True
        self.get_logger().info('Calling hinges DOWN service...')
        
        future = self.hinges_down_client.call_async(request)
        future.add_done_callback(self.hinges_down_response_callback)
    
    def joy_callback(self, msg):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        
        if len(msg.axes) > 3:
            if abs(msg.axes[1]) > 0.1:
                self.twist_msg.linear.x = msg.axes[1] * self.linear_vel
            
            if abs(msg.axes[3]) > 0.1:
                self.twist_msg.angular.z = msg.axes[3] * self.angular_vel
            
            self.twist_pub.publish(self.twist_msg)
        else:
            self.get_logger().warn(f'Not enough axes in Joy message. Expected at least 4, got {len(msg.axes)}')
        
        if len(msg.buttons) > 3:
            if msg.buttons[3] and not self.prev_button_3:
                self.call_hinges_up_service()
            
            if msg.buttons[1] and not self.prev_button_1:
                self.call_hinges_down_service()
            
            self.prev_button_3 = msg.buttons[3]
            self.prev_button_1 = msg.buttons[1]
        else:
            self.get_logger().warn(f'Not enough buttons in Joy message. Expected at least 4, got {len(msg.buttons)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = DiffJoyControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Joy controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()