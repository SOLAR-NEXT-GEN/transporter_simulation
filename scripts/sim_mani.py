#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool
from std_srvs.srv import SetBool
import time


class ManiSim(Node):
    def __init__(self):
        super().__init__('mani_sim')
        
        self.declare_parameter('animation_step', 0.05)
        self.declare_parameter('animation_rate', 20.0)
        
        self.animation_step = self.get_parameter('animation_step').get_parameter_value().double_value
        animation_rate = self.get_parameter('animation_rate').get_parameter_value().double_value
        
        self.LEFT_HINGE_UP = 0.0
        self.LEFT_HINGE_DOWN = -2.07
        self.RIGHT_HINGE_UP = 0.0
        self.RIGHT_HINGE_DOWN = 2.07
        
        self.left_hinge_pos = 0.0
        self.right_hinge_pos = 0.0
        
        self.animating = False
        self.animation_target_left = 0.0
        self.animation_target_right = 0.0
        self.animation_type = ""
        
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
        
        self.animation_status_pub = self.create_publisher(
            String,
            '/hinge_animation_status',
            10
        )
        
        self.animation_complete_pub = self.create_publisher(
            Bool,
            '/hinge_animation_complete',
            10
        )
        
        self.hinges_down_service = self.create_service(
            SetBool,
            'set_hinges_down',
            self.set_hinges_down_callback
        )
        
        self.hinges_up_service = self.create_service(
            SetBool,
            'set_hinges_up', 
            self.set_hinges_up_callback
        )
        
        self.animation_timer = self.create_timer(
            1.0 / animation_rate,
            self.animation_step_callback
        )
        
        self.left_hinge_msg = Float64MultiArray()
        self.right_hinge_msg = Float64MultiArray()
        
        self.get_logger().info('Mani Sim node initialized')
        
    def publish_hinge_positions(self):
        self.left_hinge_msg.data = [self.left_hinge_pos]
        self.left_hinge_pub.publish(self.left_hinge_msg)
        
        self.right_hinge_msg.data = [self.right_hinge_pos]
        self.right_hinge_pub.publish(self.right_hinge_msg)
    
    def start_animation(self, target_left, target_right, animation_type):
        if self.animating:
            return False
        
        self.animation_target_left = target_left
        self.animation_target_right = target_right
        self.animation_type = animation_type
        self.animating = True
        
        status_msg = String()
        status_msg.data = f"ANIMATING_{animation_type}"
        self.animation_status_pub.publish(status_msg)
        
        self.get_logger().info(f'Starting {animation_type} animation')
        
        return True
    
    def wait_for_animation_complete(self):
        """Wait for animation to complete (blocking)"""
        while self.animating:
            time.sleep(0.01)  # 10ms sleep
    
    def animation_step_callback(self):
        if not self.animating:
            return
        
        left_diff = self.animation_target_left - self.left_hinge_pos
        if abs(left_diff) < self.animation_step:
            self.left_hinge_pos = self.animation_target_left
        else:
            self.left_hinge_pos += self.animation_step if left_diff > 0 else -self.animation_step
        
        right_diff = self.animation_target_right - self.right_hinge_pos
        if abs(right_diff) < self.animation_step:
            self.right_hinge_pos = self.animation_target_right
        else:
            self.right_hinge_pos += self.animation_step if right_diff > 0 else -self.animation_step
        
        self.publish_hinge_positions()
        
        left_complete = abs(self.left_hinge_pos - self.animation_target_left) < 0.01
        right_complete = abs(self.right_hinge_pos - self.animation_target_right) < 0.01
        
        if left_complete and right_complete:
            self.finish_animation()
    
    def finish_animation(self):
        self.animating = False
        
        self.left_hinge_pos = self.animation_target_left
        self.right_hinge_pos = self.animation_target_right
        self.publish_hinge_positions()
        
        status_msg = String()
        status_msg.data = f"COMPLETE_{self.animation_type}"
        self.animation_status_pub.publish(status_msg)
        
        complete_msg = Bool()
        complete_msg.data = True
        self.animation_complete_pub.publish(complete_msg)
        
        self.get_logger().info(f'{self.animation_type} animation completed')
    
    def execute_animation_blocking(self, target_left, target_right, animation_type):
        if self.animating:
            return False, 'Animation already in progress'
        
        self.animation_target_left = target_left
        self.animation_target_right = target_right
        self.animation_type = animation_type
        
        status_msg = String()
        status_msg.data = f"ANIMATING_{animation_type}"
        self.animation_status_pub.publish(status_msg)
        
        self.get_logger().info(f'Starting {animation_type} animation')
        
        while True:
            left_diff = self.animation_target_left - self.left_hinge_pos
            if abs(left_diff) < self.animation_step:
                self.left_hinge_pos = self.animation_target_left
            else:
                self.left_hinge_pos += self.animation_step if left_diff > 0 else -self.animation_step
            
            right_diff = self.animation_target_right - self.right_hinge_pos
            if abs(right_diff) < self.animation_step:
                self.right_hinge_pos = self.animation_target_right
            else:
                self.right_hinge_pos += self.animation_step if right_diff > 0 else -self.animation_step
            
            self.publish_hinge_positions()
            
            left_complete = abs(self.left_hinge_pos - self.animation_target_left) < 0.01
            right_complete = abs(self.right_hinge_pos - self.animation_target_right) < 0.01
            
            if left_complete and right_complete:
                break
            
            time.sleep(1.0 / 20.0)
        
        self.left_hinge_pos = self.animation_target_left
        self.right_hinge_pos = self.animation_target_right
        self.publish_hinge_positions()
        
        status_msg = String()
        status_msg.data = f"COMPLETE_{animation_type}"
        self.animation_status_pub.publish(status_msg)
        
        complete_msg = Bool()
        complete_msg.data = True
        self.animation_complete_pub.publish(complete_msg)
        
        self.get_logger().info(f'{animation_type} animation completed')
        
        return True, f'{animation_type} animation completed successfully'
    
    def set_hinges_down_callback(self, request, response):
        try:
            success, message = self.execute_animation_blocking(
                self.LEFT_HINGE_DOWN,
                self.RIGHT_HINGE_DOWN,
                "DOWN"
            )
            response.success = success
            response.message = message
        except Exception as e:
            self.get_logger().error(f'Error in DOWN animation: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def set_hinges_up_callback(self, request, response):
        try:
            success, message = self.execute_animation_blocking(
                self.LEFT_HINGE_UP,
                self.RIGHT_HINGE_UP,
                "UP"
            )
            response.success = success
            response.message = message
        except Exception as e:
            self.get_logger().error(f'Error in UP animation: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ManiSim()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Mani Sim...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()