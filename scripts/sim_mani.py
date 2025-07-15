#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String, Bool
from std_srvs.srv import SetBool
import math

class ManiSim(Node):
    def __init__(self):
        super().__init__('mani_sim')
        
        # Hinge angle limits and positions
        self.LEFT_HINGE_UP = 0.0      # Up position
        self.LEFT_HINGE_DOWN = -2.07  # Down position
        self.RIGHT_HINGE_UP = 0.0     # Up position  
        self.RIGHT_HINGE_DOWN = 2.07  # Down position
        
        # Animation parameters
        self.ANIMATION_STEP = 0.05    # Step size in radians
        self.ANIMATION_RATE = 20      # Hz (50ms between steps)
        self.ANIMATION_DURATION = 2.0 # seconds for full movement
        
        # Current hinge positions
        self.left_hinge_pos = 0.0
        self.right_hinge_pos = 0.0
        
        # Animation state
        self.animating = False
        self.animation_target_left = 0.0
        self.animation_target_right = 0.0
        self.animation_type = ""  # "UP" or "DOWN"
        
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
        
        # Status publisher for animation feedback
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
        
        # Create services for hinge control
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
        
        # Animation timer
        self.animation_timer = self.create_timer(
            1.0 / self.ANIMATION_RATE,  # 50ms
            self.animation_step
        )
        
        # Initialize messages
        self.left_hinge_msg = Float64MultiArray()
        self.right_hinge_msg = Float64MultiArray()
        
        self.get_logger().info('Mani_Sim node initialized with animation')
        self.get_logger().info('Services available:')
        self.get_logger().info('  /set_hinges_down - Lower both hinges (animated)')
        self.get_logger().info('  /set_hinges_up - Raise both hinges (animated)')
        self.get_logger().info('Topics:')
        self.get_logger().info('  /hinge_animation_status - Animation status messages')
        self.get_logger().info('  /hinge_animation_complete - Animation completion signal')
        self.get_logger().info(f'Animation: {self.ANIMATION_DURATION}s duration, {self.ANIMATION_STEP} rad steps')
    
    def publish_hinge_positions(self):
        """Publish current hinge positions to controllers"""
        # Left hinge
        self.left_hinge_msg.data = [self.left_hinge_pos]
        self.left_hinge_pub.publish(self.left_hinge_msg)
        
        # Right hinge  
        self.right_hinge_msg.data = [self.right_hinge_pos]
        self.right_hinge_pub.publish(self.right_hinge_msg)
    
    def start_animation(self, target_left, target_right, animation_type):
        """Start animated movement to target positions"""
        if self.animating:
            self.get_logger().warn('Animation already in progress, ignoring new request')
            return False
        
        self.animation_target_left = target_left
        self.animation_target_right = target_right
        self.animation_type = animation_type
        self.animating = True
        
        # Publish status
        status_msg = String()
        status_msg.data = f"ANIMATING_{animation_type}"
        self.animation_status_pub.publish(status_msg)
        
        self.get_logger().info(f'Starting {animation_type} animation')
        self.get_logger().info(f'  Left: {self.left_hinge_pos:.3f} → {target_left:.3f}')
        self.get_logger().info(f'  Right: {self.right_hinge_pos:.3f} → {target_right:.3f}')
        
        return True
    
    def animation_step(self):
        """Execute one step of the animation"""
        if not self.animating:
            return
        
        # Calculate steps for left hinge
        left_diff = self.animation_target_left - self.left_hinge_pos
        if abs(left_diff) < self.ANIMATION_STEP:
            self.left_hinge_pos = self.animation_target_left
        else:
            self.left_hinge_pos += self.ANIMATION_STEP if left_diff > 0 else -self.ANIMATION_STEP
        
        # Calculate steps for right hinge
        right_diff = self.animation_target_right - self.right_hinge_pos
        if abs(right_diff) < self.ANIMATION_STEP:
            self.right_hinge_pos = self.animation_target_right
        else:
            self.right_hinge_pos += self.ANIMATION_STEP if right_diff > 0 else -self.ANIMATION_STEP
        
        # Publish current positions
        self.publish_hinge_positions()
        
        # Check if animation is complete
        left_complete = abs(self.left_hinge_pos - self.animation_target_left) < 0.01
        right_complete = abs(self.right_hinge_pos - self.animation_target_right) < 0.01
        
        if left_complete and right_complete:
            self.finish_animation()
    
    def finish_animation(self):
        """Complete the animation and notify"""
        self.animating = False
        
        # Ensure exact target positions
        self.left_hinge_pos = self.animation_target_left
        self.right_hinge_pos = self.animation_target_right
        self.publish_hinge_positions()
        
        # Publish completion status
        status_msg = String()
        status_msg.data = f"COMPLETE_{self.animation_type}"
        self.animation_status_pub.publish(status_msg)
        
        complete_msg = Bool()
        complete_msg.data = True
        self.animation_complete_pub.publish(complete_msg)
        
        self.get_logger().info(f'{self.animation_type} animation completed')
        self.get_logger().info(f'  Final positions - Left: {self.left_hinge_pos:.3f}, Right: {self.right_hinge_pos:.3f}')
    
    def set_hinges_down_callback(self, request, response):
        """Service callback for lowering both hinges with animation"""
        try:
            success = self.start_animation(
                self.LEFT_HINGE_DOWN,
                self.RIGHT_HINGE_DOWN,
                "DOWN"
            )
            
            response.success = success
            if success:
                response.message = f'Started DOWN animation to loading position'
            else:
                response.message = 'Animation already in progress'
            
        except Exception as e:
            self.get_logger().error(f'Error starting DOWN animation: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response
    
    def set_hinges_up_callback(self, request, response):
        """Service callback for raising both hinges with animation"""
        try:
            success = self.start_animation(
                self.LEFT_HINGE_UP,
                self.RIGHT_HINGE_UP,
                "UP"
            )
            
            response.success = success
            if success:
                response.message = f'Started UP animation to transport position'
            else:
                response.message = 'Animation already in progress'
            
        except Exception as e:
            self.get_logger().error(f'Error starting UP animation: {str(e)}')
            response.success = False
            response.message = f'Error: {str(e)}'
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ManiSim()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()