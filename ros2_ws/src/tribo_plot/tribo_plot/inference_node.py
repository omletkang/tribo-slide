#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Float32
import numpy as np
import torch
import threading

from tribo_plot.model.lstm import TouchNetwork, SlideNetwork


class InferenceNode(Node):
    """
    Runs ML inference on window data.
    - Subscribes to: /tribo/window_buffer, /tribo/state, /tribo/touch_metric
    - Publishes to: /tribo/velocity
    - Only processes data when state is 'stay' or 'slide' (or needs touch inference)
    """
    
    def __init__(self):
        super().__init__('inference_node')
        
        # Load models
        self.touchmodel = TouchNetwork()
        self.slidemodel = SlideNetwork()
        
        # Current state
        self.current_state = 'idle'
        self.touch_metric = 0.0
        self.window_buffer = None
        self.touch_buffer = None
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String,
            '/tribo/state',
            self.state_callback,
            10
        )
        
        self.window_sub = self.create_subscription(
            Float32MultiArray,
            '/tribo/window_buffer',
            self.window_callback,
            10
        )
        
        self.touch_metric_sub = self.create_subscription(
            Float32,
            '/tribo/touch_metric',
            self.touch_metric_callback,
            10
        )
        
        # Publisher
        self.velocity_pub = self.create_publisher(
            Float32MultiArray,
            '/tribo/velocity',
            10
        )
        
        # Threading
        self.lock = threading.Lock()
        self.inference_thread = threading.Thread(
            target=self._inference_loop,
            daemon=True
        )
        self.inference_thread.start()
        
        self.get_logger().info('Inference Node initialized')
    
    def state_callback(self, msg):
        """Subscribe to state changes"""
        with self.lock:
            self.current_state = msg.data
    
    def touch_metric_callback(self, msg):
        """Subscribe to touch metric for touch state detection"""
        with self.lock:
            self.touch_metric = msg.data
    
    def window_callback(self, msg):
        """
        Receive window buffer when it's full
        Reshape from flat to (4, 50)
        """
        with self.lock:
            self.window_buffer = np.array(msg.data, dtype=np.float32).reshape(4, 50)
    
    def _inference_loop(self):
        """
        Inference loop - processes data when needed
        For 'stay'/'slide' states: runs when window is full (published by StateManager)
        For 'touch' state: would run on touch_buffer (can be triggered separately)
        """
        while rclpy.ok():
            with self.lock:
                state = self.current_state
                window_buf = self.window_buffer
                touch_metric = self.touch_metric
            
            # Process based on state
            if state == 'stay' or state == 'slide':
                if window_buf is not None:
                    # Check state to determine inference
                    if state == 'stay':
                        # Stay state: send zero velocity
                        vel = np.array([0.0, 0.0], dtype=np.float32)
                    else:  # state == 'slide'
                        # Slide state: run inference
                        vel = self._infer_slide(window_buf, state)
                    
                    if vel is not None:
                        self._publish_velocity(vel)
                    
                    # Clear buffer after processing
                    with self.lock:
                        self.window_buffer = None
            
            elif state == 'touch':
                # Touch state: infer touch location
                # You can trigger this based on a separate message or timeout
                pass
            
            # Prevent busy-waiting
            rclpy.spin_once(self, timeout_sec=0.001)
    
    def _infer_slide(self, window_data, state):
        """
        Run slide inference
        Input: (4, 50) window
        Output: (2,) velocity
        """
        try:
            # Prepare input
            input_arr = window_data.reshape(1, 4, 50).astype(np.float32)
            input_tensor = torch.from_numpy(input_arr)
            
            # Inference
            with torch.no_grad():
                vel = self.slidemodel(input_tensor)  # (1, 2)
            
            vel = vel.detach().numpy().flatten()  # (2,)
            
            self.get_logger().debug(f'Slide inference: vel={vel}, state={state}')
            return vel
        
        except Exception as e:
            self.get_logger().error(f'Error in slide inference: {e}')
            return None
    
    def _infer_touch(self, touch_data):
        """
        Run touch inference
        Input: (4, 1000) buffer
        Output: (2,) pose
        """
        try:
            # Prepare input
            input_arr = touch_data.reshape(1, 4, 1000).astype(np.float32)
            input_tensor = torch.from_numpy(input_arr)
            
            # Inference
            with torch.no_grad():
                pose = self.touchmodel(input_tensor)  # (1, 2)
            
            pose = pose.detach().numpy().flatten()  # (2,)
            
            self.get_logger().debug(f'Touch inference: pose={pose}')
            return pose
        
        except Exception as e:
            self.get_logger().error(f'Error in touch inference: {e}')
            return None
    
    def _publish_velocity(self, vel):
        """
        Publish velocity result (or pose)
        vel/pose: (2,) numpy array
        """
        msg = Float32MultiArray(data=vel.tolist())
        self.velocity_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = InferenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
