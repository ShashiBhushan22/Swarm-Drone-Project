#!/usr/bin/env python3
"""
Camera Interface Module

Handles camera data from Gazebo simulation drones.
Subscribes to camera topics and provides image data to vision processing modules.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Dict, Optional, Callable
import threading


class CameraInterface(Node):
    """
    Interface for accessing camera feeds from multiple drones.
    Subscribes to Gazebo camera topics and converts ROS images to OpenCV format.
    """
    
    def __init__(self, num_drones: int = 5):
        super().__init__('camera_interface_node')
        
        self.num_drones = num_drones
        self.bridge = CvBridge()
        
        # Storage for latest images from each drone (0-indexed)
        self.latest_images: Dict[int, np.ndarray] = {}
        self.camera_info: Dict[int, CameraInfo] = {}
        
        # Image callbacks for external processing
        self.image_callbacks: Dict[int, list] = {i: [] for i in range(num_drones)}
        
        # Thread locks for image access
        self.image_locks: Dict[int, threading.Lock] = {
            i: threading.Lock() for i in range(num_drones)
        }
        
        # Create subscribers for each drone's camera
        self.image_subscribers = {}
        self.info_subscribers = {}
        
        self._create_camera_subscribers()
        
        self.get_logger().info(f'Camera interface initialized for {num_drones} drones')
    
    def _create_camera_subscribers(self):
        """Create camera topic subscribers for all drones"""
        for drone_id in range(self.num_drones):
            # Image topic - iris_depth_camera publishes to /iris_X/camera/depth/image_raw
            image_topic = f'/iris_{drone_id}/camera/depth/image_raw'
            self.image_subscribers[drone_id] = self.create_subscription(
                Image,
                image_topic,
                lambda msg, d_id=drone_id: self._image_callback(msg, d_id),
                10
            )
            
            self.get_logger().info(f'Subscribed to {image_topic} for drone {drone_id}')
    
    def _image_callback(self, msg: Image, drone_id: int):
        """Callback for receiving camera images"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Store latest image with thread safety
            with self.image_locks[drone_id]:
                self.latest_images[drone_id] = cv_image
            
            # Call registered callbacks
            for callback in self.image_callbacks[drone_id]:
                callback(cv_image, drone_id)
                
        except Exception as e:
            self.get_logger().error(f'Error converting image from drone {drone_id}: {str(e)}')
    
    def _camera_info_callback(self, msg: CameraInfo, drone_id: int):
        """Callback for receiving camera calibration info"""
        self.camera_info[drone_id] = msg
    
    def get_latest_image(self, drone_id: int) -> Optional[np.ndarray]:
        """
        Get the latest image from a specific drone.
        
        Args:
            drone_id: ID of the drone (1-based)
        
        Returns:
            OpenCV image (numpy array) or None if no image available
        """
        if drone_id not in self.latest_images:
            return None
        
        with self.image_locks[drone_id]:
            return self.latest_images[drone_id].copy() if drone_id in self.latest_images else None
    
    def register_callback(self, drone_id: int, callback: Callable):
        """
        Register a callback function to be called when new image arrives.
        
        Args:
            drone_id: ID of the drone
            callback: Function with signature: callback(image: np.ndarray, drone_id: int)
        """
        if drone_id in self.image_callbacks:
            self.image_callbacks[drone_id].append(callback)
            self.get_logger().info(f'Registered callback for drone {drone_id}')
    
    def get_camera_info(self, drone_id: int) -> Optional[CameraInfo]:
        """Get camera calibration information for a specific drone"""
        return self.camera_info.get(drone_id)
    
    def has_image(self, drone_id: int) -> bool:
        """Check if an image is available for a specific drone"""
        return drone_id in self.latest_images
    
    def get_all_latest_images(self) -> Dict[int, np.ndarray]:
        """Get latest images from all drones"""
        images = {}
        for drone_id in range(1, self.num_drones + 1):
            img = self.get_latest_image(drone_id)
            if img is not None:
                images[drone_id] = img
        return images
    
    def save_image(self, drone_id: int, filepath: str) -> bool:
        """
        Save the latest image from a drone to file.
        
        Args:
            drone_id: ID of the drone
            filepath: Path to save the image
        
        Returns:
            True if successful, False otherwise
        """
        img = self.get_latest_image(drone_id)
        if img is not None:
            cv2.imwrite(filepath, img)
            self.get_logger().info(f'Saved image from drone {drone_id} to {filepath}')
            return True
        else:
            self.get_logger().warning(f'No image available from drone {drone_id}')
            return False


class CameraDisplay:
    """Helper class for displaying camera feeds"""
    
    def __init__(self, camera_interface: CameraInterface):
        self.camera_interface = camera_interface
        self.windows_created = set()
    
    def show_single_feed(self, drone_id: int, window_name: Optional[str] = None):
        """Display live feed from a single drone"""
        if window_name is None:
            window_name = f"Drone {drone_id} Camera"
        
        img = self.camera_interface.get_latest_image(drone_id)
        if img is not None:
            if window_name not in self.windows_created:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                self.windows_created.add(window_name)
            
            cv2.imshow(window_name, img)
            cv2.waitKey(1)
    
    def show_all_feeds(self, grid_cols: int = 3):
        """Display all drone feeds in a grid layout"""
        images = self.camera_interface.get_all_latest_images()
        
        if not images:
            return
        
        # Create grid layout
        drone_ids = sorted(images.keys())
        rows = (len(drone_ids) + grid_cols - 1) // grid_cols
        
        # Get image dimensions (assuming all images same size)
        first_img = next(iter(images.values()))
        img_h, img_w = first_img.shape[:2]
        
        # Create canvas
        canvas = np.zeros((rows * img_h, grid_cols * img_w, 3), dtype=np.uint8)
        
        # Place images in grid
        for idx, drone_id in enumerate(drone_ids):
            row = idx // grid_cols
            col = idx % grid_cols
            
            img = images[drone_id]
            
            # Add drone ID label
            labeled_img = img.copy()
            cv2.putText(labeled_img, f"Drone {drone_id}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Place in canvas
            y_start = row * img_h
            x_start = col * img_w
            canvas[y_start:y_start+img_h, x_start:x_start+img_w] = labeled_img
        
        cv2.imshow("All Drone Feeds", canvas)
        cv2.waitKey(1)
    
    def close_all(self):
        """Close all OpenCV windows"""
        cv2.destroyAllWindows()


def main(args=None):
    """Test camera interface"""
    rclpy.init(args=args)
    
    camera_interface = CameraInterface(num_drones=5)
    display = CameraDisplay(camera_interface)
    
    try:
        print("Camera interface running. Press Ctrl+C to stop.")
        print("Displaying camera feeds...")
        
        while rclpy.ok():
            rclpy.spin_once(camera_interface, timeout_sec=0.1)
            
            # Display all feeds
            display.show_all_feeds(grid_cols=3)
            
    except KeyboardInterrupt:
        print("\nShutting down camera interface...")
    finally:
        display.close_all()
        camera_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
