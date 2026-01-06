#!/usr/bin/env python3
"""
Area Mapping Module

Creates maps from drone camera feeds by stitching images together.
Supports multiple drones contributing to a single map.
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, field
import time


@dataclass
class MapTile:
    """Represents a piece of the map"""
    image: np.ndarray
    position: Tuple[float, float, float]  # (x, y, z) GPS/NED position
    drone_id: int
    timestamp: float
    features: Optional[Tuple] = None  # (keypoints, descriptors)


class AreaMapper:
    """
    Creates maps by stitching images from multiple drones.
    Uses feature matching for image alignment.
    """
    
    def __init__(self, map_width: int = 2000, map_height: int = 2000):
        self.map_width = map_width
        self.map_height = map_height
        
        # Initialize blank map
        self.map_image = np.zeros((map_height, map_width, 3), dtype=np.uint8)
        self.coverage_mask = np.zeros((map_height, map_width), dtype=np.uint8)
        
        # Store map tiles
        self.tiles: List[MapTile] = []
        
        # Feature detector for stitching
        self.feature_detector = cv2.ORB_create(nfeatures=2000)
        
        # Map origin (center of map in world coordinates)
        self.origin = (0.0, 0.0)
        self.scale = 10.0  # pixels per meter
    
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """
        Convert world coordinates (NED) to map pixel coordinates.
        
        Args:
            x, y: World coordinates in meters
        
        Returns:
            (pixel_x, pixel_y) in map image
        """
        map_x = int(self.map_width / 2 + (x - self.origin[0]) * self.scale)
        map_y = int(self.map_height / 2 + (y - self.origin[1]) * self.scale)
        return map_x, map_y
    
    def map_to_world(self, pixel_x: int, pixel_y: int) -> Tuple[float, float]:
        """
        Convert map pixel coordinates to world coordinates.
        
        Args:
            pixel_x, pixel_y: Pixel coordinates in map
        
        Returns:
            (x, y) world coordinates in meters
        """
        x = (pixel_x - self.map_width / 2) / self.scale + self.origin[0]
        y = (pixel_y - self.map_height / 2) / self.scale + self.origin[1]
        return x, y
    
    def add_image(self, image: np.ndarray, position: Tuple[float, float, float], 
                  drone_id: int, extract_features: bool = True):
        """
        Add an image to the map at the specified position.
        
        Args:
            image: Camera image
            position: (x, y, z) position where image was taken
            drone_id: ID of the drone that captured the image
            extract_features: Whether to extract features for stitching
        """
        features = None
        if extract_features:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            keypoints, descriptors = self.feature_detector.detectAndCompute(gray, None)
            features = (keypoints, descriptors)
        
        tile = MapTile(
            image=image.copy(),
            position=position,
            drone_id=drone_id,
            timestamp=time.time(),
            features=features
        )
        
        self.tiles.append(tile)
        
        # Add image to map at position
        self._place_image_on_map(image, position)
    
    def _place_image_on_map(self, image: np.ndarray, position: Tuple[float, float, float]):
        """Place an image on the map at the given position"""
        x, y, z = position
        map_x, map_y = self.world_to_map(x, y)
        
        img_h, img_w = image.shape[:2]
        
        # Calculate placement bounds
        x_start = max(0, map_x - img_w // 2)
        y_start = max(0, map_y - img_h // 2)
        x_end = min(self.map_width, map_x + img_w // 2)
        y_end = min(self.map_height, map_y + img_h // 2)
        
        # Calculate image crop if it goes outside map bounds
        img_x_start = max(0, img_w // 2 - map_x)
        img_y_start = max(0, img_h // 2 - map_y)
        img_x_end = img_w - max(0, (map_x + img_w // 2) - self.map_width)
        img_y_end = img_h - max(0, (map_y + img_h // 2) - self.map_height)
        
        # Resize image to fit map scale (assume image covers certain area)
        # This is a simplified approach - more sophisticated stitching would use homography
        scaled_img = cv2.resize(image, (img_w, img_h))
        
        # Place on map with alpha blending for overlap
        if x_end > x_start and y_end > y_start:
            map_region = self.map_image[y_start:y_end, x_start:x_end]
            img_region = scaled_img[img_y_start:img_y_end, img_x_start:img_x_end]
            
            # Resize if dimensions don't match
            if map_region.shape[:2] != img_region.shape[:2]:
                img_region = cv2.resize(img_region, (map_region.shape[1], map_region.shape[0]))
            
            # Blend with existing map content
            coverage = self.coverage_mask[y_start:y_end, x_start:x_end]
            alpha = np.where(coverage > 0, 0.5, 1.0)[:, :, np.newaxis]
            
            self.map_image[y_start:y_end, x_start:x_end] = (
                alpha * img_region + (1 - alpha) * map_region
            ).astype(np.uint8)
            
            # Update coverage mask
            self.coverage_mask[y_start:y_end, x_start:x_end] = 255
    
    def stitch_images(self, use_features: bool = True) -> np.ndarray:
        """
        Stitch all images together using feature matching.
        
        Args:
            use_features: Whether to use feature-based alignment
        
        Returns:
            Stitched map image
        """
        if not self.tiles:
            return self.map_image
        
        if use_features and len(self.tiles) > 1:
            # Use OpenCV's stitcher for better results
            images = [tile.image for tile in self.tiles]
            
            stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
            status, stitched = stitcher.stitch(images)
            
            if status == cv2.Stitcher_OK:
                return stitched
            else:
                print(f"Stitching failed with status: {status}")
                return self.map_image
        else:
            return self.map_image
    
    def get_map(self) -> np.ndarray:
        """Get the current map image"""
        return self.map_image.copy()
    
    def get_coverage_map(self) -> np.ndarray:
        """Get the coverage mask showing which areas have been mapped"""
        return self.coverage_mask.copy()
    
    def get_coverage_percentage(self) -> float:
        """Calculate percentage of map area that has been covered"""
        total_pixels = self.map_width * self.map_height
        covered_pixels = np.count_nonzero(self.coverage_mask)
        return (covered_pixels / total_pixels) * 100.0
    
    def save_map(self, filepath: str):
        """Save the map to a file"""
        cv2.imwrite(filepath, self.map_image)
        print(f"Map saved to {filepath}")
    
    def draw_drone_positions(self, drone_positions: Dict[int, Tuple[float, float]]) -> np.ndarray:
        """
        Draw drone positions on the map.
        
        Args:
            drone_positions: Dictionary mapping drone_id to (x, y) position
        
        Returns:
            Map image with drone markers
        """
        result = self.map_image.copy()
        
        for drone_id, (x, y) in drone_positions.items():
            map_x, map_y = self.world_to_map(x, y)
            
            # Draw drone marker
            cv2.circle(result, (map_x, map_y), 10, (0, 255, 0), -1)
            cv2.circle(result, (map_x, map_y), 15, (255, 255, 255), 2)
            
            # Draw drone ID
            cv2.putText(result, str(drone_id), (map_x - 5, map_y + 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        return result
    
    def clear_map(self):
        """Clear the map and start fresh"""
        self.map_image = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)
        self.coverage_mask = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        self.tiles.clear()


class GridMapper:
    """
    Specialized mapper for grid-based area scanning.
    Divides area into grid cells and tracks coverage.
    """
    
    def __init__(self, area_width: float, area_height: float, cell_size: float = 5.0):
        """
        Initialize grid mapper.
        
        Args:
            area_width: Width of area to map (meters)
            area_height: Height of area to map (meters)
            cell_size: Size of each grid cell (meters)
        """
        self.area_width = area_width
        self.area_height = area_height
        self.cell_size = cell_size
        
        self.grid_cols = int(np.ceil(area_width / cell_size))
        self.grid_rows = int(np.ceil(area_height / cell_size))
        
        # Track which cells have been visited
        self.visited_cells = np.zeros((self.grid_rows, self.grid_cols), dtype=bool)
        
        # Store images for each cell
        self.cell_images: Dict[Tuple[int, int], List[np.ndarray]] = {}
    
    def position_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world position to grid cell indices"""
        col = int((x + self.area_width / 2) / self.cell_size)
        row = int((y + self.area_height / 2) / self.cell_size)
        
        col = max(0, min(col, self.grid_cols - 1))
        row = max(0, min(row, self.grid_rows - 1))
        
        return row, col
    
    def mark_visited(self, x: float, y: float, image: Optional[np.ndarray] = None):
        """Mark a cell as visited and optionally store image"""
        row, col = self.position_to_cell(x, y)
        self.visited_cells[row, col] = True
        
        if image is not None:
            if (row, col) not in self.cell_images:
                self.cell_images[(row, col)] = []
            self.cell_images[(row, col)].append(image)
    
    def get_coverage_percentage(self) -> float:
        """Calculate percentage of grid cells that have been visited"""
        total_cells = self.grid_rows * self.grid_cols
        visited_count = np.sum(self.visited_cells)
        return (visited_count / total_cells) * 100.0
    
    def get_unvisited_cells(self) -> List[Tuple[int, int]]:
        """Get list of unvisited cell coordinates"""
        unvisited = []
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                if not self.visited_cells[row, col]:
                    unvisited.append((row, col))
        return unvisited
    
    def visualize_coverage(self, cell_pixel_size: int = 50) -> np.ndarray:
        """
        Create a visualization of grid coverage.
        
        Args:
            cell_pixel_size: Size of each cell in pixels
        
        Returns:
            Coverage visualization image
        """
        img_height = self.grid_rows * cell_pixel_size
        img_width = self.grid_cols * cell_pixel_size
        
        coverage_img = np.ones((img_height, img_width, 3), dtype=np.uint8) * 200
        
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                y1 = row * cell_pixel_size
                x1 = col * cell_pixel_size
                y2 = y1 + cell_pixel_size
                x2 = x1 + cell_pixel_size
                
                if self.visited_cells[row, col]:
                    color = (0, 255, 0)  # Green for visited
                else:
                    color = (0, 0, 255)  # Red for unvisited
                
                cv2.rectangle(coverage_img, (x1, y1), (x2, y2), color, -1)
                cv2.rectangle(coverage_img, (x1, y1), (x2, y2), (0, 0, 0), 1)
        
        # Add percentage text
        percentage = self.get_coverage_percentage()
        text = f"Coverage: {percentage:.1f}%"
        cv2.putText(coverage_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                   1, (255, 255, 255), 2)
        
        return coverage_img
