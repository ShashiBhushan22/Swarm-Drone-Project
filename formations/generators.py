#!/usr/bin/env python3
"""
Formation Generators

Geometric formation pattern generators for multi-drone coordination.
Calculates positions for various formation types.
"""

from typing import Dict, Tuple, Optional
import math


class FormationGenerator:
    """Generates formation positions for different formation types"""
    
    @staticmethod
    def generate_v_formation(
        num_drones: int,
        center: Tuple[float, float, float],
        spacing: float
    ) -> Dict[int, Tuple[float, float, float]]:
        """
        Generate V-formation positions
        
        Pattern:
              1 (leader)
            2   3
          4       5
        """
        positions = {}
        x, y, z = center
        
        # Leader at front
        positions[1] = (x, y, z)
        
        # Arrange others in V-shape
        for i in range(2, num_drones + 1):
            side = 1 if i % 2 == 0 else -1  # Alternate left/right
            row = (i - 1) // 2
            
            pos_x = x - row * spacing
            pos_y = y + side * row * spacing
            pos_z = z + row * 0.5  # Slight altitude increase per row
            
            positions[i] = (pos_x, pos_y, pos_z)
        
        return positions
    
    @staticmethod
    def generate_line_formation(
        num_drones: int,
        center: Tuple[float, float, float],
        spacing: float,
        orientation: str = "horizontal"
    ) -> Dict[int, Tuple[float, float, float]]:
        """Generate line formation (horizontal or vertical)"""
        positions = {}
        x, y, z = center
        
        # Calculate starting offset to center the line
        offset = (num_drones - 1) * spacing / 2
        
        for i in range(1, num_drones + 1):
            idx = i - 1
            if orientation == "horizontal":
                positions[i] = (x, y - offset + idx * spacing, z)
            else:  # vertical
                positions[i] = (x - offset + idx * spacing, y, z)
        
        return positions
    
    @staticmethod
    def generate_circle_formation(
        num_drones: int,
        center: Tuple[float, float, float],
        radius: float
    ) -> Dict[int, Tuple[float, float, float]]:
        """Generate circular formation"""
        positions = {}
        x, y, z = center
        
        angle_step = 2 * math.pi / num_drones
        
        for i in range(1, num_drones + 1):
            angle = (i - 1) * angle_step
            pos_x = x + radius * math.cos(angle)
            pos_y = y + radius * math.sin(angle)
            positions[i] = (pos_x, pos_y, z)
        
        return positions
    
    @staticmethod
    def generate_grid_formation(
        num_drones: int,
        center: Tuple[float, float, float],
        spacing: float,
        cols: Optional[int] = None
    ) -> Dict[int, Tuple[float, float, float]]:
        """Generate grid formation"""
        positions = {}
        x, y, z = center
        
        # Auto-calculate columns if not specified
        if cols is None:
            cols = int(math.ceil(math.sqrt(num_drones)))
        
        rows = int(math.ceil(num_drones / cols))
        
        # Center the grid
        x_offset = (cols - 1) * spacing / 2
        y_offset = (rows - 1) * spacing / 2
        
        drone_id = 1
        for row in range(rows):
            for col in range(cols):
                if drone_id > num_drones:
                    break
                
                pos_x = x + col * spacing - x_offset
                pos_y = y + row * spacing - y_offset
                positions[drone_id] = (pos_x, pos_y, z)
                drone_id += 1
        
        return positions
    
    @staticmethod
    def generate_diamond_formation(
        num_drones: int,
        center: Tuple[float, float, float],
        spacing: float
    ) -> Dict[int, Tuple[float, float, float]]:
        """
        Generate diamond formation
        
        Pattern (5 drones):
            1
          2   3
            4
            5
        """
        positions = {}
        x, y, z = center
        
        if num_drones >= 1:
            positions[1] = (x + spacing, y, z)  # Front
        if num_drones >= 2:
            positions[2] = (x, y - spacing, z)  # Left
        if num_drones >= 3:
            positions[3] = (x, y + spacing, z)  # Right
        if num_drones >= 4:
            positions[4] = (x - spacing, y, z)  # Rear
        if num_drones >= 5:
            positions[5] = (x, y, z)  # Center
        
        # Additional drones fill in the pattern
        if num_drones > 5:
            for i in range(6, num_drones + 1):
                angle = (i - 6) * (360 / (num_drones - 5))
                pos_x = x + spacing * 1.5 * math.cos(math.radians(angle))
                pos_y = y + spacing * 1.5 * math.sin(math.radians(angle))
                positions[i] = (pos_x, pos_y, z)
        
        return positions
