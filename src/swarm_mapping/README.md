# MAPPING Module

Computer vision and area mapping for multi-drone systems using OpenCV.

## Overview

The mapping module provides camera interfacing, OpenCV-based vision processing, and map generation from drone camera feeds. It operates independently from the orchestrator and digi_rc layers.

## Architecture

```
┌──────────────────────────────────────────┐
│         ORCHESTRATOR                     │
│    (Task allocation & state machine)     │
└──────────────┬───────────────────────────┘
               │
               ├──> Commands to drones
               │
┌──────────────┴───────────────────────────┐
│         MAPPING (this layer)             │
│  - Camera Interface                      │
│  - Vision Processing (OpenCV)            │
│  - Area Mapping                          │
└──────────────┬───────────────────────────┘
               │
               ├──> Subscribe to camera topics
               │
┌──────────────┴───────────────────────────┐
│         Gazebo Simulation                │
│    (Camera sensors on drones)            │
└──────────────────────────────────────────┘
```

## Module Components

### 1. `camera_interface.py`
Handles camera data from Gazebo drone simulations:
- **Subscribes to camera topics** (`/drone_X/camera/image_raw`)
- **Converts ROS images** to OpenCV format using cv_bridge
- **Manages multiple camera feeds** from all drones
- **Provides callbacks** for real-time image processing
- **Display utilities** for viewing camera feeds

**Key Classes:**
- `CameraInterface` - Main camera subscriber and manager
- `CameraDisplay` - Helper for displaying feeds

### 2. `vision_processing.py`
OpenCV-based computer vision processing:
- **Color detection** - Detect colored objects
- **ArUco marker detection** - Find and identify markers
- **Edge detection** - Canny edge detector
- **Line detection** - Hough line transform
- **Circle detection** - Hough circle transform
- **Feature extraction** - ORB/SIFT keypoints
- **Feature matching** - Match features between images
- **Optical flow** - Motion estimation

**Key Classes:**
- `VisionProcessor` - Main vision processing algorithms
- `Detection` - Detection result data structure
- `VisualizationHelper` - Draw detection results

### 3. `area_mapping.py`
Creates maps from drone camera feeds:
- **Image stitching** - Combine images into map
- **Position-based mapping** - Place images at GPS/NED coordinates
- **Coverage tracking** - Track mapped areas
- **Multi-drone mapping** - Multiple drones contribute to single map
- **Grid-based scanning** - Systematic area coverage

**Key Classes:**
- `AreaMapper` - Main mapping engine
- `GridMapper` - Grid-based coverage mapping
- `MapTile` - Individual map piece

## Installation

```bash
# Install OpenCV and dependencies
pip install opencv-python opencv-contrib-python

# Install ROS-OpenCV bridge
sudo apt install ros-humble-cv-bridge

# Install additional vision libraries
pip install numpy scipy
```

## Usage Examples

### Example 1: Display Camera Feeds

```python
#!/usr/bin/env python3
import rclpy
from src.swarm_mapping.camera_interface import CameraInterface, CameraDisplay

rclpy.init()

camera_interface = CameraInterface(num_drones=5)
display = CameraDisplay(camera_interface)

try:
    while rclpy.ok():
        rclpy.spin_once(camera_interface, timeout_sec=0.1)
        display.show_all_feeds(grid_cols=3)
except KeyboardInterrupt:
    pass
finally:
    display.close_all()
    camera_interface.destroy_node()
    rclpy.shutdown()
```

### Example 2: Detect Objects in Camera Feed

```python
#!/usr/bin/env python3
import rclpy
from src.swarm_mapping.camera_interface import CameraInterface
from src.swarm_mapping.vision_processing import VisionProcessor, VisualizationHelper

rclpy.init()

camera_interface = CameraInterface(num_drones=5)
processor = VisionProcessor()

def process_image(image, drone_id):
    # Detect red objects
    detections = processor.detect_colors(image, colors=['red'])
    
    if detections['red']:
        print(f"Drone {drone_id}: Found {len(detections['red'])} red objects")
        
        # Visualize
        result = VisualizationHelper.draw_detections(image, detections['red'])
        cv2.imshow(f"Drone {drone_id} Detections", result)
        cv2.waitKey(1)

# Register callback for drone 1
camera_interface.register_callback(1, process_image)

try:
    rclpy.spin(camera_interface)
except KeyboardInterrupt:
    pass
finally:
    camera_interface.destroy_node()
    rclpy.shutdown()
```

### Example 3: Create Map from Multiple Drones

```python
#!/usr/bin/env python3
import rclpy
from src.swarm_mapping.camera_interface import CameraInterface
from src.swarm_mapping.area_mapping import AreaMapper

rclpy.init()

camera_interface = CameraInterface(num_drones=5)
mapper = AreaMapper(map_width=2000, map_height=2000)

# Assume we have drone positions from telemetry
drone_positions = {
    1: (0.0, 0.0, 5.0),
    2: (10.0, 0.0, 5.0),
    3: (0.0, 10.0, 5.0),
    # ... etc
}

def capture_for_map(image, drone_id):
    if drone_id in drone_positions:
        position = drone_positions[drone_id]
        mapper.add_image(image, position, drone_id)
        
        # Display current map
        map_img = mapper.get_map()
        cv2.imshow("Area Map", map_img)
        cv2.waitKey(1)
        
        # Print coverage
        coverage = mapper.get_coverage_percentage()
        print(f"Map coverage: {coverage:.1f}%")

# Register callbacks for all drones
for drone_id in range(1, 6):
    camera_interface.register_callback(drone_id, capture_for_map)

try:
    rclpy.spin(camera_interface)
except KeyboardInterrupt:
    mapper.save_map("final_map.png")
finally:
    camera_interface.destroy_node()
    rclpy.shutdown()
```

### Example 4: ArUco Marker Detection

```python
#!/usr/bin/env python3
import rclpy
import cv2
from src.swarm_mapping.camera_interface import CameraInterface
from src.swarm_mapping.vision_processing import VisionProcessor, VisualizationHelper

rclpy.init()

camera_interface = CameraInterface(num_drones=5)
processor = VisionProcessor()

def detect_markers(image, drone_id):
    # Detect ArUco markers
    markers = processor.detect_aruco_markers(image)
    
    if markers:
        print(f"Drone {drone_id}: Detected {len(markers)} markers")
        for marker in markers:
            print(f"  - {marker.label} at {marker.center}")
        
        # Draw markers
        result = VisualizationHelper.draw_detections(image, markers)
        cv2.imshow(f"Drone {drone_id} Markers", result)
        cv2.waitKey(1)

camera_interface.register_callback(1, detect_markers)

try:
    rclpy.spin(camera_interface)
except KeyboardInterrupt:
    pass
finally:
    camera_interface.destroy_node()
    rclpy.shutdown()
```

### Example 5: Grid-Based Area Scanning

```python
#!/usr/bin/env python3
from src.swarm_mapping.area_mapping import GridMapper
import cv2

# Create grid for 100x100 meter area with 5m cells
grid_mapper = GridMapper(area_width=100.0, area_height=100.0, cell_size=5.0)

# Simulate drone scanning area
scan_positions = [
    (0, 0), (5, 0), (10, 0),
    (0, 5), (5, 5), (10, 5),
    # ... etc
]

for x, y in scan_positions:
    grid_mapper.mark_visited(x, y)
    
    # Visualize coverage
    coverage_img = grid_mapper.visualize_coverage()
    cv2.imshow("Scan Coverage", coverage_img)
    cv2.waitKey(100)

print(f"Total coverage: {grid_mapper.get_coverage_percentage():.1f}%")

# Get unvisited cells
unvisited = grid_mapper.get_unvisited_cells()
print(f"Unvisited cells: {len(unvisited)}")
```

## Integration with Orchestrator

The mapping module can be used by the orchestrator to make vision-based decisions:

```python
# In orchestrator mission script
from src.swarm_mapping.camera_interface import CameraInterface
from src.swarm_mapping.vision_processing import VisionProcessor
from src.swarm_orchestrator.command_interface import CommandInterface

camera = CameraInterface(num_drones=5)
vision = VisionProcessor()
commander = CommandInterface(num_drones=5)

# Detect target and navigate
def find_and_goto_target(image, drone_id):
    detections = vision.detect_colors(image, colors=['red'])
    
    if detections['red']:
        target = detections['red'][0]
        # Convert pixel coordinates to world coordinates
        # ... coordinate transformation ...
        
        # Command drone to target
        await commander.goto_position(drone_id, target_x, target_y, altitude)
```

## Vision Processing Capabilities

### Supported Detections:
- ✅ Color-based object detection
- ✅ ArUco marker detection
- ✅ Edge detection (Canny)
- ✅ Line detection (Hough)
- ✅ Circle detection
- ✅ Feature extraction (ORB, SIFT)
- ✅ Optical flow calculation

### Supported Mapping:
- ✅ Position-based image stitching
- ✅ Feature-based alignment
- ✅ Coverage tracking
- ✅ Multi-drone mapping
- ✅ Grid-based scanning
- ✅ Map export

## Camera Topics

Assuming Gazebo cameras are set up, topics will be:
- `/drone_1/camera/image_raw` - RGB image
- `/drone_1/camera/camera_info` - Camera calibration
- `/drone_1/camera/depth/image_raw` - Depth image (if depth camera)

## Future Enhancements

- YOLO/deep learning object detection
- Semantic segmentation
- 3D reconstruction from multiple views
- Real-time SLAM (Simultaneous Localization and Mapping)
- Terrain elevation mapping
- Dynamic object tracking
- GPS coordinate mapping integration

## Files

- `__init__.py` - Module initialization
- `camera_interface.py` - Camera topic subscriber and manager
- `vision_processing.py` - OpenCV vision algorithms
- `area_mapping.py` - Map generation and stitching
- `README.md` - This file

## Dependencies

- ROS 2 Humble
- OpenCV 4.x
- cv_bridge
- NumPy
- Python 3.10+

## Notes

- Camera topics must exist in Gazebo (requires camera sensor on iris model)
- Processing is CPU-intensive - consider reducing frame rate for multiple drones
- Map coordinates assume NED (North-East-Down) frame
- For GPS coordinates, additional coordinate transformation needed
