# Swarm Drone Project Structure

## Overview
All editable swarm-related source code is organized under `src/` for clean project structure.

## Directory Structure

```
swarm_uav/
├── src/                                # All source code
│   ├── swarm_orchestrator/             # Mission coordination layer
│   │   ├── __init__.py
│   │   ├── circle_mission.py           # Example: Circle formation mission
│   │   ├── command_interface.py        # ROS2 service client for digi_rc
│   │   └── README.md                   # Orchestrator documentation
│   │
│   ├── swarm_formations/               # Formation geometry calculations
│   │   ├── __init__.py
│   │   └── generators.py               # Circle, V, Line, Grid formations
│   │
│   ├── swarm_mapping/                  # Computer vision & mapping
│   │   ├── __init__.py
│   │   ├── camera_interface.py         # ROS2 camera capture interface
│   │   ├── vision_processing.py        # OpenCV processing utilities
│   │   ├── area_mapping.py             # Grid-based area mapping
│   │   └── README.md                   # Mapping documentation
│   │
│   └── digi_rc/                        # ROS2 package (drone control)
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── srv/
│       │   └── Command.srv             # ROS2 service definition
│       └── src/
│           ├── commander.py            # Single drone MAVSDK wrapper
│           ├── single_drone.py         # Single drone controller
│           └── multi_drone.py          # Multi-drone swarm controller
│
├── build/                              # ROS2 build artifacts
├── install/                            # ROS2 install artifacts
├── log/                                # ROS2 build logs
│
├── input_map/                          # Captured drone camera images
├── maps/                               # Gazebo world files
│   ├── satellite_map.world
│   └── sample_terrain.png
│
├── mavsdk_server                       # MAVSDK gRPC server binary
├── run_multi_drone_gazebo.sh           # Main launch script
├── spawn_drones_with_cameras.sh        # Gazebo drone spawn script
├── setup_satellite_map.py              # Map configuration utility
├── add_drone_cameras.py                # Camera setup script
│
├── .gitignore                          # Git ignore rules
└── README.md                           # Main project documentation
```

## Module Responsibilities

### 1. `src/swarm_orchestrator/` - Mission Coordination
**Purpose**: High-level mission planning and execution

**What it does:**
- Defines mission sequences (ARM → TAKEOFF → FORMATION → LAND)
- Coordinates multiple drones through ROS2 services
- Manages mission timing and error handling
- Provides clean API for custom missions

**What it doesn't do:**
- Direct drone control (uses `command_interface.py` → digi_rc)
- Formation calculations (uses `swarm_formations`)
- Vision processing (uses `swarm_mapping`)

**Key files:**
- `command_interface.py` - ROS2 service client communicating with digi_rc
- `circle_mission.py` - Complete working example mission

**Communication:**
```
circle_mission.py 
  → command_interface.py (ROS2 client)
    → /drone_X/mission_command service
      → digi_rc/multi_drone.py (ROS2 server)
```

---

### 2. `src/swarm_formations/` - Formation Geometry
**Purpose**: Calculate drone positions for various formations

**Available formations:**
- **Circle**: Drones arranged in circular pattern
- **V-Formation**: Classic V-shape with leader
- **Line**: Horizontal or vertical line
- **Grid**: Rectangular grid pattern
- **Diamond**: Diamond shape with center drone

**Key file:**
- `generators.py` - Pure geometry calculations (no drone control)

**Usage:**
```python
from src.swarm_formations.generators import FormationGenerator

positions = FormationGenerator.generate_circle_formation(
    num_drones=5,
    center=(0, 0, 20),
    radius=15.0
)
# Returns: {1: (15.0, 0.0, 20.0), 2: (10.6, 10.6, 20.0), ...}
```

---

### 3. `src/swarm_mapping/` - Vision & Mapping
**Purpose**: Camera interface and computer vision processing

**Capabilities:**
- Real-time camera feed capture from multiple drones
- OpenCV-based image processing
- Grid-based area mapping
- Object detection and tracking
- Image stitching and mosaicking

**Key files:**
- `camera_interface.py` - ROS2 interface for drone cameras
- `vision_processing.py` - OpenCV utilities (edge detection, feature matching)
- `area_mapping.py` - Grid-based mapping and coverage tracking

**Integration:**
```python
from src.swarm_mapping.camera_interface import CameraInterface
from src.swarm_mapping.vision_processing import VisionProcessor

camera = CameraInterface(num_drones=5)
vision = VisionProcessor()
```

---

### 4. `src/digi_rc/` - ROS2 Communication Layer
**Purpose**: Low-level drone control via MAVSDK

**Responsibilities:**
- ROS2 service servers (`/drone_X/mission_command`)
- MAVSDK Python API wrapper
- Command translation (GOTO → MAVSDK position control)
- Telemetry publishing
- Drone state management

**Key files:**
- `multi_drone.py` - Multi-drone swarm controller (main entry point)
- `commander.py` - Single drone MAVSDK wrapper
- `srv/Command.srv` - ROS2 service definition

**Service definition:**
```
string command  # ARM, TAKEOFF, GOTO, LAND
float32 x       # Position/offset X
float32 y       # Position/offset Y
float32 z       # Altitude/height
---
string response # Execution status
```

---

## Data Flow Example: Circle Formation

```
1. User runs: python3 src/swarm_orchestrator/circle_mission.py

2. circle_mission.py:
   - Calculates positions: FormationGenerator.generate_circle_formation()
   - Returns: {1: (15.0, 0.0, 20.0), 2: (10.6, 10.6, 20.0), ...}

3. circle_mission.py → command_interface.py:
   - Calls: await cmd_interface.send_formation_positions(positions)
   
4. command_interface.py:
   - Creates ROS2 service request for each drone
   - Service: /drone_1/mission_command
   - Request: {command: "GOTO", x: 15.0, y: 0.0, z: 20.0}
   
5. digi_rc/multi_drone.py:
   - Receives service call
   - Calls: goto_drone(drone_index, 15.0, 0.0, 20.0)
   
6. goto_drone():
   - Translates to NED coordinates
   - Calls: await drone.offboard.set_position_ned(...)
   
7. MAVSDK:
   - Sends MAVLink to PX4
   - Command: SET_POSITION_TARGET_LOCAL_NED
   
8. PX4 in Gazebo:
   - Executes offboard position control
   - Drone moves to formation position
```

---

## Import Conventions

### Correct imports (after restructure):
```python
# Orchestrator
from src.swarm_orchestrator.command_interface import CommandInterface

# Formations
from src.swarm_formations.generators import FormationGenerator

# Mapping
from src.swarm_mapping.camera_interface import CameraInterface
from src.swarm_mapping.vision_processing import VisionProcessor
from src.swarm_mapping.area_mapping import AreaMapper

# digi_rc (ROS2 service)
from digi_rc.srv import Command
```

### Running missions:
```bash
# From project root
python3 src/swarm_orchestrator/circle_mission.py
```

---

## Design Principles

### ✅ Clean Separation
- **Orchestrator**: Mission logic (what to do)
- **Formations**: Geometry calculations (where to go)
- **Mapping**: Vision processing (what we see)
- **digi_rc**: Hardware abstraction (how to command drones)

### ✅ Single Communication Path
- All drone commands flow through ROS2 services
- No direct MAVSDK access from orchestrator
- No alternative communication channels

### ✅ Modular & Extensible
- Add new formations: Modify `swarm_formations/generators.py`
- Add new missions: Create new file in `swarm_orchestrator/`
- Add vision features: Extend `swarm_mapping/`
- Change hardware: Modify only `digi_rc/`

### ✅ Testable
- Each module can be tested independently
- Mock ROS2 services for orchestrator testing
- Mock MAVSDK for digi_rc testing
- No hardware required for integration tests

---

## Adding New Features

### New Formation Type
1. Add generator function to `src/swarm_formations/generators.py`
2. Use in orchestrator: `positions = FormationGenerator.generate_my_formation(...)`
3. No other changes needed

### New Mission
1. Create new file: `src/swarm_orchestrator/my_mission.py`
2. Import: `from src.swarm_orchestrator.command_interface import CommandInterface`
3. Use command interface API to control drones
4. Run: `python3 src/swarm_orchestrator/my_mission.py`

### New Vision Feature
1. Add processing function to `src/swarm_mapping/vision_processing.py`
2. Use in missions via `VisionProcessor` class
3. Integrate with camera interface for real-time processing

---

## Migration Notes

**Old structure** (before refactoring):
```
orchestrator/
formations/
mapping/
```

**New structure** (current):
```
src/swarm_orchestrator/
src/swarm_formations/
src/swarm_mapping/
```

All imports have been updated. Old paths will not work.

---

## Quick Reference

| Task | Command |
|------|---------|
| Launch simulation | `./run_multi_drone_gazebo.sh` |
| Start digi_rc controller | `python3 install/digi_rc/lib/digi_rc/multi_drone.py` |
| Run circle formation | `python3 src/swarm_orchestrator/circle_mission.py` |
| Build ROS2 package | `colcon build --packages-select digi_rc` |
| Source ROS2 workspace | `source install/setup.bash` |

---

**Last Updated**: January 6, 2026  
**Project**: Swarm Drone Multi-UAV System with ROS2
