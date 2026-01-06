# ORCHESTRATOR Module

High-level mission coordination and drone communication interface.

## Overview

The orchestrator is the **mission control layer** - it defines missions, coordinates formation patterns, sequences commands, and manages the overall mission flow. It communicates **exclusively** through the `digi_rc` ROS2 services, maintaining clean separation between mission logic and drone control.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    ORCHESTRATOR LAYER                         │
│  • Mission definition & sequencing                            │
│  • Formation position calculations                            │
│  • Command coordination (ARM → TAKEOFF → GOTO → LAND)        │
│  • NO direct drone control - uses ROS2 services only         │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         │ ROS2 Service Calls
                         │ /drone_X/mission_command
                         │
┌────────────────────────▼─────────────────────────────────────┐
│                  COMMUNICATION LAYER (digi_rc)                │
│  • ROS2 service servers for each drone                        │
│  • Command translation (GOTO → MAVSDK position control)       │
│  • MAVSDK wrapper & state management                          │
└────────────────────────┬─────────────────────────────────────┘
                         │
                         │ MAVSDK Python API
                         │ (via mavsdk_server)
                         │
┌────────────────────────▼─────────────────────────────────────┐
│                      DRONE LAYER (PX4)                        │
│  • MAVLink protocol                                           │
│  • Offboard position control                                  │
│  • Motor commands & telemetry                                 │
└──────────────────────────────────────────────────────────────┘
```

## How Communication Works

### Circle Formation Example Flow:

1. **Orchestrator calculates positions** (using `formations/generators.py`)
   ```python
   circle_positions = FormationGenerator.generate_circle_formation(
       num_drones=5, radius=15.0, center=(0, 0, 20)
   )
   # Returns: {1: (15.0, 0.0, 20.0), 2: (10.6, 10.6, 20.0), ...}
   ```

2. **Orchestrator sends ROS2 service calls** (via `command_interface.py`)
   ```python
   await cmd_interface.send_formation_positions(circle_positions)
   # Creates ROS2 service request for each drone
   ```

3. **digi_rc receives service request** (`multi_drone.py`)
   ```
   Service: /drone_1/mission_command
   Request: {command: "GOTO", x: 15.0, y: 0.0, z: 20.0}
   ```

4. **digi_rc executes MAVSDK command**
   ```python
   await drone.offboard.set_position_ned(
       PositionNedYaw(north=15.0, east=0.0, down=-20.0, yaw=0.0)
   )
   ```

5. **MAVSDK sends MAVLink to PX4**
   ```
   MAVLink: SET_POSITION_TARGET_LOCAL_NED → PX4 executes position control
   ```

### Key Design Principle: Single Communication Path

✅ **Orchestrator sends commands ONLY through:**
- ROS2 service calls to `/drone_X/mission_command`

❌ **Orchestrator NEVER uses:**
- Direct MAVSDK imports or API calls
- Direct MAVLink communication
- Shared files or memory
- Direct TCP/UDP sockets to drones

This ensures:
- All commands are traceable and logged
- Clean separation of concerns
- Easy to test independently
- Can run on different machines (network-transparent)

## Module Components

### 1. `command_interface.py`
ROS2 service client for communicating with digi_rc:
- **Service clients**: One per drone `/drone_X/mission_command`
- **Command methods**: `arm_drone()`, `takeoff_drone()`, `goto_position()`, `land_drone()`
- **Batch operations**: `arm_all_drones()`, `takeoff_all_drones()`, `send_formation_positions()`, `land_all_drones()`
- **Async communication**: Non-blocking command execution with timeout handling

### 2. `circle_mission.py`
Complete working example mission:
- Demonstrates full mission sequence
- Uses formations module for position calculation
- Shows camera capture simulation
- Proper error handling and cleanup

## Usage Examples

### Example 1: Run Complete Circle Formation Mission

The `circle_mission.py` demonstrates a complete mission workflow:

```bash
# Terminal 1: Launch Gazebo simulation
./run_multi_drone_gazebo.sh

# Terminal 2: Start digi_rc controller
source install/setup.bash
python3 install/digi_rc/lib/digi_rc/multi_drone.py

# Terminal 3: Run circle formation mission
python3 orchestrator/circle_mission.py
```

Mission sequence executed:
1. ARM all 5 drones
2. TAKEOFF to 20m altitude
3. GOTO circle formation positions (15m radius)
4. HOLD formation for 30s (capturing simulated camera images)
5. LAND all drones

### Example 2: Custom Formation Mission with Command Interface

```python
#!/usr/bin/env python3
import rclpy
import asyncio
from orchestrator.command_interface import CommandInterface
from formations.generators import FormationGenerator

async def run_v_formation():
    rclpy.init()
    
    cmd_interface = CommandInterface(num_drones=5)
    
    try:
        # Step 1: Arm all drones
        print("Arming drones...")
        await cmd_interface.arm_all_drones()
        await asyncio.sleep(2)
        
        # Step 2: Takeoff to uniform altitude
        print("Taking off...")
        await cmd_interface.takeoff_all_drones(
            altitudes={i: 15.0 for i in range(1, 6)}
        )
        await asyncio.sleep(15)
        
        # Step 3: Generate V-formation positions
        print("Moving to V-formation...")
        v_positions = FormationGenerator.generate_v_formation(
            num_drones=5,
            center=(20.0, 0.0, 15.0),
            spacing=5.0,
            angle=45.0
        )
        
        # Step 4: Send formation positions
        await cmd_interface.send_formation_positions(v_positions)
        await asyncio.sleep(20)
        
        # Step 5: Land all drones
        print("Landing...")
        await cmd_interface.land_all_drones()
        await asyncio.sleep(15)
        
        print("Mission complete!")
        
    finally:
        cmd_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(run_v_formation())
```

### Example 3: Available Formation Types

```python
from formations.generators import FormationGenerator

# Line formation - horizontal or vertical
line_positions = FormationGenerator.generate_line_formation(
    num_drones=5,
    center=(10.0, 0.0, 15.0),
    spacing=4.0,
    orientation='horizontal'  # or 'vertical'
)

# Circle formation - drones in circular pattern
circle_positions = FormationGenerator.generate_circle_formation(
    num_drones=5,
    center=(20.0, 0.0, 15.0),
    radius=10.0
)

# Grid formation - rectangular grid
grid_positions = FormationGenerator.generate_grid_formation(
    num_drones=6,
    center=(30.0, 0.0, 15.0),
    spacing=5.0,
    rows=2,
    cols=3
)

# V-formation - leader at front
v_positions = FormationGenerator.generate_v_formation(
    num_drones=5,
    center=(40.0, 0.0, 15.0),
    spacing=4.0,
    angle=45.0
)
```

## Command Interface API Reference

### Individual Drone Commands

```python
# Arm a specific drone
success = await cmd_interface.arm_drone(drone_id=1)

# Takeoff to specified altitude
success = await cmd_interface.takeoff_drone(
    drone_id=1, 
    altitude=15.0,
    x=0.0,  # optional offset
    y=0.0   # optional offset
)

# Move to position (relative x,y and absolute z)
success = await cmd_interface.goto_position(
    drone_id=1,
    x=10.0,  # meters north
    y=5.0,   # meters east
    z=15.0   # meters altitude
)

# Land the drone
success = await cmd_interface.land_drone(drone_id=1)
```

### Batch Commands (Multiple Drones)

```python
# Arm multiple drones
results = await cmd_interface.arm_all_drones()
# Returns: {1: True, 2: True, 3: True, ...}

# Takeoff with different altitudes per drone
results = await cmd_interface.takeoff_all_drones(
    altitudes={1: 15.0, 2: 16.0, 3: 17.0}
)

# Send formation positions (dict of drone_id: (x, y, z))
positions = {
    1: (10.0, 0.0, 15.0),
    2: (10.0, 5.0, 15.0),
    3: (10.0, -5.0, 15.0)
}
results = await cmd_interface.send_formation_positions(positions)

# Land all drones
results = await cmd_interface.land_all_drones()
```

## ROS2 Service Communication Details

### Service Definition

The orchestrator communicates with digi_rc through ROS2 services defined in `digi_rc/srv/Command.srv`:

```
string command  # ARM, TAKEOFF, GOTO, LAND
float32 x       # Position X or offset (meters)
float32 y       # Position Y or offset (meters)  
float32 z       # Altitude or height (meters)
---
string response # Execution status
```

### Service Topics

Each drone has its own service endpoint:
- `/drone_1/mission_command` (digi_rc/srv/Command)
- `/drone_2/mission_command` (digi_rc/srv/Command)
- `/drone_3/mission_command` (digi_rc/srv/Command)
- etc.

### Command Flow Timing

```python
# Example timing for circle formation with 5 drones:

ARM all drones          → ~0.5s per drone   = 2-3 seconds total
TAKEOFF to 20m          → ~15 seconds       (gradual ascent)
GOTO circle positions   → ~10 seconds       (smooth interpolation)
HOLD formation          → 30 seconds        (configurable)
LAND all drones         → ~15 seconds       (gradual descent)

Total mission time: ~75 seconds
```

## Integration with Existing System

The orchestrator layer adds mission coordination **on top** of existing components without modifying them:

### Unchanged Components:
- ✅ Gazebo simulation setup
- ✅ PX4 autopilot configuration
- ✅ `digi_rc/multi_drone.py` service servers
- ✅ MAVSDK communication layer
- ✅ `formations/generators.py` position calculations

### New Layer:
- ➕ `orchestrator/command_interface.py` - ROS2 service client
- ➕ `orchestrator/circle_mission.py` - Example mission
- ➕ Your custom missions using command interface

## Creating Custom Missions

Template for creating your own mission:

```python
#!/usr/bin/env python3
import rclpy
import asyncio
from orchestrator.command_interface import CommandInterface
from formations.generators import FormationGenerator

async def my_custom_mission():
    rclpy.init()
    
    # Initialize command interface
    num_drones = 5
    cmd_interface = CommandInterface(num_drones=num_drones)
    
    try:
        # Your mission logic here
        # 1. Arm drones
        await cmd_interface.arm_all_drones()
        await asyncio.sleep(2)
        
        # 2. Takeoff
        await cmd_interface.takeoff_all_drones(
            altitudes={i: 15.0 for i in range(1, num_drones + 1)}
        )
        await asyncio.sleep(15)
        
        # 3. Execute formation(s)
        # ... your formation logic ...
        
        # 4. Land
        await cmd_interface.land_all_drones()
        await asyncio.sleep(15)
        
    finally:
        cmd_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(my_custom_mission())
```

## Why This Architecture is Clean

### ✅ Separation of Concerns
- **Orchestrator**: Mission logic, formation planning, sequencing
- **digi_rc**: Hardware abstraction, MAVSDK interface, drone state
- **MAVSDK/PX4**: Low-level control, motor commands, telemetry

### ✅ Single Source of Truth
- Circle formation positions calculated **only** in orchestrator
- Commands flow **only** through ROS2 service interface
- No alternative communication paths or backdoors

### ✅ Testability
- Mock ROS2 services to test orchestrator independently
- Mock MAVSDK to test digi_rc independently
- Integration tests without hardware

### ✅ Extensibility
- Add formations: Modify `formations/generators.py`
- Add drone types: Modify `digi_rc/multi_drone.py`  
- Add missions: Create new orchestrator scripts
- Each layer independent and replaceable

## Troubleshooting

### Services Not Available
```bash
# Check if digi_rc is running
ros2 service list | grep mission_command

# Should see:
# /drone_1/mission_command
# /drone_2/mission_command
# ...
```

### Commands Not Executing
```bash
# Check service type
ros2 service type /drone_1/mission_command
# Should output: digi_rc/srv/Command

# Test service manually
ros2 service call /drone_1/mission_command digi_rc/srv/Command "{command: 'ARM', x: 0.0, y: 0.0, z: 0.0}"
```

### Timeout Errors
- Increase timeout in `command_interface.py` `send_command()` method
- Default is 30 seconds, may need more for complex maneuvers

## Files in This Module

- `__init__.py` - Module initialization
- `command_interface.py` - ROS2 service client for digi_rc communication
- `circle_mission.py` - Complete working example mission
- `README.md` - This documentation file

## Dependencies

- ROS2 Humble
- `digi_rc` package (for Command service definition)
- `formations` package (for position calculations)
- Python 3.10+
- asyncio for asynchronous command execution

## Future Enhancements

Potential additions to orchestrator (without breaking existing architecture):

- Mission templates library (search patterns, patrol routes)
- Dynamic formation transitions (morph between shapes)
- GPS waypoint integration
- Battery management and RTL (Return To Launch)
- Collision avoidance coordination
- Multi-agent task allocation algorithms
- Mission logging and replay functionality
- Web dashboard for mission monitoring
