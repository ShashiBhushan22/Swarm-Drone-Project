# ORCHESTRATOR Module

Task allocation and state machine for multi-drone operations.

## Overview

The orchestrator is the **brain** of the system - it manages task allocation, tracks mission state, and coordinates between different modules (digi_rc, mapping, formations, and future modules). It acts as a state machine without doing the actual work.

## Architecture

```
┌──────────────────────────────────────────┐
│      ORCHESTRATOR (State Machine)        │
│  - Task Queue Management                 │
│  - State Transitions                     │
│  - Drone Role Assignment                 │
│  - Resource Allocation                   │
└──────┬───────────────────┬────────────┬──┘
       │                   │            │
       v                   v            v
┌──────────────┐   ┌──────────────┐   ┌──────────────┐
│   digi_rc    │   │   mapping    │   │  formations  │
│ (Commands)   │   │  (Vision)    │   │  (Geometry)  │
└──────────────┘   └──────────────┘   └──────────────┘
```

## Responsibilities

**What Orchestrator DOES:**
- ✅ Task queue management
- ✅ State machine (IDLE → PLANNING → EXECUTING → COMPLETED)
- ✅ Drone role assignment (LEADER, FOLLOWER, SCOUT, etc.)
- ✅ Resource allocation (which drones for which tasks)
- ✅ Conflict resolution
- ✅ Health monitoring
- ✅ Coordinate between modules

**What Orchestrator DOES NOT do:**
- ❌ Send direct drone commands (→ uses command_interface)
- ❌ Vision processing (→ uses mapping module)
- ❌ Formation calculations (→ uses formations module)
- ❌ Low-level communication (→ uses digi_rc)

## Module Components

### 1. `orchestrator_core.py`
Pure state machine and task allocator:
- **Mission state management** (IDLE, PLANNING, EXECUTING, COMPLETED, etc.)
- **Drone fleet management** with role assignments (LEADER, FOLLOWER, SCOUT, etc.)
- **Task queue system** with priority handling
- **Dynamic resource allocation** (which drones for which tasks)
- **Health monitoring** and failover coordination

### 2. `command_interface.py`
Communication bridge to `digi_rc`:
- **ROS 2 service clients** for each drone
- **Async command sending** with timeout handling
- **Batch operations** (arm all, takeoff all, land all)
- Does NOT implement logic - just sends commands

### 3. `task_definitions.py`
Task type definitions and configurations:
- **TaskType constants** (FORMATION, MAPPING, VISION_SCAN, etc.)
- **Config dataclasses** (FormationConfig, MappingConfig, etc.)
- NO execution logic - just data structures
- Used by orchestrator to understand task requirements

### 4. `example_mission.py`
Example showing orchestrator coordinating modules

## Usage Examples

### Example 1: Basic Mission with Orchestrator

```python
#!/usr/bin/env python3
import rclpy
import asyncio
from orchestrator.orchestrator_core import Orchestrator, MissionTask, DroneRole
from orchestrator.command_interface import CommandInterface
from orchestrator.task_definitions import FormationConfig, TaskType

async def run_mission():
    rclpy.init()
    
    # Initialize orchestrator and command interface
    orchestrator = Orchestrator(num_drones=5)
    cmd_interface = CommandInterface(num_drones=5)
    
    # Assign roles
    orchestrator.assign_role(1, DroneRole.LEADER)
    orchestrator.assign_role(2, DroneRole.FOLLOWER)
    
    # Create V-formation task
    v_formation = FormationConfig(
        formation_type="V",
        spacing=3.0,
        center_position=(10.0, 0.0, 5.0),
        altitude=5.0,
        hold_time=5.0
    )
    
    task = MissionTask(
        task_id="formation_v_001",
        task_type=TaskType.FORMATION,
        assigned_drones=[1, 2, 3, 4, 5],
        parameters={"config": v_formation},
        priority=1
    )
    
    # Add task to queue
    orchestrator.add_mission_task(task)
    
    # Execute tasks
    orchestrator.execute_next_task()
    
    # Get swarm status
    status = orchestrator.get_swarm_status()
    print(f"Swarm status: {status}")
    
    orchestrator.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(run_mission())
```

### Example 2: Formation Flight with Command Interface

```python
#!/usr/bin/env python3
import rclpy
import asyncio
from orchestrator.command_interface import CommandInterface
from orchestrator.task_definitions import FormationGenerator

async def run_formation_flight():
    rclpy.init()
    
    cmd_interface = CommandInterface(num_drones=5)
    
    # Arm all drones
    await cmd_interface.arm_all_drones()
    
    # Takeoff to different altitudes
    await cmd_interface.takeoff_all_drones(
        altitudes={1: 5.0, 2: 6.0, 3: 7.0, 4: 8.0, 5: 9.0}
    )
    
    await asyncio.sleep(10)  # Wait for takeoff
    
    # Generate V-formation positions
    positions = FormationGenerator.generate_v_formation(
        num_drones=5,
        center=(10.0, 0.0, 5.0),
        spacing=3.0
    )
    
    # Send formation positions
    await cmd_interface.send_formation_positions(positions)
    
    await asyncio.sleep(10)  # Hold formation
    
    # Land all drones
    await cmd_interface.land_all_drones()
    
    cmd_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(run_formation_flight())
```

### Example 3: Different Formation Types

```python
from orchestrator.task_definitions import FormationGenerator

# V-formation
v_positions = FormationGenerator.generate_v_formation(
    num_drones=5,
    center=(10.0, 0.0, 5.0),
    spacing=3.0
)

# Circle formation
circle_positions = FormationGenerator.generate_circle_formation(
    num_drones=5,
    center=(20.0, 0.0, 8.0),
    radius=5.0
)

# Grid formation
grid_positions = FormationGenerator.generate_grid_formation(
    num_drones=6,
    center=(30.0, 0.0, 6.0),
    spacing=4.0,
    cols=3  # 3 columns, 2 rows
)

# Line formation
line_positions = FormationGenerator.generate_line_formation(
    num_drones=5,
    center=(15.0, 0.0, 7.0),
    spacing=3.0,
    orientation="horizontal"
)
```

## Integration with Existing System

The orchestrator integrates seamlessly with the existing setup:

1. **Gazebo & PX4**: No changes needed
2. **digi_rc controller** (`multi_drone.py`): No changes needed - continues to receive commands via ROS 2 services
3. **New orchestrator layer**: Adds high-level coordination on top

### Running with Orchestrator

You can use orchestrator in addition to or instead of `mock_mission`:

```bash
# Terminal 1: Launch Gazebo (as before)
./run_multi_drone_gazebo.sh

# Terminal 2: Start controller (as before)
source install/setup.bash
python3 install/digi_rc/lib/digi_rc/multi_drone.py

# Terminal 3: Run orchestrator mission (NEW)
source install/setup.bash
python3 orchestrator/command_interface.py  # Test script
# OR create your own orchestrator mission script
```

## Available Formation Types

1. **V-Formation**: Classic V-shape with leader at front
2. **Line Formation**: Horizontal or vertical line
3. **Circle Formation**: Drones arranged in a circle
4. **Grid Formation**: Rectangular grid pattern
5. **Diamond Formation**: Diamond shape with center

## Task Types

- `FORMATION` - Formation flight patterns
- `WAYPOINT_SEQUENCE` - Follow waypoint paths
- `AREA_SURVEILLANCE` - Scan a defined area
- `PERIMETER_PATROL` - Patrol around perimeter
- `SEARCH_AND_RESCUE` - Search pattern missions
- `PAYLOAD_DELIVERY` - Delivery missions
- `RELAY_NETWORK` - Communication relay
- `CUSTOM` - Custom mission types

## Drone Roles

- `LEADER` - Formation leader, primary coordinator
- `FOLLOWER` - Follows formation, standard role
- `SCOUT` - Advanced reconnaissance
- `GUARD` - Perimeter security
- `PAYLOAD` - Carrying payload
- `RELAY` - Communication relay node

## Future Enhancements

- Real-time telemetry monitoring
- Obstacle avoidance integration
- Battery management and swap planning
- GPS waypoint support
- Custom formation designer
- Mission replay and logging
- Web-based mission control interface

## Files

- `__init__.py` - Module initialization
- `orchestrator_core.py` - Main orchestrator class
- `task_definitions.py` - Task types and formation generators
- `command_interface.py` - ROS 2 service interface to digi_rc
- `README.md` - This file

## Dependencies

- ROS 2 Humble
- digi_rc package (for MissionCommand service definition)
- Python 3.10+
- asyncio

## Notes

- The orchestrator **does not modify** `digi_rc` package
- All communication happens through ROS 2 services
- Task definitions are completely independent
- Easy to add new formation types and mission patterns
