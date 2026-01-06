# Multi-Drone Gazebo SITL Setup

Complete guide for running multiple drones in Gazebo simulation using ROS 2, MAVSDK, and PX4 SITL.

## Prerequisites

Before starting, ensure you have the following installed:

- **ROS 2 Humble** - Full desktop installation
- **PX4-Autopilot** - Located at `~/PX4-Autopilot`
- **Gazebo Classic 11** - Comes with ROS 2 desktop
- **MAVSDK** - Server binary included in workspace
- **Python 3.10+** - Comes with Ubuntu 22.04
- **gnome-terminal** - For automatic terminal launching

## System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                  ORCHESTRATOR (State Machine)                │
│        Task Allocation │ State Management │ Coordination     │
└───────┬─────────────────┬──────────────────┬────────────────┘
        │                 │                  │
        v                 v                  v
┌───────────────┐  ┌──────────────┐  ┌──────────────┐  ┌────────────┐
│  digi_rc      │  │  formations  │  │   mapping    │  │  Future    │
│ (Communication│  │  (Geometry)  │  │   (Vision)   │  │  Modules   │
│   Layer)      │  │              │  │              │  │            │
└───────┬───────┘  └──────────────┘  └──────────────┘  └────────────┘
        │
        v
┌──────────────────────────────────────────┐
│  MAVSDK Server │ PX4 SITL │ Gazebo       │
└──────────────────────────────────────────┘
```

## Module Structure

```
src/
├── swarm_orchestrator/    - Mission coordination & command interface
├── swarm_formations/      - Formation geometry generators  
├── swarm_mapping/         - Computer vision & area mapping
└── digi_rc/               - ROS2 communication layer (drone control)
```

All swarm-related editable source code is organized under `src/` for clean structure.

## Quick Start - Automated Launch

Run the entire mission with a single command:

```bash
cd /home/bhushan-arc/skyflock_uav_arc
./run_multi_drone_gazebo.sh
```

**What happens automatically:**
1. **Terminal 1** (current): Launches Gazebo with 5 drones
2. **Terminal 2** (auto-opens): Starts multi-drone controller
3. **Terminal 3** (auto-opens): Executes orchestrator formation flight mission

**Total time:** ~50 seconds from launch to mission completion

Watch the three terminals to see:
- Gazebo initialization and drone spawning
- Controller connecting to all 5 drones
- Orchestrator executing mission with V-formation flight

**To stop:** Press `Ctrl+C` in the Gazebo terminal (Terminal 1)

---

## QGroundControl Integration

View drone telemetry, paths, and positions in QGroundControl while flying in Gazebo:

### Quick Setup:

1. **Launch simulation** (if not already running):
   ```bash
   ./run_multi_drone_gazebo.sh
   ```

2. **Open QGroundControl:**
   ```bash
   qgroundcontrol  # or ./QGroundControl.AppImage
   ```

3. **Auto-connect:**
   - QGC automatically connects to Drone 1 on UDP port 14540
   - View real-time telemetry, position, and flight path on map
   - Enable "Show Vehicle Trails" to see complete path

### What You'll See:
- ✅ **Gazebo window**: 3D simulation of drones flying
- ✅ **QGC window**: Map with drone positions, paths, telemetry
- ✅ **Terminal output**: Mission progress from orchestrator

### Multi-Drone Viewing:
See [qgc_integration/README.md](qgc_integration/README.md) for:
- Viewing multiple drones simultaneously
- MAVLink forwarding setup
- Mission planning in QGC
- Recording flight data

---

## Manual Step-by-Step Guide (Alternative)

If you prefer manual control or the automated launch has issues, follow these steps:

### Step 1: Prepare the Environment

Open a terminal and navigate to the workspace:

```bash
cd /home/bhushan-arc/skyflock_uav_arc
```

Verify that the required files exist:
```bash
ls -l run_multi_drone_gazebo.sh mavsdk_server
```

Make sure the launch script is executable:
```bash
chmod +x run_multi_drone_gazebo.sh
```

### Step 2: Launch Gazebo Simulator

In **Terminal 1**, start Gazebo with 5 drones:

```bash
./run_multi_drone_gazebo.sh
```

**What happens:**
- Kills any existing PX4/Gazebo processes
- Launches Gazebo Classic simulator
- Spawns 5 iris quadcopters (iris_1 to iris_5)
- Starts PX4 SITL instances for each drone
- Configures MAVLink UDP ports (14540-14544)

**Expected output:**
```
╔════════════════════════════════════════╗
║  ✓ Gazebo Multi-Drone SITL Running    ║
╚════════════════════════════════════════╝

Drone Configuration:
  Drone 0: MAVLink UDP Port 14540
  Drone 1: MAVLink UDP Port 14541
  Drone 2: MAVLink UDP Port 14542
  Drone 3: MAVLink UDP Port 14543
  Drone 4: MAVLink UDP Port 14544
```

**Wait Time:** 30-40 seconds for complete initialization

**Verification:** 
- Gazebo GUI window should open showing 5 drones on the ground
- Check processes: `ps aux | grep px4 | wc -l` should show 5

### Step 3: Start the Multi-Drone Controller

Open a **new terminal (Terminal 2)** and start the controller node:

```bash
cd /home/bhushan-arc/skyflock_uav_arc
source install/setup.bash
python3 install/digi_rc/lib/digi_rc/multi_drone.py
```

**What happens:**
- Starts 5 MAVSDK server instances (ports 50041-50045)
- Connects to all 5 PX4 SITL drones via UDP
- Waits for GPS position estimates
- Sets up ROS 2 services for each drone
- Publishes telemetry data (position, armed status)

**Expected output sequence:**
```
[INFO] Starting 5 MAVSDK servers
[INFO] Found mavsdk_server at: ./mavsdk_server
Started MAVSDK server 0 on port 50041
Started MAVSDK server 1 on port 50042
...
[INFO] Connecting all drones...
[INFO] Connecting Drone 1...
[INFO] Drone 1: Connected
[INFO] Drone 1: Position estimate ready
...
[INFO] All drones connected and ready
[INFO] All drones connected. Ready to receive commands.
[INFO] Waiting for commands from sender script...
```

**Wait Time:** 10-15 seconds for all drones to connect

**Important:** Leave this terminal running - do NOT close it!

### Step 4: Execute the Formation Flight Mission

Open a **third terminal (Terminal 3)** and run the orchestrator mission:

```bash
cd /home/bhushan-arc/skyflock_uav_arc
source install/setup.bash
export PYTHONPATH=$PYTHONPATH:/home/bhushan-arc/skyflock_uav_arc
python3 src/swarm_orchestrator/circle_mission.py
```

**Alternative:** Run the mock_mission instead:
```bash
python3 mock_mission/multi_wayn.py
```

**What happens - Mission Sequence:**

1. **Initialization (2 seconds)**
   - Creates commander nodes for all 5 drones
   - Connects to ROS 2 services

2. **Arming Phase (3 seconds)**
   - Sends ARM commands to all drones simultaneously
   - Waits for confirmation that all drones are armed
   - Sets initial offboard setpoints

3. **Takeoff Phase (7-10 seconds)**
   - Drone 1: Takes off to 5.0m altitude
   - Drone 2: Takes off to 6.0m altitude
   - Drone 3: Takes off to 7.0m altitude
   - Drone 4: Takes off to 8.0m altitude
   - Drone 5: Takes off to 9.0m altitude
   - Different altitudes prevent collisions

4. **Formation Flight Phase (15-20 seconds)**
   - All drones fly to V-formation positions:
     * Drone 1: (10m, 0m, 5m) - Lead position
     * Drone 2: (8m, -2m, 6m) - Left wing
     * Drone 3: (8m, 2m, 6m) - Right wing
     * Drone 4: (6m, -4m, 7m) - Left rear
     * Drone 5: (6m, 4m, 7m) - Right rear

5. **Hold Formation (5 seconds)**
   - All drones maintain formation positions
   - Message: "Formation achieved! Holding for 5 seconds..."

6. **Landing Phase (10-15 seconds)**
   - Sends LAND commands to all drones
   - Drones descend simultaneously
   - Waits for all to reach ground
   - Disarms automatically

7. **Mission Complete**
   - Message: "Formation flight complete"
   - Mission planner exits

**Expected output during mission:**
```
[INFO] Starting formation flight
[INFO] Drone 1: Sending command ARM
[INFO] Drone 1: Armed
[INFO] Drone 1: Sending command TAKEOFF (0.0, 0.0, 5.0)
[INFO] Drone 1: Takeoff in progress - Current: 0.54m, Target: 5.0m
[INFO] Drone 1: Takeoff complete
[INFO] Drone 1: Sending command GOTO (10.0, 0.0, 5.0)
[INFO] Drone 1: Moving to target - Current: (1.27, -0.01, 4.85)
[INFO] Drone 1: Location reached
[INFO] Formation achieved! Holding for 5 seconds...
[INFO] Drone 1: Landing - Current altitude: 4.99m
[INFO] Drone 1: Landed
[INFO] Formation flight complete
```

**Total Mission Time:** Approximately 40-50 seconds

### Step 5: Observe in Gazebo

During mission execution, watch the Gazebo GUI:

- **Takeoff:** Drones lift off sequentially to different altitudes
- **Formation:** Drones move forward and spread into V-shape
- **Hold:** Drones hover in formation
- **Landing:** All drones descend together

You can rotate the camera in Gazebo to view the formation from different angles.

### Step 6: Clean Shutdown

After mission completes:

1. **Terminal 3** (mission planner) - Exits automatically
2. **Terminal 2** (controller) - Press `Ctrl+C` to stop
3. **Terminal 1** (Gazebo) - Press `Ctrl+C` to stop

**Or force stop everything:**
```bash
pkill -9 px4 && pkill -9 gzserver && pkill -9 gzclient
```

## Port Configuration

The system uses the following port mappings:

| Drone | MAVLink UDP | MAVSDK gRPC | Gazebo Model |
|-------|-------------|-------------|--------------|
| 1     | 14540       | 50041       | iris_1       |
| 2     | 14541       | 50042       | iris_2       |
| 3     | 14542       | 50043       | iris_3       |
| 4     | 14543       | 50044       | iris_4       |
| 5     | 14544       | 50045       | iris_5       |

**Note:** In the code:
- `multi_drone.py` uses drone IDs 1-5 
- Internal array indices are 0-4
- UDP port = 14540 + drone_id (but drone_id starts from 0 internally)

## Troubleshooting

### Problem: Gazebo doesn't start

**Solution:**
```bash
# Clean up any stuck processes
pkill -9 px4 && pkill -9 gzserver && pkill -9 gzclient
# Check if PX4 path is correct
ls ~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_multiple_run.sh
```

### Problem: Controller can't connect to drones

**Symptoms:** "Waiting for system..." messages that don't complete

**Solutions:**
1. Ensure Gazebo fully started (wait 30-40 seconds)
2. Verify SITL instances are running: `ps aux | grep px4`
3. Check UDP ports are not blocked: `netstat -tuln | grep 145`
4. Restart from Step 1

### Problem: Mission planner fails with "Service not available"

**Solution:**
- Ensure `multi_drone.py` is running and shows "Ready to receive commands"
- Check ROS 2 setup: `source install/setup.bash`
- Verify services exist: `ros2 service list | grep drone`

### Problem: Drones don't take off

**Possible causes:**
1. GPS not ready - wait longer in Step 2
2. MAVSDK server connection issue - check Terminal 2 output
3. PX4 mode not set correctly - check Gazebo console for errors

**Solution:** Restart all terminals from Step 1

### Problem: Drones collide during takeoff

**This shouldn't happen** - the mission uses staggered altitudes (5m, 6m, 7m, 8m, 9m).

If collisions occur, check `multi_wayn.py` - ensure different altitudes are set.

### Problem: Formation is incorrect

**Check the formation coordinates in multi_wayn.py:**
```python
# Formation positions (x, y, z)
drone1: (10, 0, 5)    # Lead
drone2: (8, -2, 6)    # Left wing
drone3: (8, 2, 6)     # Right wing
drone4: (6, -4, 7)    # Left rear
drone5: (6, 4, 7)     # Right rear
```

## Customizing Missions

### Option 1: Using Orchestrator (Recommended)

**Location: `src/swarm_orchestrator/circle_mission.py`** or create your own

The orchestrator provides a cleaner, more modular approach:

```python
#!/usr/bin/env python3
import rclpy
import asyncio
from src.swarm_orchestrator.command_interface import CommandInterface
from src.swarm_formations.generators import FormationGenerator

async def my_custom_mission():
    rclpy.init()
    cmd_interface = CommandInterface(num_drones=5)
    
    # Arm and takeoff
    await cmd_interface.arm_all_drones()
    await cmd_interface.takeoff_all_drones(
        altitudes={1: 5.0, 2: 6.0, 3: 7.0, 4: 8.0, 5: 9.0}
    )
    
    # Generate circle formation
    positions = FormationGenerator.generate_circle_formation(
        num_drones=5,
        center=(15.0, 0.0, 8.0),
        radius=5.0
    )
    
    await cmd_interface.send_formation_positions(positions)
    await asyncio.sleep(10)  # Hold formation
    
    await cmd_interface.land_all_drones()
    
    cmd_interface.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(my_custom_mission())
```

See [src/swarm_orchestrator/README.md](src/swarm_orchestrator/README.md) for more examples.

### Option 2: Using Mock Mission (Legacy)

**Location: `mock_mission/multi_wayn.py`**

The mission planner contains two mission types:

### 1. Formation Flight (default)

```python
await commander.run_formation_flight()
```

Creates a V-shaped formation with 5 drones.

**To modify formation shape**, edit the `formation_positions` dictionary:

```python
formation_positions = {
    1: (10.0, 0.0, 5.0),   # Lead drone
    2: (8.0, -2.0, 6.0),   # Adjust these coordinates
    3: (8.0, 2.0, 6.0),
    4: (6.0, -4.0, 7.0),
    5: (6.0, 4.0, 7.0)
}
```

### 2. Independent Waypoint Mission

```python
await commander.run_swarm_mission()
```

Each drone follows its own waypoint sequence.

**To modify waypoints**, edit the mission definitions:

```python
missions = {
    1: [(10, 0, 5), (20, 0, 5), (20, 10, 5)],  # Drone 1 path
    2: [(10, 5, 6), (20, 5, 6), (20, 15, 6)],  # Drone 2 path
    # ... customize as needed
}
```

### After modifying missions:

```bash
# Rebuild the package
cd /home/bhushan-arc/skyflock_uav_arc
colcon build --packages-select digi_rc --symlink-install
source install/setup.bash
```

Then restart from Step 2 (controller) and Step 3 (mission).

## Architecture Details

### Command Flow

```
multi_wayn.py (Sender)
    │
    ├─> Creates ROS 2 service clients
    │   - /drone_1/mission_command
    │   - /drone_2/mission_command
    │   - ... (one per drone)
    │
    └─> Sends commands:
        - ARM
        - TAKEOFF (x, y, z)
        - GOTO (x, y, z)
        - LAND
            │
            v
multi_drone.py (Receiver)
    │
    ├─> Receives commands via ROS 2 services
    │
    ├─> Translates to MAVSDK API calls
    │   - action.arm()
    │   - action.takeoff()
    │   - offboard.set_position_ned()
    │   - action.land()
    │
    └─> Publishes telemetry
        - /drone_X/position
        - /drone_X/armed
            │
            v
MAVSDK Server (gRPC)
    │
    └─> Communicates with PX4 SITL via UDP
            │
            v
PX4 SITL Instance
    │
    └─> Controls Gazebo drone model
```

### Key Files

- **run_multi_drone_gazebo.sh** - Launch script for Gazebo SITL
- **src/digi_rc/src/multi_drone.py** - Command receiver & MAVSDK controller
- **mock_mission/multi_wayn.py** - Mission planner & command sender
- **mavsdk_server** - MAVSDK gRPC server binary

### Design Principles

1. **Separation of Concerns**: 
   - `digi_rc` package = communication layer (ROS 2 controller)
   - `mock_mission` directory = mission planning (independent from digi_rc)
   - `multi_drone.py` = stable receiver (don't modify unless changing core functionality)
   - `multi_wayn.py` = flexible mission planner (customize missions here)

2. **Async Communication**:
   - ROS 2 services provide request/response pattern
   - MAVSDK uses async/await for drone operations

3. **Fault Tolerance**:
   - Each drone operates independently
   - Mission continues even if one drone fails
   - Position monitoring with timeout protection

## Advanced Usage

### Running with Fewer/More Drones

**Edit `run_multi_drone_gazebo.sh`:**

```bash
# Change -n parameter (number of drones)
"$PX4_DIR/Tools/simulation/gazebo-classic/sitl_multiple_run.sh" -m iris -n 3
```

**Update `multi_drone.py`:**

```python
# Change NUM_DRONES constant
NUM_DRONES = 3  # or your desired number
```

**Update `mock_mission/multi_wayn.py`:**

```python
# Adjust drone list
drones = [1, 2, 3]  # remove/add as needed
```

Rebuild digi_rc package after changes to multi_drone.py (not needed for mock_mission changes).

### Monitoring Drone Telemetry

```bash
# Listen to position updates
ros2 topic echo /drone_1/position

# Check armed status
ros2 topic echo /drone_1/armed

# List all drone topics
ros2 topic list | grep drone
```

### Manual Drone Control

You can send commands manually using ROS 2 CLI:

```bash
# Arm drone 1
ros2 service call /drone_1/mission_command digi_rc/srv/MissionCommand "{command: 'ARM', x: 0.0, y: 0.0, z: 0.0}"

# Takeoff to 5m
ros2 service call /drone_1/mission_command digi_rc/srv/MissionCommand "{command: 'TAKEOFF', x: 0.0, y: 0.0, z: 5.0}"

# Go to position (10, 5, 5)
ros2 service call /drone_1/mission_command digi_rc/srv/MissionCommand "{command: 'GOTO', x: 10.0, y: 5.0, z: 5.0}"

# Land
ros2 service call /drone_1/mission_command digi_rc/srv/MissionCommand "{command: 'LAND', x: 0.0, y: 0.0, z: 0.0}"
```

## Performance Tips

1. **Gazebo GUI Performance**: If simulation is slow, use headless mode:
   ```bash
   # Edit run_multi_drone_gazebo.sh, add --headless flag
   HEADLESS=1 ./run_multi_drone_gazebo.sh
   ```

2. **Faster Startup**: Create a systemd service to auto-start Gazebo on boot

3. **Network Latency**: All communication is localhost - ensure firewall allows UDP/TCP on ports 14540-14544 and 50041-50045

## Known Limitations

- Maximum tested: 5 drones (can scale to ~10-15 depending on hardware)
- Formation changes mid-flight not implemented
- No obstacle avoidance
- No GPS coordinate support (uses local NED frame only)
- Gazebo Classic only (not Gazebo Ignition)

## Support & Contribution

For issues or enhancements, check:
- PX4 documentation: https://docs.px4.io/
- MAVSDK Python: https://mavsdk-python.readthedocs.io/
- ROS 2 Humble: https://docs.ros.org/en/humble/

## License

[Add your license here]

## Configuration

### Drone Setup

- **Number of drones**: 5
- **UDP Ports**: 14541-14545 (Drone IDs 1-5)
- **MAVSDK gRPC Ports**: 50041-50045

### Port Mapping

| Drone ID | MAVLink UDP Port | MAVSDK gRPC Port | Gazebo Model |
|----------|------------------|------------------|--------------|
| 1        | 14541            | 50041            | iris_1       |
| 2        | 14542            | 50042            | iris_2       |
| 3        | 14543            | 50043            | iris_3       |
| 4        | 14544            | 50044            | iris_4       |
| 5        | 14545            | 50045            | iris_5       |

## ROS 2 Interface

### Services (per drone)

- `/drone_1/mission_command` through `/drone_5/mission_command`

Commands: `ARM`, `TAKEOFF`, `GOTO`, `LAND`

### Topics (per drone)

- `/drone_X/armed` (Bool) - Arm status
- `/drone_X/position` (Point) - Current NED position
- `/swarm_status` (String) - Overall swarm status

### Manual Command Examples

```bash
# Arm drone 1
ros2 service call /drone_1/mission_command digi_rc/srv/Command \
  "{command: 'ARM', x: 0.0, y: 0.0, z: 0.0}"

# Takeoff to 5 meters
ros2 service call /drone_1/mission_command digi_rc/srv/Command \
  "{command: 'TAKEOFF', x: 0.0, y: 0.0, z: 5.0}"

# Go to position (10m north, 5m east, 5m altitude)
ros2 service call /drone_1/mission_command digi_rc/srv/Command \
  "{command: 'GOTO', x: 10.0, y: 5.0, z: 5.0}"

# Land
ros2 service call /drone_1/mission_command digi_rc/srv/Command \
  "{command: 'LAND', x: 0.0, y: 0.0, z: 0.0}"
```

## Customizing Missions

Edit `src/digi_rc/mock_mission/multi_wayn.py` to customize missions:

**Option 1: Formation Flight** (default)
```python
loop.run_until_complete(swarm.run_formation_flight())
```

**Option 2: Individual Waypoint Missions**
```python
loop.run_until_complete(swarm.run_swarm_mission())
```

Modify waypoints in the `run_swarm_mission()` or `run_formation_flight()` methods.

## Architecture

- **multi_drone.py** - Receiver node that manages MAVSDK connections and executes commands
- **multi_wayn.py** - Mission planner that sends commands to control drones
- **Command flow**: multi_wayn.py → ROS 2 Services → multi_drone.py → MAVSDK → PX4 SITL

## Troubleshooting

**Ports already in use:**
```bash
pkill -9 px4 gzserver gzclient mavsdk_server python3
```

**Rebuild package:**
```bash
cd /home/bhushan-arc/skyflock_uav_arc
rm -rf build/digi_rc install/digi_rc
colcon build --packages-select digi_rc --symlink-install
source install/setup.bash
```

**Check Gazebo status:**
```bash
ps aux | grep -E "(gzserver|px4)" | grep -v grep
```

**View logs:**
```bash
tail -f gazebo.log
```

## Stopping the Simulation

```bash
pkill -9 px4 && pkill -9 gzserver && pkill -9 gzclient
```

Or press `Ctrl+C` in each terminal.

## Files

- `run_multi_drone_gazebo.sh` - Launch Gazebo with 5 drones
- `src/digi_rc/src/multi_drone.py` - Command receiver and MAVSDK controller
- `src/digi_rc/mock_mission/multi_wayn.py` - Mission planner and command sender
- `mavsdk_server` - MAVSDK server binary

## License

Apache 2.0
