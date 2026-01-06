#!/bin/bash
# Script to spawn iris drones with downward-facing cameras

PX4_DIR="$HOME/PX4-Autopilot"
NUM_DRONES=5

cd "$PX4_DIR"

# Clean up
pkill -9 px4 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true
pkill -9 gzclient 2>/dev/null || true
sleep 2

# Start Gazebo
HEADLESS=0 NO_PX4=1 make px4_sitl_default gazebo &
sleep 10

# Spawn each iris drone with camera plugin
for i in $(seq 0 $((NUM_DRONES - 1))); do
    x_pos=$((i * 3))
    
    # Spawn iris base
    gz model --spawn-file="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris/iris.sdf" \
        --model-name="iris_$i" \
        -x $x_pos -y 0 -z 0
    
    # Start PX4 instance
    DRONE_ID=$i ./Tools/simulation/gazebo-classic/sitl_gazebo-classic/scripts/jmavsim_run.sh -l -d $i &
    
    sleep 2
done

echo "✓ Spawned $NUM_DRONES drones with cameras"
