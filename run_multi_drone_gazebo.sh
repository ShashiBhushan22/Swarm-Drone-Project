#!/bin/bash

# Automated multi-drone mission launcher
# Launches Gazebo, controller, and mission in separate terminals

set -e

NUM_DRONES=5
PX4_DIR="$HOME/PX4-Autopilot"
WORKSPACE_DIR="/home/bhushan-arc/skyflock_uav_arc"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Automated Multi-Drone Mission       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# Verify PX4 directory
if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}✗ Error: PX4-Autopilot not found at $PX4_DIR${NC}"
    exit 1
fi

# Verify workspace
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo -e "${RED}✗ Error: Workspace not found at $WORKSPACE_DIR${NC}"
    exit 1
fi

echo -e "${GREEN}✓ PX4 Directory: $PX4_DIR${NC}"
echo -e "${GREEN}✓ Workspace: $WORKSPACE_DIR${NC}"
echo -e "${YELLOW}✓ Number of Drones: $NUM_DRONES${NC}"
echo ""

# Clean up any existing instances
echo -e "${YELLOW}⟳ Cleaning up existing instances...${NC}"
pkill -9 px4 2>/dev/null || true
pkill -9 gzserver 2>/dev/null || true  
pkill -9 gzclient 2>/dev/null || true
pkill -9 mavsdk_server 2>/dev/null || true
pkill -9 python3 2>/dev/null || true
sleep 2

cd "$PX4_DIR"

echo -e "${YELLOW}⟳ Step 1/3: Launching Gazebo with $NUM_DRONES drones...${NC}"
echo -e "${YELLOW}            (Waiting 30 seconds for initialization)${NC}"
echo ""

# Use PX4's multi-drone script with basic iris model (most stable)
Tools/simulation/gazebo-classic/sitl_multiple_run.sh -m iris -n $NUM_DRONES > /dev/null 2>&1 &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 30

# Check if Gazebo started successfully
if ! pgrep -x "gzserver" > /dev/null; then
    echo -e "${RED}✗ Error: Gazebo failed to start${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Gazebo running with $NUM_DRONES drones${NC}"
echo ""

# Launch multi_drone.py controller in new terminal
echo -e "${YELLOW}⟳ Step 2/3: Starting multi-drone controller...${NC}"
echo -e "${YELLOW}            (Waiting 15 seconds for connections)${NC}"
echo ""

gnome-terminal --title="Multi-Drone Controller" -- bash -c "
cd $WORKSPACE_DIR
source install/setup.bash
python3 install/digi_rc/lib/digi_rc/multi_drone.py
exec bash
" &

CONTROLLER_PID=$!
sleep 15

echo -e "${GREEN}✓ Controller initialized${NC}"
echo ""

# Launch orchestrator mission in new terminal
echo -e "${YELLOW}⟳ Step 3/3: Starting Orchestrator mission...${NC}"
echo ""

gnome-terminal --title="Orchestrator Mission" -- bash -c "
cd $WORKSPACE_DIR
source install/setup.bash
export PYTHONPATH=\$PYTHONPATH:$WORKSPACE_DIR
echo -e '${GREEN}════════════════════════════════════════${NC}'
echo -e '${GREEN}  ORCHESTRATOR: Circle Formation Flight${NC}'
echo -e '${GREEN}════════════════════════════════════════${NC}'
echo ''
python3 orchestrator/circle_mission.py
echo ''
echo -e '${YELLOW}Press Enter to close this window...${NC}'
read
" &

sleep 2

echo ""
echo -e "${GREEN}╔════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║    ✓ All Systems Launched!            ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}Active Components:${NC}"
echo -e "  ${GREEN}✓${NC} Gazebo Simulator (this terminal)"
echo -e "  ${GREEN}✓${NC} Multi-Drone Controller (new terminal)"
echo -e "  ${GREEN}✓${NC} Orchestrator Mission (new terminal)"
echo ""
echo -e "${BLUE}Drone Configuration:${NC}"
for i in $(seq 0 $(($NUM_DRONES - 1))); do
    UDP_PORT=$((14540 + $i))
    MAV_ID=$i
    echo -e "  ${YELLOW}Drone $MAV_ID${NC}: MAVLink UDP Port ${GREEN}$UDP_PORT${NC}"
done
echo ""
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo -e "${YELLOW}Watch the other terminal windows for:${NC}"
echo -e "  • Controller connection status"
echo -e "  • Mission execution progress"
echo -e "  • Formation flight completion"
echo -e "${BLUE}═══════════════════════════════════════${NC}"
echo ""
echo -e "${RED}Press Ctrl+C here to stop Gazebo${NC}"
echo -e "${YELLOW}(Controllers will stop automatically when mission completes)${NC}"
echo ""

# Keep running and handle cleanup on exit
trap 'echo -e "\n${YELLOW}Shutting down all components...${NC}"; pkill -9 px4; pkill -9 gzserver; pkill -9 gzclient; pkill -9 mavsdk_server; sleep 1; echo -e "${GREEN}✓ Cleanup complete${NC}"; exit 0' INT TERM

# Wait for Gazebo to finish
wait $GAZEBO_PID
