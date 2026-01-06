#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from digi_rc.srv import Command
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
import asyncio
import threading
import math

class Mavsdk:
    def __init__(self):
        self.drone = System()
        self.current_position = {"north": 0.0, "east": 0.0, "down": 0.0}
        self.is_offboard_active = False

    async def connect(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("MAVSDK: Connected to drone")
                break
    
    async def start_offboard(self):
        """Start offboard mode with initial setpoint"""
        print("MAVSDK: Starting offboard mode")
        # Set initial setpoint before starting offboard
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0)
        )
        try:
            await self.drone.offboard.start()
            self.is_offboard_active = True
            print("MAVSDK: Offboard mode started")
        except OffboardError as e:
            print(f"MAVSDK: Failed to start offboard mode: {e}")
            raise
            
    async def arm(self):
        print("MAVSDK: Arming drone")
        await self.drone.action.arm()
        if not self.is_offboard_active:
            await self.start_offboard()

    async def takeoff(self, height: float):
        """Takeoff using offboard position control"""
        print(f"MAVSDK: Taking off to {height} m")
        
        # Get current position
        async for pos in self.drone.telemetry.position_velocity_ned():
            self.current_position["north"] = pos.position.north_m
            self.current_position["east"] = pos.position.east_m
            self.current_position["down"] = pos.position.down_m
            break
        
        # Set target altitude (negative down for positive altitude)
        target_down = -height
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(
                self.current_position["north"],
                self.current_position["east"],
                target_down,
                0.0
            )
        )
        
        # Wait until we reach target altitude (with tolerance)
        while True:
            async for pos in self.drone.telemetry.position_velocity_ned():
                current_alt = -pos.position.down_m
                if abs(current_alt - height) < 0.5:  # 0.5m tolerance
                    print(f"MAVSDK: Reached takeoff altitude: {current_alt:.2f} m")
                    return
                break
            await asyncio.sleep(0.1)

    async def goto(self, x: float, y: float, z: float):
        """Move to position using offboard control
        x, y: relative position in meters (North, East)
        z: absolute altitude in meters (positive up from ground)
        """
        # Get current position
        async for pos in self.drone.telemetry.position_velocity_ned():
            current_north = pos.position.north_m
            current_east = pos.position.east_m
            current_down = pos.position.down_m
            break
        
        # Calculate target position
        target_north = current_north + x
        target_east = current_east + y
        
        # Z is absolute altitude (positive up), convert to NED down coordinate
        target_down = -z
        
        target_alt = -target_down  # Convert back to positive altitude for logging
        
        print(f"MAVSDK: Going to NED position: N={target_north:.2f}, E={target_east:.2f}, D={target_down:.2f} (Target alt={target_alt:.2f}m)")
        
        await self.drone.offboard.set_position_ned(
            PositionNedYaw(target_north, target_east, target_down, 0.0)
        )
        
        # Wait until we reach target position (with 0.5m tolerance on each axis)
        while True:
            async for pos in self.drone.telemetry.position_velocity_ned():
                dn = abs(pos.position.north_m - target_north)
                de = abs(pos.position.east_m - target_east)
                dd = abs(pos.position.down_m - target_down)
                
                current_alt = -pos.position.down_m
                print(f"MAVSDK: Distance to target - N:{dn:.2f}m E:{de:.2f}m D:{dd:.2f}m (Current alt:{current_alt:.2f}m, Target alt:{target_alt:.2f}m)")
                
                if dn < 0.5 and de < 0.5 and dd < 0.5:  # 0.5m tolerance
                    print(f"MAVSDK: Reached target position")
                    return
                break
            await asyncio.sleep(0.5)

    async def land(self):
        """Land using offboard mode then stop offboard"""
        print("MAVSDK: Landing drone")
        
        # Get current position
        async for pos in self.drone.telemetry.position_velocity_ned():
            current_north = pos.position.north_m
            current_east = pos.position.east_m
            break
        
        # Descend slowly to ground (0 altitude = down position from takeoff point)
        # We'll use velocity control for landing
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 1.0, 0.0)  # 1 m/s down
        )
        
        # Wait until landed
        async for landed in self.drone.telemetry.landed_state():
            if landed == 1:  # Landed state
                print("MAVSDK: Landed")
                break
            await asyncio.sleep(0.1)
        
        # Stop offboard mode
        if self.is_offboard_active:
            await self.drone.offboard.stop()
            self.is_offboard_active = False
            print("MAVSDK: Offboard mode stopped")

class Ros2(Node):
    def __init__(self, mavsdk: Mavsdk, loop: asyncio.AbstractEventLoop):
        super().__init__("ros2_node")
        self.mavsdk = mavsdk
        self.loop = loop
        self.current_cmd = ""
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_z = 0.0
        self.service = self.create_service(Command, "mission_command", self.service_callback)
        self.arm_pub = self.create_publisher(Bool, "armed", 10)
        self.pos_pub = self.create_publisher(Point, "drone_position", 10)

    def service_callback(self, request, response):
        self.current_cmd = request.command
        self.cmd_x = request.x
        self.cmd_y = request.y
        self.cmd_z = request.z
        self.get_logger().info(f"Received: {self.current_cmd} | ({self.cmd_x}, {self.cmd_y}, {self.cmd_z})")
        asyncio.run_coroutine_threadsafe(self.execute_command(), self.loop)
        response.response = "executed"
        return response

    async def execute_command(self):
        cmd = self.current_cmd
        self.get_logger().info(f"Executing command: {cmd}")
        try:
            if cmd == "ARM":
                await self.mavsdk.arm()
            elif cmd == "TAKEOFF":
                await self.mavsdk.takeoff(self.cmd_z)
            elif cmd == "GOTO":
                await self.mavsdk.goto(self.cmd_x, self.cmd_y, self.cmd_z)
            elif cmd == "LAND":
                await self.mavsdk.land()
            else:
                self.get_logger().error(f"Unknown command: {cmd}")
            self.get_logger().info(f"Command {cmd} executed")
        except Exception as e:
            self.get_logger().error(f"Error executing {cmd}: {e}")

    async def telemetry_task(self):
        async for armed in self.mavsdk.drone.telemetry.armed():
            msg = Bool()
            msg.data = armed
            self.arm_pub.publish(msg)

            async for posvel in self.mavsdk.drone.telemetry.position_velocity_ned():
                p = Point()
                p.x = posvel.position.north_m
                p.y = posvel.position.east_m
                p.z = -posvel.position.down_m 
                self.pos_pub.publish(p)
                break

            await asyncio.sleep(0.2)

def spin_ros(executor):
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

async def run_asyncio_forever():
    while True:
        await asyncio.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    mavsdk = Mavsdk()
    loop.run_until_complete(mavsdk.connect())
    node = Ros2(mavsdk, loop)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    asyncio.run_coroutine_threadsafe(node.telemetry_task(), loop)
    ros_thread = threading.Thread(target=spin_ros, args=(executor,), daemon=True)
    ros_thread.start()
    try:
        loop.run_until_complete(run_asyncio_forever())
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == "__main__":
    main()