#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from std_msgs.msg import String , Bool
from digi_rc.srv import Command
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState
import asyncio
import threading
import subprocess
import signal
import os

class MavsdkDrone:
    def __init__(self, drone_id, udp_port, grpc_port):
        self.drone_id = drone_id
        self.udp_port = udp_port
        self.grpc_port = grpc_port
        self.drone = System(mavsdk_server_address="127.0.0.1", port=grpc_port)
        self.current_position = {"north": 0.0, "east": 0.0, "down": 0.0}
        self.is_offboard_active = False
        self.mavsdk_server = None

    def start_mavsdk_server(self):
        """Start MAVSDK server process"""
        print(f"Drone {self.drone_id}: Starting MAVSDK server on port {self.grpc_port}")
        self.mavsdk_server = subprocess.Popen(
            ["./mavsdk_server", "-p", str(self.grpc_port), f"udp://:{self.udp_port}"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

    def stop_mavsdk_server(self):
        """Stop MAVSDK server process"""
        if self.mavsdk_server:
            os.kill(self.mavsdk_server.pid, signal.SIGTERM)
            print(f"Drone {self.drone_id}: MAVSDK server stopped")


async def run_drone_mission(drone_id, target_position, udp_port, grpc_port, time_offset, altitude_offset):
    """Run complete mission for a single drone - matches original code structure"""
    try:
        print(f"Drone {drone_id}: Mission task started")
        drone = System(mavsdk_server_address="127.0.0.1", port=grpc_port)
        await drone.connect(system_address=f"udp://:{udp_port}")

        # Wait for connection
        print(f"Drone {drone_id}: Waiting for connection...")
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Drone {drone_id}: Connected")
                break

        # Wait for position estimate
        print(f"Drone {drone_id}: Waiting for position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print(f"Drone {drone_id}: Position estimate ready")
                break

        # Apply time offset for staggered launch
        if time_offset > 0:
            print(f"Drone {drone_id}: Waiting {time_offset}s before launch")
            await asyncio.sleep(time_offset)
        
        # Check if armable
        async for health in drone.telemetry.health():
            if health.is_armable:
                print(f"Drone {drone_id}: Ready to arm")
                break
            await asyncio.sleep(1)
        
        # Set initial setpoint and arm
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.5)
        
        try:
            await drone.action.arm()
            await drone.offboard.start()
            print(f"Drone {drone_id}: Armed and offboard started")
        except (Exception, OffboardError) as error:
            print(f"Drone {drone_id}: Failed to arm/start offboard: {error}")
            await drone.action.disarm()
            return

        # Takeoff
        takeoff_altitude = -5.0 - altitude_offset
        print(f"Drone {drone_id}: Taking off to {-takeoff_altitude}m")
        for i in range(51):
            z = (takeoff_altitude / 50) * i
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, z, 0.0))
            await asyncio.sleep(0.1)
        
        # Hold at takeoff altitude
        for _ in range(20):
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, takeoff_altitude, 0.0))
            await asyncio.sleep(0.1)
        
        # Move to target position
        target_x, target_y, target_z = target_position
        target_z = target_z - altitude_offset
        print(f"Drone {drone_id}: Moving to target ({target_x}, {target_y}, {target_z})")
        
        for i in range(101):
            progress = i / 100
            x = progress * target_x
            y = progress * target_y
            z = takeoff_altitude + progress * (target_z - takeoff_altitude)
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, 0.0))
            await asyncio.sleep(0.1)
        
        # Hold at target
        for _ in range(50):
            await drone.offboard.set_position_ned(PositionNedYaw(target_x, target_y, target_z, 0.0))
            await asyncio.sleep(0.1)
        
        
        print(f"Drone {drone_id}: Returning home")
        for i in range(101):
            progress = i / 100
            x = target_x * (1 - progress)
            y = target_y * (1 - progress)
            z = target_z + progress * (takeoff_altitude - target_z)
            await drone.offboard.set_position_ned(PositionNedYaw(x, y, z, 0.0))
            await asyncio.sleep(0.1)
        
        # Hold before landing
        for _ in range(10):
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, takeoff_altitude, 0.0))
            await asyncio.sleep(0.1)

        print(f"Drone {drone_id}: Landing")
        await drone.action.land()

        async for state in drone.telemetry.landed_state():
            if state == LandedState.ON_GROUND:
                print(f"Drone {drone_id}: Landed")
                break

        try:
            await drone.offboard.stop()
        except:
            pass

        await drone.action.disarm()
        print(f"Drone {drone_id}: Mission complete")
        
    except Exception as e:
        print(f"Drone {drone_id}: CRITICAL ERROR - {e}")
        import traceback
        traceback.print_exc()


class SwarmController(Node):
    def __init__(self):
        super().__init__("swarm_controller_node")
        self.num_drones = 5
        self.time_offset = 2
        self.mavsdk_servers = []
        self.drone_instances = {}
        self.loop = None  # Will be set in main()
        
        # Drone configuration - Start from drone 1 to avoid port conflicts
        self.drone_start_index = 1  # Skip drone 0
        self.altitude_offsets = [0.5 * i for i in range(self.num_drones)]
        self.target_positions = [
            (10.0, 0.0, -5.0),
            (10.0, 5.0, -5.0),
            (10.0, -5.0, -5.0),
            (15.0, 0.0, -5.0),
            (15.0, 5.0, -5.0),
        ]
        # Start from 14541 to skip 14540 which might be in use
        self.udp_ports = [14541 + i for i in range(self.num_drones)]
        self.grpc_ports = [50041 + i for i in range(self.num_drones)]
        
        # Publishers and services for each drone
        self.drone_services = {}
        self.armed_publishers = {}
        self.position_publishers = {}
        self.telemetry_timers = {}
        
        for i in range(self.num_drones):
            drone_id = i + 1
            # Create service for each drone
            self.drone_services[drone_id] = self.create_service(
                Command, 
                f"drone_{drone_id}/mission_command", 
                lambda req, resp, did=drone_id: self.handle_command(req, resp, did)
            )
            # Publishers for telemetry
            self.armed_publishers[drone_id] = self.create_publisher(Bool, f"drone_{drone_id}/armed", 10)
            self.position_publishers[drone_id] = self.create_publisher(Point, f"drone_{drone_id}/position", 10)
            # Create timer for telemetry (will start after connection)
            self.telemetry_timers[drone_id] = None
        
        # General swarm status publisher
        self.status_pub = self.create_publisher(String, "swarm_status", 10)
        
        # Start MAVSDK servers
        self.start_mavsdk_servers()
    
    def handle_command(self, request, response, drone_id):
        """Handle service command for specific drone"""
        self.get_logger().info(f"Drone {drone_id}: Received command {request.command}")
        
        # Execute command asynchronously using the event loop
        if self.loop:
            asyncio.run_coroutine_threadsafe(
                self.execute_drone_command(drone_id, request.command, request.x, request.y, request.z),
                self.loop
            )
        else:
            self.get_logger().error("Event loop not initialized!")
        
        response.response = "executing"
        return response
    
    async def execute_drone_command(self, drone_id, command, x, y, z):
        """Execute command for specific drone"""
        try:
            drone_index = drone_id - 1
            
            if command == "ARM":
                await self.arm_drone(drone_index)
            elif command == "TAKEOFF":
                await self.takeoff_drone(drone_index, z)
            elif command == "GOTO":
                await self.goto_drone(drone_index, x, y, z)
            elif command == "LAND":
                await self.land_drone(drone_index)
            else:
                self.get_logger().error(f"Drone {drone_id}: Unknown command {command}")
        except Exception as e:
            self.get_logger().error(f"Drone {drone_id}: Error executing {command}: {e}")
    
    async def arm_drone(self, drone_index):
        """Arm specific drone"""
        drone_id = drone_index + 1
        self.get_logger().info(f"Drone {drone_id}: Arming...")
        
        drone = self.drone_instances[drone_id]
        
        # Wait until armable
        self.get_logger().info(f"Drone {drone_id}: Checking if armable...")
        armable = False
        timeout = 30
        start = asyncio.get_event_loop().time()
        
        while not armable and (asyncio.get_event_loop().time() - start) < timeout:
            async for health in drone.telemetry.health():
                self.get_logger().info(f"Drone {drone_id}: Armable={health.is_armable}")
                if health.is_armable:
                    armable = True
                break
            if not armable:
                await asyncio.sleep(0.5)
        
        if not armable:
            self.get_logger().error(f"Drone {drone_id}: Not armable after timeout")
            return
        
        self.get_logger().info(f"Drone {drone_id}: Is armable, setting initial setpoint...")
        
        # Set initial setpoint and arm
        await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        await asyncio.sleep(0.5)
        
        self.get_logger().info(f"Drone {drone_id}: Attempting to arm and start offboard...")
        
        try:
            await drone.action.arm()
            self.get_logger().info(f"Drone {drone_id}: Armed")
            await drone.offboard.start()
            self.get_logger().info(f"Drone {drone_id}: Offboard started")
        except Exception as e:
            self.get_logger().error(f"Drone {drone_id}: Failed to arm/start offboard: {e}")
            import traceback
            traceback.print_exc()
    
    async def takeoff_drone(self, drone_index, altitude):
        """Takeoff specific drone"""
        drone_id = drone_index + 1
        self.get_logger().info(f"Drone {drone_id}: Taking off to {altitude}m")
        
        drone = self.drone_instances[drone_id]
        target_altitude = -altitude  # NED coordinates
        
        # Gradual ascent
        for i in range(51):
            z = (target_altitude / 50) * i
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, z, 0.0))
            await asyncio.sleep(0.1)
        
        # Hold at altitude
        for _ in range(20):
            await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, target_altitude, 0.0))
            await asyncio.sleep(0.1)
        
        self.get_logger().info(f"Drone {drone_id}: Takeoff complete")
    
    async def goto_drone(self, drone_index, x, y, z):
        """Move specific drone to position"""
        drone_id = drone_index + 1
        self.get_logger().info(f"Drone {drone_id}: Going to ({x}, {y}, {z})")
        
        drone = self.drone_instances[drone_id]
        
        # Get current position
        async for pos in drone.telemetry.position_velocity_ned():
            current_north = pos.position.north_m
            current_east = pos.position.east_m
            current_down = pos.position.down_m
            break
        
        # Calculate target
        target_north = current_north + x
        target_east = current_east + y
        target_down = -z  # Convert altitude to NED down
        
        # Smooth movement
        for i in range(101):
            progress = i / 100
            n = current_north + progress * (target_north - current_north)
            e = current_east + progress * (target_east - current_east)
            d = current_down + progress * (target_down - current_down)
            await drone.offboard.set_position_ned(PositionNedYaw(n, e, d, 0.0))
            await asyncio.sleep(0.1)
        
        # Hold at target
        for _ in range(50):
            await drone.offboard.set_position_ned(PositionNedYaw(target_north, target_east, target_down, 0.0))
            await asyncio.sleep(0.1)
        
        self.get_logger().info(f"Drone {drone_id}: Reached target")
    
    async def land_drone(self, drone_index):
        """Land specific drone"""
        drone_id = drone_index + 1
        self.get_logger().info(f"Drone {drone_id}: Landing")
        
        drone = self.drone_instances[drone_id]
        
        await drone.action.land()
        
        # Wait until landed
        async for state in drone.telemetry.landed_state():
            if state == LandedState.ON_GROUND:
                break
        
        # Stop offboard and disarm
        try:
            await drone.offboard.stop()
        except:
            pass
        
        await drone.action.disarm()
        self.get_logger().info(f"Drone {drone_id}: Landed and disarmed")
    
    async def connect_all_drones(self):
        """Connect to all drones"""
        for i in range(self.num_drones):
            drone_id = i + 1
            grpc_port = self.grpc_ports[i]
            udp_port = self.udp_ports[i]
            
            self.get_logger().info(f"Connecting Drone {drone_id}...")
            
            drone = System(mavsdk_server_address="127.0.0.1", port=grpc_port)
            await drone.connect(system_address=f"udp://:{udp_port}")
            
            # Wait for connection
            async for state in drone.core.connection_state():
                if state.is_connected:
                    self.get_logger().info(f"Drone {drone_id}: Connected")
                    break
            
            # Wait for position estimate
            async for health in drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    self.get_logger().info(f"Drone {drone_id}: Position estimate ready")
                    break
            
            self.drone_instances[drone_id] = drone
        
        # Start telemetry publishing using ROS2 timers
        self.get_logger().info("Starting telemetry publishing for all drones...")
        for drone_id in self.drone_instances.keys():
            self.telemetry_timers[drone_id] = self.create_timer(
                0.1,  # 10 Hz
                lambda did=drone_id: self.publish_telemetry_sync(did)
            )
            self.get_logger().info(f"Telemetry timer started for Drone {drone_id}")
        
        self.get_logger().info("All drones connected and ready")
    
    def publish_telemetry_sync(self, drone_id):
        """Publish telemetry synchronously using timer callback"""
        if self.loop and drone_id in self.drone_instances:
            asyncio.run_coroutine_threadsafe(
                self.get_telemetry_data(drone_id),
                self.loop
            )
    
    async def get_telemetry_data(self, drone_id):
        """Get and publish telemetry data for a drone"""
        try:
            drone = self.drone_instances[drone_id]
            
            # Get armed status
            async for armed in drone.telemetry.armed():
                msg = Bool()
                msg.data = armed
                self.armed_publishers[drone_id].publish(msg)
                break
            
            # Get position
            async for posvel in drone.telemetry.position_velocity_ned():
                pos = Point()
                pos.x = posvel.position.north_m
                pos.y = posvel.position.east_m
                pos.z = -posvel.position.down_m  # Convert NED to positive altitude
                self.position_publishers[drone_id].publish(pos)
                break
                
        except Exception as e:
            pass  # Silently ignore telemetry errors
    
    async def publish_telemetry(self, drone_id):
        """Continuously publish telemetry for a drone"""
        drone = self.drone_instances[drone_id]
        
        async for armed in drone.telemetry.armed():
            msg = Bool()
            msg.data = armed
            self.armed_publishers[drone_id].publish(msg)
            
            # Also publish position when we get armed status
            try:
                async for posvel in drone.telemetry.position_velocity_ned():
                    pos = Point()
                    pos.x = posvel.position.north_m
                    pos.y = posvel.position.east_m
                    pos.z = -posvel.position.down_m  # Convert NED to positive altitude
                    self.position_publishers[drone_id].publish(pos)
                    break
            except:
                pass
            
            await asyncio.sleep(0.1)

    def start_mavsdk_servers(self):
        """Start all MAVSDK servers"""
        self.get_logger().info(f"Starting {self.num_drones} MAVSDK servers")
        
        # Find mavsdk_server executable
        mavsdk_server_path = None
        possible_paths = [
            "./mavsdk_server",  # Current directory
            "/usr/local/bin/mavsdk_server",  # System install
            "/usr/bin/mavsdk_server",  # Alternative system install
            os.path.expanduser("~/mavsdk_server"),  # Home directory
            os.path.expanduser("~/Downloads/mavsdk_drone_show-0.2/mavsdk_server"),  # Your specific path
        ]
        
        for path in possible_paths:
            if os.path.exists(path):
                mavsdk_server_path = path
                self.get_logger().info(f"Found mavsdk_server at: {path}")
                break
        
        if not mavsdk_server_path:
            raise FileNotFoundError(
                "mavsdk_server not found. Please specify the correct path. "
                "Searched locations: " + ", ".join(possible_paths)
            )
        
        for i in range(self.num_drones):
            port = self.grpc_ports[i]
            mavsdk_server = subprocess.Popen(
                [mavsdk_server_path, "-p", str(port), f"udp://:{self.udp_ports[i]}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.mavsdk_servers.append(mavsdk_server)
            print(f"Started MAVSDK server {i} on port {port}")
        
        self.get_logger().info("All MAVSDK servers started")

    async def run_swarm_mission(self):
        """Execute mission for all drones - matches original code flow"""
        self.get_logger().info("Starting swarm mission")
        
        # Debug: Print configuration
        for i in range(self.num_drones):
            print(f"Drone {i}: UDP={self.udp_ports[i]}, gRPC={self.grpc_ports[i]}, Target={self.target_positions[i]}, AltOffset={self.altitude_offsets[i]}")
        
        # Create mission tasks for all drones (exactly like original code)
        tasks = []
        for i in range(self.num_drones):
            task = asyncio.create_task(
                run_drone_mission(
                    i,
                    self.target_positions[i],
                    self.udp_ports[i],
                    self.grpc_ports[i],
                    i * self.time_offset,
                    self.altitude_offsets[i]
                )
            )
            tasks.append(task)
        
        # Execute all missions
        await asyncio.gather(*tasks)
        
        self.get_logger().info("Swarm mission complete")
        
        # Publish completion status
        msg = String()
        msg.data = "Mission Complete"
        self.status_pub.publish(msg)

    def cleanup(self):
        """Cleanup all resources"""
        self.get_logger().info("Cleaning up MAVSDK servers...")
        for mavsdk_server in self.mavsdk_servers:
            os.kill(mavsdk_server.pid, signal.SIGTERM)


def spin_ros(executor):
    """Spin ROS2 executor"""
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass


def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    node = SwarmController()
    
    # Setup executor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Start ROS2 spinning in separate thread
    ros_thread = threading.Thread(target=spin_ros, args=(executor,), daemon=True)
    ros_thread.start()
    
    # Create event loop for async operations and store in node
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    node.loop = loop  # Store loop reference for command handling
    
    async def run_receiver():
        """Main receiver loop"""
        # Wait for MAVSDK servers to start
        await asyncio.sleep(2)
        
        # Connect all drones
        node.get_logger().info("Connecting all drones...")
        await node.connect_all_drones()
        node.get_logger().info("All drones connected. Ready to receive commands.")
        
        # Keep running and wait for commands from sender script
        node.get_logger().info("Waiting for commands from sender script...")
        while rclpy.ok():
            await asyncio.sleep(0.1)
    
    try:
        loop.run_until_complete(run_receiver())
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.cleanup()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()