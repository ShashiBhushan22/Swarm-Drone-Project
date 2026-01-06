"""
Mission Command Interface

Provides a clean interface between the orchestrator and digi_rc communication layer.
Sends commands to drones via ROS 2 services without modifying digi_rc.
"""

import rclpy
from rclpy.node import Node
import asyncio
from typing import List, Dict, Tuple, Optional
from digi_rc.srv import Command


class CommandInterface(Node):
    """
    Interface to send commands from orchestrator to digi_rc controller.
    Uses ROS 2 service clients to communicate with multi_drone.py.
    """
    
    def __init__(self, num_drones: int = 5):
        super().__init__('orchestrator_command_interface')
        
        self.num_drones = num_drones
        self.service_clients = {}
        self.telemetry_subscribers = {}
        
        # Create service clients for each drone
        for drone_id in range(1, num_drones + 1):
            service_name = f'/drone_{drone_id}/mission_command'
            client = self.create_client(Command, service_name)
            self.service_clients[drone_id] = client
            
            self.get_logger().info(f'Created service client for {service_name}')
        
        # Wait for services to be available
        self._wait_for_services()
    
    def _wait_for_services(self, timeout: float = 10.0):
        """Wait for all drone services to become available"""
        self.get_logger().info('Waiting for drone services...')
        
        start_time = self.get_clock().now()
        all_ready = False
        
        while not all_ready and (self.get_clock().now() - start_time).nanoseconds / 1e9 < timeout:
            all_ready = True
            for drone_id, client in self.service_clients.items():
                if not client.service_is_ready():
                    all_ready = False
                    break
            
            if not all_ready:
                rclpy.spin_once(self, timeout_sec=0.1)
        
        if all_ready:
            self.get_logger().info('All drone services are ready')
        else:
            self.get_logger().warning('Timeout waiting for some drone services')
    
    async def send_command(
        self,
        drone_id: int,
        command: str,
        x: float = 0.0,
        y: float = 0.0,
        z: float = 0.0,
        timeout: float = 30.0
    ) -> bool:
        """
        Send a command to a specific drone.
        
        Args:
            drone_id: ID of the drone (1-based)
            command: Command type (ARM, TAKEOFF, GOTO, LAND)
            x, y, z: Position parameters
            timeout: Service call timeout in seconds
        
        Returns:
            True if command was successful, False otherwise
        """
        if drone_id not in self.service_clients:
            self.get_logger().error(f'Invalid drone_id: {drone_id}')
            return False
        
        client = self.service_clients[drone_id]
        
        if not client.service_is_ready():
            self.get_logger().warning(
                f'Service for drone {drone_id} is not ready'
            )
            return False
        
        # Create request
        request = Command.Request()
        request.command = command
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        
        self.get_logger().info(
            f'Sending to Drone {drone_id}: {command} ({x:.1f}, {y:.1f}, {z:.1f})'
        )
        
        try:
            # Call service asynchronously
            future = client.call_async(request)
            
            # Wait for response with timeout
            start_time = self.get_clock().now()
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
                if elapsed > timeout:
                    self.get_logger().error(
                        f'Timeout waiting for response from drone {drone_id}'
                    )
                    return False
            
            response = future.result()
            
            if response.response:
                self.get_logger().info(
                    f'Drone {drone_id}: Command {command} - {response.response}'
                )
                return True
            else:
                self.get_logger().error(
                    f'Drone {drone_id}: Command {command} failed'
                )
                return False
                
        except Exception as e:
            self.get_logger().error(
                f'Exception while sending command to drone {drone_id}: {str(e)}'
            )
            return False
    
    async def arm_drone(self, drone_id: int) -> bool:
        """Arm a specific drone"""
        return await self.send_command(drone_id, "ARM")
    
    async def takeoff_drone(
        self,
        drone_id: int,
        altitude: float,
        x: float = 0.0,
        y: float = 0.0
    ) -> bool:
        """Command drone to takeoff to specified altitude"""
        return await self.send_command(drone_id, "TAKEOFF", x, y, altitude)
    
    async def goto_position(
        self,
        drone_id: int,
        x: float,
        y: float,
        z: float
    ) -> bool:
        """Command drone to fly to specified position"""
        return await self.send_command(drone_id, "GOTO", x, y, z)
    
    async def land_drone(self, drone_id: int) -> bool:
        """Command drone to land"""
        return await self.send_command(drone_id, "LAND")
    
    async def arm_all_drones(self, drone_ids: Optional[List[int]] = None) -> Dict[int, bool]:
        """Arm multiple drones simultaneously"""
        if drone_ids is None:
            drone_ids = list(range(1, self.num_drones + 1))
        
        self.get_logger().info(f'Arming {len(drone_ids)} drones...')
        
        results = {}
        for drone_id in drone_ids:
            results[drone_id] = await self.arm_drone(drone_id)
            await asyncio.sleep(0.1)  # Small delay between commands
        
        successful = sum(1 for success in results.values() if success)
        self.get_logger().info(
            f'Armed {successful}/{len(drone_ids)} drones successfully'
        )
        
        return results
    
    async def takeoff_all_drones(
        self,
        drone_ids: Optional[List[int]] = None,
        altitudes: Optional[Dict[int, float]] = None
    ) -> Dict[int, bool]:
        """
        Takeoff multiple drones to specified altitudes.
        
        Args:
            drone_ids: List of drone IDs to takeoff
            altitudes: Dict mapping drone_id to target altitude
        
        Returns:
            Dict mapping drone_id to success status
        """
        if drone_ids is None:
            drone_ids = list(range(1, self.num_drones + 1))
        
        if altitudes is None:
            # Default staggered altitudes
            altitudes = {i: 5.0 + i for i in drone_ids}
        
        self.get_logger().info(f'Taking off {len(drone_ids)} drones...')
        
        results = {}
        for drone_id in drone_ids:
            altitude = altitudes.get(drone_id, 5.0)
            results[drone_id] = await self.takeoff_drone(drone_id, altitude)
            await asyncio.sleep(0.5)  # Delay between takeoffs
        
        successful = sum(1 for success in results.values() if success)
        self.get_logger().info(
            f'Took off {successful}/{len(drone_ids)} drones successfully'
        )
        
        return results
    
    async def send_formation_positions(
        self,
        positions: Dict[int, Tuple[float, float, float]]
    ) -> Dict[int, bool]:
        """
        Send GOTO commands to move drones to formation positions.
        
        Args:
            positions: Dict mapping drone_id to (x, y, z) position
        
        Returns:
            Dict mapping drone_id to success status
        """
        self.get_logger().info(
            f'Sending formation positions to {len(positions)} drones'
        )
        
        results = {}
        for drone_id, (x, y, z) in positions.items():
            results[drone_id] = await self.goto_position(drone_id, x, y, z)
            await asyncio.sleep(0.1)
        
        successful = sum(1 for success in results.values() if success)
        self.get_logger().info(
            f'Sent positions to {successful}/{len(positions)} drones successfully'
        )
        
        return results
    
    async def land_all_drones(
        self,
        drone_ids: Optional[List[int]] = None
    ) -> Dict[int, bool]:
        """Land multiple drones"""
        if drone_ids is None:
            drone_ids = list(range(1, self.num_drones + 1))
        
        self.get_logger().info(f'Landing {len(drone_ids)} drones...')
        
        results = {}
        for drone_id in drone_ids:
            results[drone_id] = await self.land_drone(drone_id)
            await asyncio.sleep(0.1)
        
        successful = sum(1 for success in results.values() if success)
        self.get_logger().info(
            f'Landed {successful}/{len(drone_ids)} drones successfully'
        )
        
        return results


async def test_command_interface():
    """Test function for command interface"""
    rclpy.init()
    
    interface = CommandInterface(num_drones=5)
    
    try:
        # Test arming all drones
        print("Testing ARM command...")
        arm_results = await interface.arm_all_drones([1, 2, 3])
        print(f"Arm results: {arm_results}")
        
        await asyncio.sleep(2)
        
        # Test takeoff
        print("\nTesting TAKEOFF command...")
        takeoff_results = await interface.takeoff_all_drones(
            drone_ids=[1, 2, 3],
            altitudes={1: 5.0, 2: 6.0, 3: 7.0}
        )
        print(f"Takeoff results: {takeoff_results}")
        
        await asyncio.sleep(10)
        
        # Test formation positions
        print("\nTesting GOTO command...")
        positions = {
            1: (10.0, 0.0, 5.0),
            2: (8.0, -2.0, 6.0),
            3: (8.0, 2.0, 6.0)
        }
        goto_results = await interface.send_formation_positions(positions)
        print(f"GOTO results: {goto_results}")
        
        await asyncio.sleep(10)
        
        # Test landing
        print("\nTesting LAND command...")
        land_results = await interface.land_all_drones([1, 2, 3])
        print(f"Land results: {land_results}")
        
    finally:
        interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(test_command_interface())
