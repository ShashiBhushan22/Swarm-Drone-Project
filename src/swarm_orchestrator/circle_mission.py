#!/usr/bin/env python3
"""
Circle Formation Mission with Drone Camera Capture
Arms 5 drones, flies them in a circle formation, and captures images from simulated downward cameras.
"""

import rclpy
import asyncio
import os
import cv2
import numpy as np
import threading
import time
from datetime import datetime
from src.swarm_orchestrator.command_interface import CommandInterface
from src.swarm_formations.generators import FormationGenerator

# Configuration
NUM_DRONES = 5
TAKEOFF_ALTITUDE = 20.0  # meters
CIRCLE_RADIUS = 15.0  # meters
CIRCLE_CENTER = (0.0, 0.0, TAKEOFF_ALTITUDE)  # (x, y, z)
HOLD_TIME = 30.0  # seconds - hold formation and capture images
IMAGE_SAVE_INTERVAL = 2.0  # seconds - save image every 2 seconds
INPUT_MAP_DIR = '/home/bhushan-arc/skyflock_uav_arc/input_map'


class CircleMissionOrchestrator:
    """Orchestrator for circle formation flight with simulated camera capture"""
    
    def __init__(self):
        self.cmd_interface = None
        self.image_capture_active = False
        self.captured_images = {}
        
        # Ensure input_map directory exists
        os.makedirs(INPUT_MAP_DIR, exist_ok=True)
        print(f"✓ Image save directory: {INPUT_MAP_DIR}")
        
    def start_camera_capture(self):
        """Start capturing simulated overhead views for each drone"""
        print("\n" + "="*60)
        print("INITIALIZING SIMULATED DRONE CAMERA CAPTURE")
        print("="*60)
        print("Simulating downward-facing cameras on each drone")
        
        self.image_capture_active = True
        
        # Start capture thread
        capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        capture_thread.start()
        
        print("✓ Camera capture initialized")
        
    def _generate_simulated_camera_view(self, drone_id, position):
        """Generate a simulated camera view based on drone position"""
        # Create a 640x480 image with position overlay
        img = np.ones((480, 640, 3), dtype=np.uint8) * 200
        
        # Add terrain-like pattern
        for i in range(0, 640, 50):
            for j in range(0, 480, 50):
                color = (150 + np.random.randint(-30, 30), 
                        180 + np.random.randint(-30, 30),
                        150 + np.random.randint(-30, 30))
                cv2.rectangle(img, (i, j), (i+50, j+50), color, -1)
        
        # Add crosshair
        cv2.line(img, (320-20, 240), (320+20, 240), (0, 0, 255), 2)
        cv2.line(img, (320, 240-20), (320, 240+20), (0, 0, 255), 2)
        
        # Add drone infopub
        info_text = [
            f"Drone {drone_id}",
            f"Alt: {position[2]:.1f}m",
            f"Pos: ({position[0]:.1f}, {position[1]:.1f})",
            f"Time: {datetime.now().strftime('%H:%M:%S')}"
        ]
        
        y_offset = 30
        for text in info_text:
            cv2.putText(img, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(img, text, (10, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
            y_offset += 30
        
        return img
    
    def _capture_loop(self):
        """Background thread to continuously capture simulated camera images"""
        capture_count = 0
        print(f"  ⟳ Starting simulated camera capture (saving to {INPUT_MAP_DIR})")
        
        # Simulated drone positions (will be updated from actual positions in real implementation)
        drone_positions = {
            i: (CIRCLE_RADIUS * np.cos(2 * np.pi * i / NUM_DRONES),
                CIRCLE_RADIUS * np.sin(2 * np.pi * i / NUM_DRONES),
                TAKEOFF_ALTITUDE)
            for i in range(NUM_DRONES)
        }
        
        while self.image_capture_active:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            for drone_id in range(NUM_DRONES):
                # Generate simulated camera view
                position = drone_positions[drone_id]
                image = self._generate_simulated_camera_view(drone_id, position)
                
                # Save image
                filename = f"drone_{drone_id}_camera_{timestamp}.jpg"
                filepath = os.path.join(INPUT_MAP_DIR, filename)
                cv2.imwrite(filepath, image)
                
                # Track captured images
                if drone_id not in self.captured_images:
                    self.captured_images[drone_id] = []
                self.captured_images[drone_id].append(filepath)
            
            capture_count += 1
            print(f"  ✓ Captured images from all {NUM_DRONES} drones (set #{capture_count})")
            
            time.sleep(IMAGE_SAVE_INTERVAL)
    
    def stop_camera_capture(self):
        """Stop camera capture"""
        self.image_capture_active = False
        time.sleep(1)
        print("\n✓ Camera capture stopped")
        
    async def run_mission(self):
        """Execute the complete circle formation mission"""
        print("\n" + "="*60)
        print("CIRCLE FORMATION MISSION WITH DRONE CAMERAS")
        print("="*60)
        print(f"Drones: {NUM_DRONES}")
        print(f"Formation: Circle (radius {CIRCLE_RADIUS}m)")
        print(f"Altitude: {TAKEOFF_ALTITUDE}m")
        print(f"Hold time: {HOLD_TIME}s")
        print(f"Camera capture interval: {IMAGE_SAVE_INTERVAL}s")
        print("="*60 + "\n")
        
        rclpy.init()
        self.cmd_interface = CommandInterface(NUM_DRONES)
        
        try:
            # Step 1: Arm all drones
            print("\n[1/6] ARMING DRONES...")
            print("-" * 60)
            arm_results = await self.cmd_interface.arm_all_drones()
            successful = sum(1 for success in arm_results.values() if success)
            print(f"✓ Armed {successful}/{NUM_DRONES} drones")
            await asyncio.sleep(2)
            
            # Step 2: Takeoff
            print("\n[2/6] TAKEOFF TO {:.1f}m...".format(TAKEOFF_ALTITUDE))
            print("-" * 60)
            takeoff_results = await self.cmd_interface.takeoff_all_drones(
                altitudes={i: TAKEOFF_ALTITUDE for i in range(1, NUM_DRONES + 1)}
            )
            successful = sum(1 for success in takeoff_results.values() if success)
            print(f"✓ {successful}/{NUM_DRONES} drones taking off")
            await asyncio.sleep(15)  # Wait for takeoff to complete
            
            # Step 3: Move to circle formation
            print("\n[3/6] MOVING TO CIRCLE FORMATION...")
            print("-" * 60)
            circle_positions = FormationGenerator.generate_circle_formation(
                num_drones=NUM_DRONES,
                center=CIRCLE_CENTER,
                radius=CIRCLE_RADIUS
            )
            
            print(f"Circle formation positions:")
            for drone_id, (x, y, z) in circle_positions.items():
                print(f"  Drone {drone_id}: ({x:.1f}, {y:.1f}, {z:.1f})")
            
            goto_results = await self.cmd_interface.send_formation_positions(circle_positions)
            successful = sum(1 for success in goto_results.values() if success)
            print(f"✓ {successful}/{NUM_DRONES} drones moving to formation")
            await asyncio.sleep(10)  # Wait for drones to reach formation
            
            # Step 4: Start drone camera capture once in formation
            print("\n[4/6] STARTING DRONE CAMERA CAPTURE...")
            print("-" * 60)
            self.start_camera_capture()
            await asyncio.sleep(3)
            
            # Step 5: Hold formation and capture camera images
            print("\n[5/6] HOLDING FORMATION AND CAPTURING IMAGES...")
            print("-" * 60)
            print(f"Simulating drone camera views for {HOLD_TIME} seconds...")
            print(f"Images saved to: {INPUT_MAP_DIR}")
            print("-" * 60)
            
            start_time = asyncio.get_event_loop().time()
            while asyncio.get_event_loop().time() - start_time < HOLD_TIME:
                elapsed = asyncio.get_event_loop().time() - start_time
            
            start_time = asyncio.get_event_loop().time()
            while asyncio.get_event_loop().time() - start_time < HOLD_TIME:
                elapsed = asyncio.get_event_loop().time() - start_time
                remaining = HOLD_TIME - elapsed
                print(f"\rHolding formation... {remaining:.1f}s remaining", end='', flush=True)
                await asyncio.sleep(1)
            print("\n✓ Formation hold complete")
            
            # Stop camera capture
            self.stop_camera_capture()
            await asyncio.sleep(1)
            
            # Print capture summary
            print("\n" + "="*60)
            print("CAMERA CAPTURE SUMMARY")
            print("="*60)
            total_images = sum(len(imgs) for imgs in self.captured_images.values())
            print(f"Total images captured: {total_images}")
            print(f"Images per drone:")
            for drone_id, images in self.captured_images.items():
                print(f"  Drone {drone_id}: {len(images)} images")
            print(f"Images location: {INPUT_MAP_DIR}")
            print("="*60 + "\n")
            
            # Step 6: Land all drones
            print("\n[6/6] LANDING ALL DRONES...")
            print("-" * 60)
            land_results = await self.cmd_interface.land_all_drones()
            successful = sum(1 for success in land_results.values() if success)
            print(f"✓ {successful}/{NUM_DRONES} drones landing")
            await asyncio.sleep(15)  # Wait for landing
            
            print("\n" + "="*60)
            print("✓ MISSION COMPLETED SUCCESSFULLY")
            print("="*60 + "\n")
            return True
            
        except KeyboardInterrupt:
            print("\n\n⚠ Mission interrupted by user")
            self.stop_camera_capture()
            print("Landing drones...")
            await self.cmd_interface.land_all_drones()
            await asyncio.sleep(5)
            return False
            
        except Exception as e:
            print(f"\n✗ Mission failed with error: {e}")
            import traceback
            traceback.print_exc()
            self.stop_camera_capture()
            print("Emergency landing...")
            await self.cmd_interface.land_all_drones()
            await asyncio.sleep(5)
            return False
            
        finally:
            if self.cmd_interface:
                self.cmd_interface.destroy_node()
            rclpy.shutdown()


async def main():
    """Main entry point"""
    print("\n" + "="*60)
    print("INITIALIZING CIRCLE FORMATION MISSION WITH DRONE CAMERAS")
    print("="*60)
    
    # Create orchestrator
    orchestrator = CircleMissionOrchestrator()
    
    # Run mission
    success = await orchestrator.run_mission()
    
    if success:
        print("\n✓ Mission execution completed")
    else:
        print("\n✗ Mission execution failed")


if __name__ == "__main__":
    asyncio.run(main())
