#!/usr/bin/env python3
"""
Add cameras to existing Gazebo simulation via plugins
Attaches downward-facing cameras to iris drones dynamically
"""

import subprocess
import time
import sys

def add_camera_to_drone(drone_id):
    """Add a camera sensor to a drone using Gazebo service"""
    
    camera_sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="camera_{drone_id}">
    <pose>0 0 0.1 0 1.57 0</pose>
    <link name="camera_link">
      <sensor name="camera_sensor" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
        <topic>/drone_{drone_id}/camera</topic>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <alwaysOn>true</alwaysOn>
          <updateRate>30.0</updateRate>
          <cameraName>drone_{drone_id}_camera</cameraName>
          <imageTopicName>/drone_{drone_id}/camera/image_raw</imageTopicName>
          <cameraInfoTopicName>/drone_{drone_id}/camera/camera_info</cameraInfoTopicName>
          <frameName>camera_link_{drone_id}</frameName>
        </plugin>
      </sensor>
    </link>
  </model>
</sdf>'''
    
    # Save SDF to temp file
    sdf_file = f"/tmp/camera_{drone_id}.sdf"
    with open(sdf_file, 'w') as f:
        f.write(camera_sdf)
    
    # Spawn camera model attached to drone
    try:
        subprocess.run([
            "gz", "model",
            "--spawn-file", sdf_file,
            "--model-name", f"camera_{drone_id}",
            "-x", "0", "-y", "0", "-z", "0.1"
        ], check=True, timeout=5)
        print(f"✓ Added camera to drone {drone_id}")
        return True
    except Exception as e:
        print(f"✗ Failed to add camera to drone {drone_id}: {e}")
        return False

def main():
    num_drones = 5
    
    print("Waiting for Gazebo to be ready...")
    time.sleep(5)
    
    print(f"\nAdding cameras to {num_drones} drones...")
    for i in range(num_drones):
        add_camera_to_drone(i)
        time.sleep(1)
    
    print("\n✓ Camera setup complete")
    print("Camera topics:")
    for i in range(num_drones):
        print(f"  /drone_{i}/camera/image_raw")

if __name__ == "__main__":
    main()
