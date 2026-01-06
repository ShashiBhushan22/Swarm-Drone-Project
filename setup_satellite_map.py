#!/usr/bin/env python3
"""
Download and prepare satellite imagery for Gazebo terrain
Uses Google Maps Static API or open alternatives like OpenStreetMap
"""

import os
import requests
from PIL import Image
import numpy as np

# Configuration
MAP_CENTER_LAT = 37.7749  # San Francisco latitude (change to your location)
MAP_CENTER_LON = -122.4194  # San Francisco longitude
ZOOM_LEVEL = 18  # Higher = more detail (max 20)
MAP_SIZE = 2048  # Size in pixels
OUTPUT_DIR = '/home/bhushan-arc/skyflock_uav_arc/maps'

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)


def download_osm_satellite_tile(lat, lon, zoom, size):
    """
    Download satellite imagery using free OpenStreetMap/ESRI satellite tiles
    No API key or registration required!
    """
    
    print(f"Downloading satellite imagery for ({lat}, {lon}) at zoom {zoom}...")
    print("Using free ESRI World Imagery service (no registration needed)")
    
    try:
        # Calculate tile coordinates for slippy map
        x_tile = int((lon + 180.0) / 360.0 * (1 << zoom))
        y_tile = int((1.0 - np.log(np.tan(lat * np.pi / 180.0) + 
                      1.0 / np.cos(lat * np.pi / 180.0)) / np.pi) / 2.0 * (1 << zoom))
        
        # ESRI World Imagery - Free, no API key needed, good satellite imagery
        url = f"https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{zoom}/{y_tile}/{x_tile}"
        
        print(f"Downloading from: {url}")
        
        response = requests.get(url, timeout=10, headers={'User-Agent': 'Mozilla/5.0'})
        if response.status_code == 200:
            output_file = os.path.join(OUTPUT_DIR, 'satellite_map.png')
            with open(output_file, 'wb') as f:
                f.write(response.content)
            print(f"✓ Satellite image downloaded successfully!")
            print(f"✓ Saved to: {output_file}")
            return output_file
        else:
            print(f"✗ Failed to download: HTTP {response.status_code}")
            return None
            
    except Exception as e:
        print(f"✗ Error downloading map: {e}")
        return None


def create_sample_map():
    """Create a sample map image if API is not available"""
    print("Creating sample terrain map...")
    
    # Create a simple terrain-like image
    img = Image.new('RGB', (MAP_SIZE, MAP_SIZE), color=(139, 175, 139))
    
    # Add some features (roads, buildings, etc.)
    pixels = img.load()
    for i in range(MAP_SIZE):
        for j in range(MAP_SIZE):
            # Add variation
            noise = np.random.randint(-20, 20)
            r = max(0, min(255, 139 + noise))
            g = max(0, min(255, 175 + noise))
            b = max(0, min(255, 139 + noise))
            pixels[i, j] = (r, g, b)
    
    output_file = os.path.join(OUTPUT_DIR, 'sample_terrain.png')
    img.save(output_file)
    print(f"✓ Sample terrain saved to: {output_file}")
    return output_file


def create_gazebo_world_with_map(map_image_path):
    """Create a custom Gazebo world file with the satellite map as ground texture"""
    
    world_content = f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="satellite_map_world">
    
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane with satellite map texture -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>500 500</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://{map_image_path}</uri>
            </script>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Scene -->
    <scene>
      <ambient>0.8 0.8 0.8 1</ambient>
      <background>0.7 0.7 1.0 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
    
    <!-- GUI Camera Position -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>0 0 80 0 1.57 0</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    
  </world>
</sdf>
"""
    
    world_file = os.path.join(OUTPUT_DIR, 'satellite_map.world')
    with open(world_file, 'w') as f:
        f.write(world_content)
    
    print(f"✓ Gazebo world file created: {world_file}")
    return world_file


def main():
    """Main setup function"""
    print("\n" + "="*60)
    print("SATELLITE MAP SETUP FOR GAZEBO")
    print("="*60)
    print(f"Location: ({MAP_CENTER_LAT}, {MAP_CENTER_LON})")
    print(f"Zoom Level: {ZOOM_LEVEL}")
    print(f"Output: {OUTPUT_DIR}")
    print("="*60 + "\n")
    
    # Try to download satellite imagery
    map_image = download_osm_satellite_tile(MAP_CENTER_LAT, MAP_CENTER_LON, ZOOM_LEVEL, MAP_SIZE)
    
    # If download fails, create sample map
    if not map_image:
        print("\n⚠ Satellite download failed. Using sample terrain instead.")
        print("This may happen due to:")
        print("  - Network connectivity issues")
        print("  - Rate limiting from tile server")
        print("  - Try again in a few minutes")
        print("")
        map_image = create_sample_map()
    
    # Create Gazebo world file with the map
    world_file = create_gazebo_world_with_map(map_image)
    
    print("\n" + "="*60)
    print("✓ SETUP COMPLETE")
    print("="*60)
    print(f"Map image: {map_image}")
    print(f"World file: {world_file}")
    print("\nTo use in simulation:")
    print("  Export GAZEBO_MODEL_PATH and load the world file")
    print("="*60 + "\n")


if __name__ == "__main__":
    main()
