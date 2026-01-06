# Satellite Map Integration - No Credit Card Needed!

Get real satellite imagery for your drone simulation using **completely free services**.

## Quick Start

### 1. Choose Your Location

Edit `setup_satellite_map.py`:

```python
MAP_CENTER_LAT = 37.7749      # San Francisco (change to your location)
MAP_CENTER_LON = -122.4194    
ZOOM_LEVEL = 18               # Higher = more detail (14-19)
```

Find coordinates at: https://www.google.com/maps (right-click → coordinates)

### 2. Download Free Satellite Map

```bash
python3 setup_satellite_map.py
```

Uses **ESRI World Imagery** - completely free, no registration!

### 3. Run Simulation

```bash
./run_multi_drone_gazebo.sh
```

Drones fly over real locations and save images to `input_map/`

## Popular Locations

```python
# Your picks:
MAP_CENTER_LAT = 40.7580; MAP_CENTER_LON = -73.9855  # Times Square, NYC
MAP_CENTER_LAT = 37.8199; MAP_CENTER_LON = -122.4783 # Golden Gate Bridge
MAP_CENTER_LAT = 25.1124; MAP_CENTER_LON = 55.1390   # Dubai Palm Islands
MAP_CENTER_LAT = 48.8584; MAP_CENTER_LON = 2.2945    # Eiffel Tower, Paris
```

## ✅ Completely Free
- No credit card
- No registration  
- No API key
- Global coverage
- Good quality

That's it! Just change coordinates and run.
