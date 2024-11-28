
# Autonomous Navigation Workspace

This repository contains **four ROS 2 packages** designed to enable autonomous navigation by performing lane detection, GPS transformation, pathfinding, and navigation command execution. Below is a brief description of each package along with their respective commands.

---

## 1. Lane Detection (`lane_detection`)
This package processes camera images to detect lanes. It performs binary thresholding and DBSCAN clustering to identify lane markings. The outputs include:  
- **Largest Lane Image:** An image showing the largest detected lane.  
- **Combined Lane Image:** An image with both lanes if detected.  
- **Lane Width:** Published only when both lanes are detected.  

### Command:
```bash
ros2 run lane_detection lane_finder
```

---

## 2. GPS Transformer (`gps_transformer`)
This package transforms GPS coordinates (specifically for a no man's land scenario) into map coordinates for navigation.

### Commands:
1. **Publish GPS Coordinates:**  
   ```bash
   ros2 run gps_transformer gps_publisher
   ```
2. **Transform GPS to Map Coordinates:**  
   ```bash
   ros2 run gps_transformer transformer
   ```

---

## 3. Path Finder (`path_finder`)
This package is responsible for generating navigation paths. It utilizes:
- **Inverse Perspective Mapping (IPM):** To rectify and process the lane image.  
- **Curve Fitting:** To generate smooth paths for navigation.  

### Commands:
1. **Run IPM:**  
   ```bash
   ros2 run path_finder ipm
   ```
2. **Publish Path using Curve Fitting:**  
   ```bash
   ros2 run path_finder path_publisher.py
   ```

---

## 4. Navigation Commander (`nav_commander`)
This package handles path publishing for the navigation stack (`nav2`). It supports two modes:  
- **Normal Path Publishing:** Sends paths to the navigation stack.  
- **Path Publishing with GPS Integration:** Combines GPS data with pathfinding for enhanced navigation.  

### Commands:
1. **Publish Path to `nav2`:**  
   ```bash
   ros2 run nav_commander go
   ```
2. **Publish Path with GPS Integration:**  
   ```bash
   ros2 run nav_commander go_with_gps
   ```

---
