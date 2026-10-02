# ROS 2 USV Autonomous Navigation Stack

An end-to-end autonomous navigation, perception, and control pipeline for Unmanned Surface Vehicles (USVs). Built on **ROS 2 Humble**, this repository provides a robust architecture integrating **ArduPilot (CubePilot)** via FastDDS/Micro-ROS, **Nav2** for path planning, and **OAK-D** for spatial AI and obstacle avoidance.

Designed for robust field operations, it features hardware-accelerated inference (TensorRT on NVIDIA Jetson), dynamic state machine supervision, and mathematically rigorous coordinate and frame transformations.

---

## 🧠 Core Architecture & Algorithms

### 1. Geodesy & Odometry Bridging
Seamlessly binds ArduPilot's MAVLink/DDS telemetry into the ROS 2 TF tree:
* **WGS84 to ENU Transformations:** Custom implementation of Bowring's formulas (`GEOtoECEF`, `ECEFtoENU`) for precise local planar projection (ENU) from global geodetic coordinates.
* **Twist Frame Correction (`odom_bridge.py`):** Solves the frame mismatch between ArduPilot (which outputs Twist in the world ENU frame) and ROS 2 Nav2 (which expects Twist in the `base_link` body frame) by applying real-time yaw rotation matrices to linear velocities.
* **Jump Detection:** Monitors continuous pose streams for GPS coordinate jumps or sudden yaw shifts (e.g., GCS "Set Home Here" commands) to prevent Nav2 teleportation issues.

### 2. High-Performance Perception (OAK-D & YOLOv8)
* **Host-Side TensorRT Inference:** Bypasses OAK-D on-device limitations by piping aligned RGB/Depth frames to the host (NVIDIA Jetson) and running YOLOv8 via `.engine` for maximum FPS.
* **Spatial Fusion:** Projects 2D bounding boxes onto the depth map, applying sanity checks (e.g., water reflection noise filtering by checking Mono depth vs. Stereo depth variances) to extract accurate 3D spatial coordinates.
* **Virtual Obstacle Injection:** Converts filtered spatial detections into `PointCloud2` and `LaserScan` messages, feeding directly into Nav2's Local and Global Costmaps.

### 3. Kinematic Navigation & Path Planning
* Integrates **Nav2** with `RegulatedPurePursuitController` (and DWB) for smooth, velocity-scaled lookahead trajectory tracking.
* **Dynamic Waypoint Generation:** Algorithmically generates semicircular or full-circular waypoints on the fly to navigate around target buoys using geometric offsets and tangent vector calculations.

### 4. State Machine & Safety Supervisor
* **Event-Driven Mission Control:** A robust Python-based state machine handling sequential tasks (Searching, Navigating, Blind Pass, Ghost Buoy Tracking) with timeout fallbacks.
* **Universal Safety Shield (`global_safety_node.py`):** Runs independently of the active task state. It continuously tracks critical obstacles and injects them into the costmap, ensuring collision avoidance even during manual overrides or task transitions.

---

## 📂 Project Structure

```text
ros2-usv-autonomous-nav/
├── communication/         # FastDDS profiles and MAVLink testing tools
├── compose/               # Docker & Docker Compose configurations for Jetson deployment
├── launch/                # ROS 2 Launch files (vision, navigation, state machine)
├── navigation/            # Nav2 configs, Odometry/Cmd_Vel bridges, GPS initialization
├── simulation/            # Gazebo world files and OAK-D / Nav2 mock nodes for testing
├── tasks/                 # State machine task logics (Task 1, 2, 3 and Base Gate logic)
├── utils/                 # Data classes, parameters, and math helper functions
├── vision/                # OAK-D camera node, TensorRT integration, Global Safety Node
├── data_logger.py         # Asynchronous telemetry and local costmap CSV/PNG logger
└── state_machine.py       # Main autonomy execution loop
