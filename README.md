# Aquatic Robot Path Planner & Controller

A Webots simulation of a differential-drive aquatic robot capable of navigating complex obstacle environments with fluid dynamics. This project implements a Hybrid A* path planner for global navigation and a Pure Pursuit controller for trajectory tracking, handling both known static maps and unknown dynamic obstacles via Lidar.

![Simulation Demo](https://raw.githubusercontent.com/2vyy/webots-aquatic-path-follower/refs/heads/main/recording.gif)

## Technical Details

### Algorithms & Architecture
* **Path Planning:** Uses Hybrid A* with Reeds-Shepp curves (heuristic) to generate kinematically feasible, smooth paths for non-holonomic robots.
* **Control:** A tuned Pure Pursuit controller with adaptive lookahead logic and direct "gear" management for reversing maneuvers.
* **Mapping:** Real-time Costmap that integrates Lidar scans to manage obstacles and inflation layers.

### Edge Cases
* **Cusps (Reversing):** To navigate tight spaces, the controller implements a "gear switching" state machine. It stops, moves forward slowly to align, and reverses direction when the path requires a 3-point turn (cusp).
* **Dynamic Replanning:** If the path is blocked by a dynamic obstacle, the robot halts, inflates the costmap to the robot's footprint, and triggers a replan to find a new route.

### Known Nuances
* **Single-Node Latency:** The simulation step, perception, and control pipelines currently run within a single ROS 2 node. This introduces a slight but noticeable control latency compared to a distributed, multi-threaded architecture.
* **Visual LiDAR Artifacts:** When the robot turns and accelerates quickly (such as when oscilating into a stable heading), the fluid dynamics cause the vessel to pitch and roll very slightly. This briefly angles the 2D plane that the Lidar scans are projected onto, creating temporary lines of cells that are considered occupied in the costmap (as seen in the gif). These are overridden quickly by the following scans and the tradeoff benefit is more solid obstacle detection.

## Installation & Usage

1.  **Dependencies:**
    * Webots (R2023b or later recommended)
    * ROS 2 Iron (Humble might work but i haven't tested it)
    * Python 3.10+
    * `numpy`, `opencv-python`, `scipy`

2.  **Run:**
    ```./run_simulation.sh
    ```

## Credits
- Reference for fluid physics in WeBots: [silvery107/auto-docking-vessels](https://github.com/silvery107/auto-docking-vessels)
- Reeds-Shepp Path Python implementation: [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)