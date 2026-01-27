A Webots simulation of a differential-drive aquatic robot capable of navigating complex obstacle environments with fluid dynamics. This project implements a Hybrid A* path planner that uses Reeds-Shepp curves for branching heuristics and a Pure Pursuit controller, as well as processing LiDAR sensor data to dynamically update a costmap.

![Simulation Demo](https://raw.githubusercontent.com/2vyy/webots-aquatic-path-follower/refs/heads/main/recording.gif)

### Technical Details
- 3-point-turns or "cusps" are pretty tricky to handle with Pure Pursuit. The path planner returns a list of indexes where a "gear change" occurs and when the robot is close enough to one of these points, the controller logic switches to a special state that slow the robot down, stop it, and prepares for acceleration in the opposite direction.
- This project was started when I had much less ROS experience than I do now, so the controller, planning, visualization, and Lidar parsing subsystems are contained within a single ROS 2 node. Because of this, there are noticable lag spikes when replanning is triggered. I've done some brief experiments and utilizing distributed nodes makes a significant difference, but I unfortunately learned lesson that after writing the 3,000 lines of code in this project.
- The controller can handle a slight current force from the Fluid object (streamVelocity), but I've found it struggles to stay aligned to the path around >0.05 streamVelocity on the X and Y axes. There's likely some improvements to be made, but most of the error is that standard Pure Pursuit treats all angles of movement to be equal and doesn't account for force dynamics.
- When the robot turns too quickly (especially during a lag spike as explained above), the physics engine causes the boat to roll slightly. This causes the 2D plane that the Lidar scans on to be angled, creating temporary jagged lines of cells that are falsely parsed as occupied in the costmap (visible in the gif). These are usually overridden quickly by the following scans and the tradeoff benefit to less noise-filtering logic is more stable obstacle detection and permeance.

### Credits
- Reference for fluid physics in WeBots: [silvery107/auto-docking-vessels](https://github.com/silvery107/auto-docking-vessels)
- Reeds-Shepp Path Python implementation: [AtsushiSakai/PythonRobotics](https://github.com/AtsushiSakai/PythonRobotics)
