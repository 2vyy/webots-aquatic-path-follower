import numpy as np
import math
import time
import sys
import os
from my_package.costmap_visualizer import CostmapVisualizer
from controller import Robot, Motor, Lidar, GPS, Compass
# Ensure the package directory is in the path so we can import my_package even if not sourced perfectly
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from my_package.hybrid_a_star import HybridAStarPlanner
from my_package.costmap import Costmap2D
from my_package.angle import angle_mod

class MyRobotDriver():
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        # x, y, heading
        self.waypoints = [
            [8.0, 0.0, 0.0]
        ]
        # setup costmap
        self.costmap = Costmap2D(x_min=-12, x_max=12, z_min=-12, z_max=12, resolution=0.1)
        self.visualizer = CostmapVisualizer(self.costmap, scale=4)
        # dubins/RS path constants
        self.v_max = 2.0
        self.curvature = 1.5
        self.omega_max = self.curvature * self.v_max
        self.path_step_size = 0.2
        self.wheel_base = 0.5
        # pure pursuit constants
        self.Ld_base = 0.3
        self.arrival_radius = 0.3
        # states
        self.last_target_idx = 0
        self.current_waypoint_idx = 0
        # store computed path
        self.full_path_x = []
        self.full_path_y = []
        self.full_path_yaw = []
        self.full_path_dir = []
        self.left_prop_motor = self.robot.getDevice('left_prop_motor')
        self.right_prop_motor = self.robot.getDevice('right_prop_motor')
        self.left_prop_motor.setPosition(float('inf'))
        self.right_prop_motor.setPosition(float('inf'))
        self.left_prop_motor.setVelocity(0.0)
        self.right_prop_motor.setVelocity(0.0)
        timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(timestep)
        # Lidar Init
        self.lidar = self.robot.getDevice('lidar')
        self.lidar.enable(timestep)
        # Pre-compute lidar angle array
        self.lidar_res = self.lidar.getHorizontalResolution()
        self.lidar_fov = self.lidar.getFov()
        indices = np.arange(self.lidar_res)
        self.lidar_angles = self.lidar_fov/2 - (indices / self.lidar_res) * self.lidar_fov
        # State
        self.startup_steps = 0
        self.startup_duration = 50  # Wait 50 steps before first plan
        self.replanning_timer = 0   # Timer for stopping before replan
        self.cusp_timer = 0         # Timer for stopping at cusps
        # Current Lidar frame for visualization
        self.current_lidar_xs = np.array([])
        self.current_lidar_ys = np.array([])
        # Trajectory history for visualization
        self.trajectory_x = []
        self.trajectory_y = []
        self.planner = HybridAStarPlanner(
            curvature=self.curvature,
            step_size=self.path_step_size,
            grid_resolution=0.2,
            collision_check_fn=self._check_collision_with_footprint
        )
        self._footprint_offsets = self._precompute_footprint_offsets()

    def _precompute_footprint_offsets(self):
        hl = 0.25  # Increased for sensitivity
        hw = 0.35  # Increased for sensitivity
        res = 0.1 # Check every 10cm inside the robot
        
        xs = np.arange(-hl, hl + 0.001, res)
        ys = np.arange(-hw, hw + 0.001, res)
        
        # Create a grid of points (filled rectangle)
        grid_x, grid_y = np.meshgrid(xs, ys)
        
        # Flatten to list of (x, y) tuples or just array
        # We need array (N, 2)
        pts = np.column_stack((grid_x.ravel(), grid_y.ravel()))
        return pts

    def visualize_now(self, cx, cy, cyaw):
        self.step_counter = getattr(self, 'step_counter', 0)
        if self.step_counter % 20 != 0:
            return

        lidar_pts = np.column_stack((self.current_lidar_xs, self.current_lidar_ys)) if len(self.current_lidar_xs) > 0 else None
        path_pts = (self.full_path_x, self.full_path_y) if len(self.full_path_x) > 0 else None
        traj_pts = (self.trajectory_x, self.trajectory_y) if len(self.trajectory_x) > 1 else None
        self.visualizer.show((cx, cy, cyaw), lidar_pts, path_pts, traj_pts, self.waypoints)

    def step(self):
        # update pose
        gps_values = self.gps.getValues()
        curr_x = gps_values[0]
        curr_y = gps_values[1]
        compass_values = self.compass.getValues()
        raw_yaw = math.atan2(compass_values[0], -compass_values[2])
        curr_yaw = angle_mod(raw_yaw + math.pi/2)
        # Track Trajectory for Visualization
        self.trajectory_x.append(curr_x)
        self.trajectory_y.append(curr_y)
        # Lidar Update & Mapping
        ranges = np.array(self.lidar.getRangeImage())
        min_r = self.lidar.getMinRange()
        max_r = self.lidar.getMaxRange()
        
        # identify valid hits (obstacles)
        hit_mask = (ranges > min_r) & (ranges < max_r) & np.isfinite(ranges)
        
        # identify clearing rays (free space)
        clearing_mask = (ranges >= max_r) | np.isinf(ranges)
        
        # combine both for processing
        process_mask = hit_mask | clearing_mask
        
        self.current_lidar_xs = np.array([])
        self.current_lidar_ys = np.array([])
        
        if np.any(process_mask):
            # Recalculate angles if needed
            if len(self.lidar_angles) != len(ranges):
                indices = np.arange(len(ranges))
                self.lidar_angles = self.lidar_fov/2 - (indices / len(ranges)) * self.lidar_fov
            
            # Select data
            proc_ranges = ranges[process_mask]
            proc_angles = self.lidar_angles[process_mask]
            
            # clamp ranges for clearing rays to max_r (or slightly less to be safe?)
            proc_ranges = np.minimum(proc_ranges, max_r)
            
            # transform to world
            angles_world = curr_yaw - math.pi/2 + proc_angles
            xs = curr_x + proc_ranges * np.cos(angles_world)
            ys = curr_y + proc_ranges * np.sin(angles_world)
            
            # store only hits for visualization (don't clutter view with max range points)
            self.current_lidar_xs = xs[hit_mask[process_mask]]
            self.current_lidar_ys = ys[hit_mask[process_mask]]
            
            self.step_counter = getattr(self, 'step_counter', 0)
            self.step_counter += 1
            
            if self.step_counter % 5 == 0:
                # Pass numpy array directly (N, 2)
                world_points = np.column_stack((xs, ys))
                robot_pose = (curr_x, curr_y, curr_yaw)
                self.costmap.update_from_lidar(robot_pose, world_points)
        
        # Handle Replanning Wait
        if self.replanning_timer > 0:
            self.replanning_timer -= 1
            self.left_prop_motor.setVelocity(0.0)
            self.right_prop_motor.setVelocity(0.0)
            if self.replanning_timer == 0:
                 print("Wait complete. Replanning now...", flush=True)
                 if not self.plan_to_current_waypoint(curr_x, curr_y, curr_yaw):
                      print("Replanning failed! Path is blocked and no solution found.", flush=True)
                      self.full_path_x = [] # Stop following the old invalid path
            # Don't proceed to path following while waiting
            # But continue visualizing
            self.visualize_now(curr_x, curr_y, curr_yaw)
            return

        if self.step_counter % 10 == 0:
            self.costmap.clean_noise()

        # Check if current path is blocked (More frequent check: every 2 steps)
        if self.step_counter % 2 == 0: 
            if len(self.full_path_x) > 0:
                # Check points ahead of the robot
                search_start = self.last_target_idx
                # Check up to 40 points ahead (approx 8m) - Look further!
                search_end = min(self.last_target_idx + 40, len(self.full_path_x))
                
                if search_start < search_end:
                    xs = self.full_path_x[search_start:search_end]
                    ys = self.full_path_y[search_start:search_end]
                    yaws = self.full_path_yaw[search_start:search_end]
                    
                    if self._check_collision_with_footprint(xs, ys, yaws):
                        print("Path blocked! Stopping to replan...", flush=True)
                        self.left_prop_motor.setVelocity(0.0)
                        self.right_prop_motor.setVelocity(0.0)
                        self.replanning_timer = 20 # Wait ~0.6s
                        return

        # Startup Phase
        if self.startup_steps < self.startup_duration:
            self.left_prop_motor.setVelocity(0.0)
            self.right_prop_motor.setVelocity(0.0)
            self.startup_steps += 1
            if self.startup_steps == self.startup_duration:
                print("Startup complete. Planning...", flush=True)
                self.costmap.inflate()
                self.plan_to_current_waypoint(curr_x, curr_y, curr_yaw)
            return
        # Path Following (Pure Pursuit)
        if len(self.full_path_x) < 2:
            self.left_prop_motor.setVelocity(0.0)
            self.right_prop_motor.setVelocity(0.0)
            return

        if self.current_waypoint_idx >= len(self.waypoints):
             self.left_prop_motor.setVelocity(0.0)
             self.right_prop_motor.setVelocity(0.0)
             return

        # Check if reached current waypoint
        current_wp = self.waypoints[self.current_waypoint_idx]
        dist_to_wp = math.sqrt((curr_x - current_wp[0])**2 + (curr_y - current_wp[1])**2)
        if dist_to_wp < self.arrival_radius:
            print(f"*** Reached Waypoint {self.current_waypoint_idx + 1} ***", flush=True)
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                print("*** Finished ***", flush=True)
                self.left_prop_motor.setVelocity(0.0)
                self.right_prop_motor.setVelocity(0.0)
                return
            self.plan_to_current_waypoint(curr_x, curr_y, curr_yaw)
            return
    # Find target point
        (target_idx, target_x, target_y, dist_to_cusp, cusp_idx) = self.find_target_path_point(curr_x, curr_y)
        # Gear
        desired_gear = 1
        if target_idx < len(self.full_path_dir):
            desired_gear = self.full_path_dir[target_idx]
        # Slow down if approaching cusp
        target_vel = self.v_max
        
        # Velocity Control Logic:
        # Only slow down if the CUSP is actually close.
        if dist_to_cusp < 1.0: # Start slowing down 5m away (smooth deceleration)
            # Linearly interpolate or clamp
            target_vel = max(0.2, min(self.v_max, dist_to_cusp))
        
        # cusp switching logic
        if dist_to_cusp < 0.1: # Close enough to switch
             if self.cusp_timer == 0:
                  print(f"Approaching cusp (dist={dist_to_cusp:.2f}). Stopping...", flush=True)
                  self.cusp_timer = 75 # start waiting
             
             if self.cusp_timer > 0:
                  self.cusp_timer -= 1
                  self.left_prop_motor.setVelocity(0.1)
                  self.right_prop_motor.setVelocity(0.1)
                  if self.cusp_timer == 0:
                       # wait done. advance index to jump exactly to the new gear segment.
                       print("Cusp wait done. Switching gear.", flush=True)
                       # cusp_idx is the index of the first point in the new gear.
                       # we align our tracking starting from there.
                       self.last_target_idx = cusp_idx
                  return # skip the rest while waiting
        else:
             self.cusp_timer = 0

        # Pure Pursuit
        target_angle = math.atan2(target_y - curr_y, target_x - curr_x)
        if desired_gear == -1:
            target_angle = angle_mod(target_angle + math.pi)
        angle_error = angle_mod(target_angle - curr_yaw)
        omega_desired = (target_vel * 2.0 * math.sin(angle_error)) / self.Ld_base
        omega_clamp = max(-self.omega_max, min(self.omega_max, omega_desired))
        left_vel = target_vel - omega_clamp * self.wheel_base
        right_vel = target_vel + omega_clamp * self.wheel_base
        if desired_gear == -1:
            left_vel, right_vel = -right_vel, -left_vel
        self.left_prop_motor.setVelocity(left_vel)
        self.right_prop_motor.setVelocity(right_vel)
        self.telemetry_counter = getattr(self, 'telemetry_counter', 0) + 1
        if self.telemetry_counter % 20 == 0:
            print(f"[CTRL] pos:({curr_x:.2f},{curr_y:.2f}) err:{math.degrees(angle_error):.0f}° val:{target_vel:.1f} d_cusp:{dist_to_cusp:.2f}", flush=True)
        # Visualization
        self.visualize_now(curr_x, curr_y, curr_yaw)

    def plan_to_current_waypoint(self, start_x, start_y, start_yaw):
        if self.current_waypoint_idx >= len(self.waypoints):
            return False
            
        goal = self.waypoints[self.current_waypoint_idx]
        goal_x, goal_y, goal_yaw = goal[0], goal[1], goal[2]
        
        path_x, path_y, path_yaw, path_dir, success = self.planner.plan(
            start_x, start_y, start_yaw,
            goal_x, goal_y, goal_yaw
        )
        
        if success:
             # Convert to numpy arrays for fast vectorization
             self.full_path_x = np.array(path_x)
             self.full_path_y = np.array(path_y)
             self.full_path_yaw = np.array(path_yaw)
             self.full_path_dir = np.array(path_dir)
             self.last_target_idx = 0
             
             # check why it might be reversing
             if len(self.full_path_dir) > 0:
                 print(f"DEBUG: Path start gears: {self.full_path_dir[:10]}", flush=True)
                 unique_gears = np.unique(self.full_path_dir)
                 print(f"DEBUG: Unique gears in path: {unique_gears}", flush=True)
                 
             return True
        else:
             print(f"Failed to plan to waypoint {self.current_waypoint_idx+1}", flush=True)
             return False

    def find_target_path_point(self, curr_x, curr_y):
        """
        finds the lookahead point for pure pursuit. 
        handles gear changes (cusps) by stopping before switching direction.
        Returns: target_idx, target_x, target_y, dist_to_cusp, cusp_idx
        """
        if len(self.full_path_x) == 0:
            return 0, 0, 0, float('inf'), -1

        # 1. find closest point on the path to the robot
        # only search a small window ahead to ensure efficient progress
        search_window = 50
        search_start = self.last_target_idx
        search_end = min(self.last_target_idx + search_window, len(self.full_path_x))
        
        # slice the arrays for the search window
        dx = self.full_path_x[search_start:search_end] - curr_x
        dy = self.full_path_y[search_start:search_end] - curr_y
        dists_sq = dx**2 + dy**2
        
        # find index of minimum distance within the slice
        min_idx_in_slice = np.argmin(dists_sq)
        closest_idx = search_start + min_idx_in_slice
        
        # 2. identify the current gear (direction of motion)
        current_gear = self.full_path_dir[closest_idx]
        
        # 3. find target point (fixed lookahead) on the remaining path
        # we start searching from the closest point onwards
        search_slice_x = self.full_path_x[closest_idx:]
        search_slice_y = self.full_path_y[closest_idx:]
        search_slice_dir = self.full_path_dir[closest_idx:]
        
        dx = search_slice_x - curr_x
        dy = search_slice_y - curr_y
        dists = np.sqrt(dx**2 + dy**2)
        
        # find first index where distance >= lookahead distance
        mask = dists >= self.Ld_base
        
        target_idx = len(self.full_path_x) - 1 # default to end of path
        
        # Cusp Detection
        gear_changes = np.where(search_slice_dir != current_gear)[0]
        
        limit_idx = len(search_slice_x) # relative to closest_idx
        dist_to_cusp = float('inf')
        cusp_idx = -1
        stop_at_cusp = False
        
        if len(gear_changes) > 0:
            # constraint: lookahead must stop before the gear change
            limit_idx = gear_changes[0]
            # Found a cusp!
            cusp_idx = closest_idx + limit_idx
            
            # distance to the cusp point
            cx = search_slice_x[limit_idx]
            cy = search_slice_y[limit_idx]
            dist_to_cusp = math.sqrt((cx - curr_x)**2 + (cy - curr_y)**2)
            
            stop_at_cusp = True
            
        # Find valid lookahead points WITHIN the same gear
        valid_lookaheads = np.where(mask[:limit_idx])[0]
        
        if len(valid_lookaheads) > 0:
            # found a valid point far enough ahead and before any gear switch
            slide_idx = valid_lookaheads[0]
            target_idx = closest_idx + slide_idx
        elif stop_at_cusp:
            # if constrained by a cusp and no far point found, aim for the cusp itself.
            # aim for point just before switch to guide smoothly to it
            target_idx = closest_idx + limit_idx - 1
            if target_idx < closest_idx: target_idx = closest_idx
        else:
            # no gear switch, but also no point far enough (approaching end of path)
            target_idx = len(self.full_path_x) - 1
            
        self.last_target_idx = closest_idx
        return target_idx, self.full_path_x[target_idx], self.full_path_y[target_idx], dist_to_cusp, cusp_idx

    def get_robot_footprint(self, x, y, yaw):
        hl = 0.35
        hw = 0.45
        
        # Determine sampling to ensure we don't skip obstacles > 0.1m
        res = 0.1
        n_w = int(math.ceil(2 * hw / res))
        n_l = int(math.ceil(2 * hl / res))
        
        # Generate local points (cache this if performance becomes an issue)
        local_pts = []
        
        # Front and Back edges (x is constant hl/-hl)
        ys = np.linspace(-hw, hw, n_w + 1)
        for yi in ys:
            local_pts.append((hl, yi))   # Front
            local_pts.append((-hl, yi))  # Back
            
        # Left and Right edges (y is constant hw/-hw)
        xs = np.linspace(-hl, hl, n_l + 1)
        for xi in xs:
            local_pts.append((xi, hw))   # Left
            local_pts.append((xi, -hw))  # Right
            
        # Transform to world coordinates
        c = math.cos(yaw)
        s = math.sin(yaw)
        
        corners_world = []
        for lx, ly in local_pts:
            wx = x + (lx * c - ly * s)
            wy = y + (lx * s + ly * c)
            corners_world.append((wx, wy))
            
        return corners_world

    def _check_collision_with_footprint(self, xs, ys, yaws):
        # check for all points in the trajectory chunk
        # xs, ys, yaws are lists from the planner
        
        # 1. Convert trajectory points to arrays
        traj_x = np.array(xs)
        traj_y = np.array(ys)
        traj_yaw = np.array(yaws)
        
        # 2. Prepare footprint (N_footprint, 2)
        footprint_local = np.array(self._footprint_offsets) 
        
        # 3. For each point in trajectory, transform footprint
        for i in range(len(traj_x)):
            x, y, yaw = traj_x[i], traj_y[i], traj_yaw[i]
            
            c, s = math.cos(yaw), math.sin(yaw)
            rot = np.array([[c, -s], [s, c]])
            
            world_pts = footprint_local @ rot.T + np.array([x, y])
            
            if self.costmap.check_collision(world_pts[:, 0], world_pts[:, 1]):
                return True
                
        return False