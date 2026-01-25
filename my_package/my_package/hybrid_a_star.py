import math
import heapq
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Callable

from my_package.reeds_shepp_path_planner import reeds_shepp_path_planning
from my_package.angle import angle_mod

@dataclass
class Node:
    """A node in the Hybrid A* search tree."""
    f_cost: float
    g_cost: float
    x: float
    y: float
    yaw: float
    direction: int # 1 for forward, -1 for backward
    steering: int # -1, 0, 1
    parent: Optional['Node'] = None
    
    path_x: list = field(default_factory=list)
    path_y: list = field(default_factory=list)
    path_yaw: list = field(default_factory=list)
    path_direction: list = field(default_factory=list)

    def __lt__(self, other):
        return self.f_cost < other.f_cost

class HybridAStarPlanner:
    def __init__(self, curvature=1.0, step_size=1.0, grid_resolution=0.5, 
                 yaw_resolution=np.deg2rad(10.0), collision_check_fn=None):
        
        self.curvature = curvature
        self.step_size = step_size
        self.grid_resolution = grid_resolution
        self.yaw_resolution = yaw_resolution
        self.collision_check_fn = collision_check_fn
        
        # Penalties
        self.steering_change_cost = 2.0 
        self.gear_change_cost = 2.0 
        self.steering_cost = 2.0
        self.reverse_cost = 0.5 

        self.rs_shot_interval = 5 
        
    def plan(self, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        start_yaw = angle_mod(start_yaw)
        goal_yaw = angle_mod(goal_yaw)
        
        open_set = []
        closed_set = {} 
        
        h_cost = self._calc_euclidean_heuristic(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)
        
        start_node = Node(
            f_cost=h_cost, g_cost=0.0,
            x=start_x, y=start_y, yaw=start_yaw, 
            direction=1, steering=0
        )
        
        heapq.heappush(open_set, start_node)
        
        iteration = 0
        max_iterations = 100000
        
        while open_set:
            iteration += 1
            if iteration > max_iterations:
                print(f"Hybrid A* failed: Max iterations ({max_iterations}) reached.")
                return [], [], [], [], False

            current = heapq.heappop(open_set)
            
            grid_idx = self._get_grid_index(current.x, current.y, current.yaw)
            if grid_idx in closed_set and closed_set[grid_idx] <= current.g_cost:
                continue
            closed_set[grid_idx] = current.g_cost
            
            # Analytic Expansion (Shot) using Reeds-Shepp
            if iteration % self.rs_shot_interval == 0:
                path = self._try_analytic_shot(current, goal_x, goal_y, goal_yaw)
                if path:
                    return path[0], path[1], path[2], path[3], True

            # Expand nodes
            for gear in [1, -1]:
                for steer_dir in [-1, 0, 1]:
                    steer_angle = steer_dir * self.curvature
                    
                    nx, ny, nyaw, traj_x, traj_y, traj_yaw, traj_dir = self._simulate_step(
                        current.x, current.y, current.yaw, gear, steer_angle
                    )
                    
                    if self.collision_check_fn:
                        if self.collision_check_fn(traj_x, traj_y, traj_yaw):
                            continue

                    step_cost = self.step_size
                    if steer_dir != 0: step_cost += self.steering_cost
                    if current.steering != steer_dir: step_cost += self.steering_change_cost
                    if current.direction != gear: step_cost += self.gear_change_cost
                    if gear == -1: step_cost += self.reverse_cost
                    
                    new_g = current.g_cost + step_cost
                    new_h = self._calc_euclidean_heuristic(nx, ny, nyaw, goal_x, goal_y, goal_yaw)
                    
                    new_node = Node(
                        f_cost=new_g + new_h,
                        g_cost=new_g,
                        x=nx, y=ny, yaw=nyaw,
                        direction=gear,
                        steering=steer_dir,
                        parent=current,
                        path_x=traj_x, path_y=traj_y, path_yaw=traj_yaw, path_direction=traj_dir
                    )
                    
                    heapq.heappush(open_set, new_node)
        
        return [], [], [], [], False

    def _calc_euclidean_heuristic(self, x, y, yaw, gx, gy, gyaw):
        """
        Heuristic: Distance + Heading Difference penalty.
        Encourages the robot to turn towards the goal early.
        Corrected for Reversing: Measures alignment with the LINE to the goal.
        """
        dist = math.hypot(gx - x, gy - y)
        angle_to_goal = math.atan2(gy - y, gx - x)
        
        # Difference if moving Forward
        diff_fwd = abs(angle_mod(angle_to_goal - yaw))
        # Difference if moving Backward (yaw + PI)
        diff_rev = abs(angle_mod(angle_to_goal - (yaw + math.pi)))
        
        # Take the best alignment (we can drive in either gear)
        angle_diff = min(diff_fwd, diff_rev)
        
        return dist + angle_diff * 2.0

    def _simulate_step(self, x, y, yaw, gear, steer):
        traj_x, traj_y, traj_yaw, traj_dir = [], [], [], []
        
        num_substeps = 5
        dt = self.step_size / num_substeps
        
        curr_x, curr_y, curr_yaw = x, y, yaw
        
        for _ in range(num_substeps):
            dist = gear * dt
            curr_x += dist * math.cos(curr_yaw)
            curr_y += dist * math.sin(curr_yaw)
            curr_yaw += steer * dist
            
            traj_x.append(curr_x)
            traj_y.append(curr_y)
            traj_yaw.append(curr_yaw)
            traj_dir.append(gear)
            
        curr_yaw = angle_mod(curr_yaw)
        return curr_x, curr_y, curr_yaw, traj_x, traj_y, traj_yaw, traj_dir

    def _try_analytic_shot(self, node, gx, gy, gyaw):
        """Try to connect directly to goal with a Reeds-Shepp path."""
        # [FIX] Use Reeds-Shepp to find a valid reverse/forward path to goal
        path_x, path_y, path_yaw, _, _, path_directions = reeds_shepp_path_planning(
            node.x, node.y, node.yaw, gx, gy, gyaw, 
            self.curvature, step_size=self.step_size
        )
        if path_x is None:
             return None
             
        if self.collision_check_fn:
            if self.collision_check_fn(path_x, path_y, path_yaw):
                return None
        
        if len(path_x) > 0:
            return self._reconstruct_path(node, path_x, path_y, path_yaw, path_directions)
        return None

    def _reconstruct_path(self, node, end_x, end_y, end_yaw, end_directions):
        path_x, path_y, path_yaw, path_dir = list(end_x), list(end_y), list(end_yaw), list(end_directions)
        
        curr = node
        while curr.parent:
            path_x = curr.path_x + path_x
            path_y = curr.path_y + path_y
            path_yaw = curr.path_yaw + path_yaw
            path_dir = curr.path_direction + path_dir
            curr = curr.parent
        
        # Post-process to filter out short cusps
        path_x, path_y, path_yaw, path_dir = self._filter_short_cusps(
            path_x, path_y, path_yaw, path_dir, min_segment_len=10
        )
            
        return path_x, path_y, path_yaw, path_dir

    def _filter_short_cusps(self, path_x, path_y, path_yaw, path_dir, min_segment_len=10):
        """
        Filter out direction changes that are too short to be meaningful.
        Short segments (< min_segment_len points) are merged with neighbors.
        """
        if len(path_dir) < min_segment_len:
            return path_x, path_y, path_yaw, path_dir
        
        # Find segment boundaries
        segments = []
        start = 0
        for i in range(1, len(path_dir)):
            if path_dir[i] != path_dir[i-1]:
                segments.append((start, i, path_dir[start]))
                start = i
        segments.append((start, len(path_dir), path_dir[start]))
        
        # Identify short segments to merge
        if len(segments) <= 1:
            return path_x, path_y, path_yaw, path_dir
        
        # Keep track of which direction each point should have
        new_dir = list(path_dir)
        
        for i, (start, end, direction) in enumerate(segments):
            seg_len = end - start
            if seg_len < min_segment_len:
                # Merge with dominant neighbor direction
                if i > 0:
                    prev_dir = segments[i-1][2]
                    for j in range(start, end):
                        new_dir[j] = prev_dir
                elif i < len(segments) - 1:
                    next_dir = segments[i+1][2]
                    for j in range(start, end):
                        new_dir[j] = next_dir
        
        return path_x, path_y, path_yaw, new_dir

    def _get_grid_index(self, x, y, yaw):
        ix = int(round(x / self.grid_resolution))
        iy = int(round(y / self.grid_resolution))
        iyaw = int(round(angle_mod(yaw) / self.yaw_resolution))
        return (ix, iy, iyaw)