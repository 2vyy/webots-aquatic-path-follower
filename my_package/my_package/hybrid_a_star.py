"""
Hybrid A* Path Planner with Dubins Heuristic

Improvements over basic version:
1. Uses Dubins Path Length for Heuristic (prevents 'loops of shame').
2. Adds 'Steering Change Cost' (prevents wobbling/oscillation).
3. Optimized Node structure for memory efficiency.
"""
import math
import heapq
import numpy as np
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Callable

# Import your existing Dubins logic
from my_package.dubins_path_planner import plan_dubins_path
from my_package.angle import angle_mod

@dataclass
class Node:
    """A node in the Hybrid A* search tree."""
    f_cost: float
    g_cost: float
    x: float
    y: float
    yaw: float
    direction: int
    parent: Optional['Node'] = None
    
    # Path segments
    path_x: list = field(default_factory=list)
    path_y: list = field(default_factory=list)
    path_yaw: list = field(default_factory=list)

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
        
        # penalties
        self.steering_change_cost = 1.5 
        self.steering_cost = 0.1

        self.dubins_shot_interval = 5 # try direct path every N nodes
        
    def plan(self, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        """Main planning loop."""
        start_yaw = angle_mod(start_yaw)
        goal_yaw = angle_mod(goal_yaw)
        
        open_set = []
        closed_set = {} # Map grid_index -> g_cost (to allow re-visiting with better cost)
        
        # Initial Heuristic (Dubins)
        h_cost = self._calc_dubins_heuristic(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)
        
        start_node = Node(
            f_cost=h_cost, g_cost=0.0,
            x=start_x, y=start_y, yaw=start_yaw, direction=0
        )
        heapq.heappush(open_set, start_node)
        
        iteration = 0
        max_iterations = 100000
        while open_set:
            iteration += 1
            if iteration > max_iterations:
                print(f"Hybrid A* failed: Max iterations ({max_iterations}) reached.")
                return [], [], [], False

            current = heapq.heappop(open_set)
            
            # 1. Grid Pruning (Check if we found a better path to this cell already)
            grid_idx = self._get_grid_index(current.x, current.y, current.yaw)
            if grid_idx in closed_set and closed_set[grid_idx] <= current.g_cost:
                continue
            closed_set[grid_idx] = current.g_cost
            
            # 2. Analytic Expansion (Try to shoot straight to goal)
            if iteration % self.dubins_shot_interval == 0:
                path = self._try_analytic_shot(current, goal_x, goal_y, goal_yaw)
                if path:
                    return path[0], path[1], path[2], True

            # 3. expand neighbors
            # -1 (right), 0 (straight), 1 (left)
            for steer_dir in [-1, 0, 1]:
                steer_angle = steer_dir * self.curvature
                
                # simulate motion
                nx, ny, nyaw, traj_x, traj_y, traj_yaw = self._simulate_step(
                    current.x, current.y, current.yaw, steer_angle
                )
                
                # collision check
                if self.collision_check_fn and self.collision_check_fn(traj_x, traj_y, traj_yaw):
                    continue

                # calc costs
                # distance cost
                step_cost = self.step_size
                
                # b. Steering Cost (penalize turning slightly)
                if steer_dir != 0:
                    step_cost += self.steering_cost
                    
                # c. Change of Direction Cost (penalize zigzagging)
                if current.direction != steer_dir:
                    step_cost += self.steering_change_cost
                
                new_g = current.g_cost + step_cost
                
                # d. Heuristic Cost (Dubins)
                # Note: We use Dubins length as H. This guides us along valid curves.
                new_h = self._calc_dubins_heuristic(nx, ny, nyaw, goal_x, goal_y, goal_yaw)
                
                new_node = Node(
                    f_cost=new_g + new_h,
                    g_cost=new_g,
                    x=nx, y=ny, yaw=nyaw,
                    direction=steer_dir,
                    parent=current,
                    path_x=traj_x, path_y=traj_y, path_yaw=traj_yaw
                )
                
                heapq.heappush(open_set, new_node)
                
        return [], [], [], False

    def _calc_dubins_heuristic(self, x, y, yaw, gx, gy, gyaw):
        """Use Dubins path length as H cost for A* search."""
        # We assume empty space for the heuristic (no obstacles)
        # We only need the length, not the full path points
        _, _, _, mode, lengths = plan_dubins_path(
            x, y, yaw, gx, gy, gyaw, self.curvature, step_size=self.step_size
        )
        if mode is None:
            return float('inf') # Should not happen unless start/goal same
        return sum(lengths)

    def _simulate_step(self, x, y, yaw, steer):
        """Integrate motion for one step_size."""
        traj_x, traj_y, traj_yaw = [], [], []
        
        # Sub-steps for collision checking accuracy
        num_substeps = 5
        dt = self.step_size / num_substeps
        
        curr_x, curr_y, curr_yaw = x, y, yaw
        
        for _ in range(num_substeps):
            curr_x += dt * math.cos(curr_yaw)
            curr_y += dt * math.sin(curr_yaw)
            curr_yaw += steer * dt
            
            traj_x.append(curr_x)
            traj_y.append(curr_y)
            traj_yaw.append(curr_yaw)
            
        curr_yaw = angle_mod(curr_yaw)
        return curr_x, curr_y, curr_yaw, traj_x, traj_y, traj_yaw

    def _try_analytic_shot(self, node, gx, gy, gyaw):
        """Try to connect directly to goal with a Dubins path."""
        path_x, path_y, path_yaw, _, _ = plan_dubins_path(
            node.x, node.y, node.yaw, gx, gy, gyaw, 
            self.curvature, step_size=self.step_size,
            collision_check_fn=self.collision_check_fn
        )
        if len(path_x) > 0:
            return self._reconstruct_path(node, path_x, path_y, path_yaw)
        return None

    def _reconstruct_path(self, node, end_x, end_y, end_yaw):
        """Trace back from node to start, then append the analytic shot."""
        path_x, path_y, path_yaw = list(end_x), list(end_y), list(end_yaw)
        
        curr = node
        while curr.parent:
            # Prepend the segment (reverse order because we are walking back)
            path_x = curr.path_x + path_x
            path_y = curr.path_y + path_y
            path_yaw = curr.path_yaw + path_yaw
            curr = curr.parent
            
        return path_x, path_y, path_yaw

    def _get_grid_index(self, x, y, yaw):
        ix = int(round(x / self.grid_resolution))
        iy = int(round(y / self.grid_resolution))
        iyaw = int(round(angle_mod(yaw) / self.yaw_resolution))
        return (ix, iy, iyaw)

def plan_hybrid_astar(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw,
                      curvature=1.0, step_size=0.1, collision_check_fn=None):
    planner = HybridAStarPlanner(curvature=curvature, step_size=step_size, collision_check_fn=collision_check_fn)
    return planner.plan(start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw)