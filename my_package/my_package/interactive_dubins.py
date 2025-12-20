"""
Interactive Hybrid A* Path Planner GUI
"""
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backend_bases import MouseButton
from matplotlib.path import Path
from hybrid_a_star import plan_hybrid_astar

class InteractiveDubinsPlanner:
    def __init__(self):
        self.start = (-6.0, -6.0, np.deg2rad(45.0))
        self.goal = (6.0, 6.0, np.deg2rad(45.0))
        self.curvature = 0.5 # Wider turns for a boat
        
        self.obstacles = []
        self.obstacle_paths = []
        self.current_polygon = []
        
        # Boat footprint (for collision checking)
        self.footprint_radius = 0.5

        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_plot()
        
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)
        
        self.update_path()
        plt.show()

    def setup_plot(self):
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.ax.grid(True, linestyle=':', alpha=0.5)
        self.ax.set_title(f"Hybrid A* (Curvature {self.curvature})\nLeft-Click: Add Point | Right-Click: Close Poly")

    def check_collision(self, xs, ys, yaws=None):
        """Vectorized collision check using Matplotlib Paths."""
        if not self.obstacle_paths: return False
        
        # Convert trajectory lists to (N, 2) array for fast checking
        points = np.column_stack((xs, ys))
        
        for path in self.obstacle_paths:
            # contains_points returns a boolean array; np.any checks if any point hits
            if np.any(path.contains_points(points)):
                return True
        return False

    def update_path(self):
        self.ax.clear()
        self.setup_plot()
        
        # Draw Obstacles
        for poly in self.obstacles:
            patch = patches.Polygon(poly, closed=True, fc='gray', ec='black', alpha=0.5)
            self.ax.add_patch(patch)
            
        # draw current polygon
        if self.current_polygon:
            xs, ys = zip(*self.current_polygon)
            self.ax.plot(xs, ys, 'r--o')

        # plan path
        print("Planning...", end="", flush=True)
        path_x, path_y, path_yaw, success = plan_hybrid_astar(
            self.start[0], self.start[1], self.start[2],
            self.goal[0], self.goal[1], self.goal[2],
            curvature=self.curvature,
            collision_check_fn=self.check_collision
        )
        print("Done!")

        # Draw Path
        if success:
            self.ax.plot(path_x, path_y, 'b-', linewidth=2, label="Hybrid A*")
        else:
            self.ax.text(0, 0, "No Path Found", color='red', fontsize=20, ha='center')

        # Draw Start/Goal
        self.draw_arrow(self.start, 'g', 'Start')
        self.draw_arrow(self.goal, 'r', 'Goal')
        self.fig.canvas.draw()

    def draw_arrow(self, pose, color, label):
        x, y, yaw = pose
        self.ax.arrow(x, y, math.cos(yaw), math.sin(yaw), fc=color, ec='k', head_width=0.3)
        self.ax.text(x, y+0.5, label, color=color, ha='center')

    def on_click(self, event):
        if event.button == MouseButton.LEFT and event.inaxes:
            self.current_polygon.append((event.xdata, event.ydata))
            self.update_path()
        elif event.button == MouseButton.RIGHT and len(self.current_polygon) >= 3:
            self.obstacles.append(self.current_polygon)
            self.obstacle_paths.append(Path(self.current_polygon))
            self.current_polygon = []
            self.update_path()

    def on_key(self, event):
        if event.key == 'c':
            self.obstacles = []
            self.obstacle_paths = []
            self.current_polygon = []
            self.update_path()
        elif event.key == 'q':
            plt.close()

if __name__ == '__main__':
    InteractiveDubinsPlanner()