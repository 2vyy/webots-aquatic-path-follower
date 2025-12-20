import numpy as np
import math
from matplotlib.path import Path

class Costmap2D:
    def __init__(self, x_min, x_max, z_min, z_max, resolution=0.1):
        """2D Occupancy Grid for collision checking."""
        self.x_min = x_min
        self.z_min = z_min
        self.resolution = resolution
        
        # Calculate grid dimensions
        self.width = int(math.ceil((x_max - x_min) / resolution))
        self.height = int(math.ceil((z_max - z_min) / resolution))
        
        # grid: 0=free, 1=occupied
        self.grid = np.zeros((self.width, self.height), dtype=np.int8)
        
        self.static_obstacles = []

    def world_to_grid(self, x, z):
        """Convert world coordinates (meters) to grid indices (int)."""
        gx = int((x - self.x_min) / self.resolution)
        gz = int((z - self.z_min) / self.resolution)
        return gx, gz

    def grid_to_world(self, gx, gz):
        """Convert grid indices to world coordinates (center of cell)."""
        x = (gx * self.resolution) + self.x_min + (self.resolution / 2.0)
        z = (gz * self.resolution) + self.z_min + (self.resolution / 2.0)
        return x, z

    def add_static_polygon(self, vertices):
        """Burn obstacle into grid."""
        self.static_obstacles.append(vertices)
        
        # 1. create mpl path
        poly_path = Path(vertices)
        
        # 2. bounding box to limit search
        xs = [v[0] for v in vertices]
        zs = [v[1] for v in vertices]
        min_gx, min_gz = self.world_to_grid(min(xs), min(zs))
        max_gx, max_gz = self.world_to_grid(max(xs), max(zs))
        
        # clamp
        min_gx = max(0, min_gx)
        min_gz = max(0, min_gz)
        max_gx = min(self.width, max_gx + 1)
        max_gz = min(self.height, max_gz + 1)
        
        # 3. burn interval
        gx_range = np.arange(min_gx, max_gx)
        gz_range = np.arange(min_gz, max_gz)
        
        for gx in gx_range:
            for gz in gz_range:
                wx, wz = self.grid_to_world(gx, gz)
                if poly_path.contains_point((wx, wz)):
                    self.grid[gx, gz] = 1

    def add_point_cloud(self, points):
        """
        Add raw sensor data (Phase 2).
        points: List or array of (x, z) points.
        """
        for x, z in points:
            gx, gz = self.world_to_grid(x, z)
            if 0 <= gx < self.width and 0 <= gz < self.height:
                self.grid[gx, gz] = 1

    def check_collision(self, x_list, z_list, yaw_list=None):
        """
        Fast collision check for a trajectory.
        Compatible with Hybrid A* collision_check_fn signature.
        """
        # Vectorized check is possible, but loop is fast enough for grid lookup
        for x, z in zip(x_list, z_list):
            gx, gz = self.world_to_grid(x, z)
            
            # Check bounds
            if gx < 0 or gx >= self.width or gz < 0 or gz >= self.height:
                return True # Treat out-of-bounds as collision
            
            # Check occupancy
            if self.grid[gx, gz] == 1:
                return True
                
        return False