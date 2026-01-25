import numpy as np
import math

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

class Costmap2D:
    def __init__(self, x_min, x_max, z_min, z_max, resolution=0.1, robot_radius=0.1):
        """2D Occupancy Grid for collision checking."""
        self.x_min = x_min
        self.z_min = z_min
        self.resolution = resolution
        self.robot_radius = robot_radius
        
        # Number of cells to inflate by (robot radius in cells)
        self.inflate_cells = int(math.ceil(robot_radius / resolution))
        
        # Calculate grid dimensions
        self.width = int(math.ceil((x_max - x_min) / resolution))
        self.height = int(math.ceil((z_max - z_min) / resolution))
        
        # grid: -1=unknown, 0=free, 1=occupied
        self.grid = np.full((self.width, self.height), -1, dtype=np.int8) 

    def clean_noise(self):
        # Create a binary mask of occupied cells (0 or 1)
        occ = (self.grid == 1).astype(np.int8)
        padded = np.pad(occ, 1, mode='constant', constant_values=0)
        neighbors = (
            padded[:-2, :-2] + padded[:-2, 1:-1] + padded[:-2, 2:] +
            padded[1:-1, :-2]                    + padded[1:-1, 2:] +
            padded[2:, :-2]  + padded[2:, 1:-1]  + padded[2:, 2:]
        )
        noise_mask = (occ == 1) & (neighbors < 1)
        self.grid[noise_mask] = 0

    def clear_all(self):
        self.grid.fill(-1)

    def inflate(self):
        if self.inflate_cells <= 0: return
        occupied = np.argwhere(self.grid == 1)
        for gx, gz in occupied:
            for dx in range(-self.inflate_cells, self.inflate_cells + 1):
                for dz in range(-self.inflate_cells, self.inflate_cells + 1):
                    if dx*dx + dz*dz <= self.inflate_cells * self.inflate_cells:
                        nx, nz = gx + dx, gz + dz
                        if 0 <= nx < self.width and 0 <= nz < self.height:
                            self.grid[nx, nz] = 1

    def world_to_grid(self, wx, wz):
        gx = int((wx - self.x_min) / self.resolution)
        gy = int((wz - self.z_min) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        wx = self.x_min + gx * self.resolution
        wz = self.z_min + gy * self.resolution
        return wx, wz

    def check_collision(self, xs, ys):
        gxs = ((np.array(xs) - self.x_min) / self.resolution).astype(int)
        gys = ((np.array(ys) - self.z_min) / self.resolution).astype(int)
        valid = (gxs >= 0) & (gxs < self.width) & (gys >= 0) & (gys < self.height)
        if not np.all(valid):
            return True  # Out of bounds = collision
        return np.any(self.grid[gxs, gys] == 1)

    def update_from_lidar(self, robot_pose, world_points):
        rx, rz, ryaw = robot_pose
        start_gx, start_gy = self.world_to_grid(rx, rz)
        end_gxs = []
        end_gys = []
        for wx, wz in world_points:
            egx, egy = self.world_to_grid(wx, wz)
            end_gxs.append(egx)
            end_gys.append(egy)
        end_gxs = np.array(end_gxs)
        end_gys = np.array(end_gys)
        if CV2_AVAILABLE:
            # FAST METHOD: Draw lines on a temp image
            temp_img = np.zeros((self.height, self.width), dtype=np.uint8)
            for ex, ey in zip(end_gxs, end_gys):
                if 0 <= ex < self.width and 0 <= ey < self.height:
                    cv2.line(temp_img, (start_gx, start_gy), (ex, ey), 1, 1)
            clear_mask = temp_img == 1
            # Clear unknown cells only
            unknown_clear = (self.grid == -1) & clear_mask.T
            self.grid[unknown_clear] = 0
            # Mark hits as occupied (override anything)
            valid_mask = (end_gxs >= 0) & (end_gxs < self.width) & (end_gys >= 0) & (end_gys < self.height)
            self.grid[end_gxs[valid_mask], end_gys[valid_mask]] = 1
        else:
            # SLOW METHOD: Bresenham in Python
            for ex, ey in zip(end_gxs, end_gys):
                cells = self.bresenham_line(start_gx, start_gy, ex, ey)
                for cx, cy in cells[:-1]:
                    if 0 <= cx < self.width and 0 <= cy < self.height:
                        if self.grid[cx, cy] == -1:  # Only clear unknown
                            self.grid[cx, cy] = 0
                if len(cells) > 0:
                    cx, cy = cells[-1]
                    if 0 <= cx < self.width and 0 <= cy < self.height:
                        self.grid[cx, cy] = 1  # Mark hit occupied

    @staticmethod
    def bresenham_line(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
            cells.append((x, y))
        else:
            err = dy / 2.0
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
            cells.append((x, y))
            
        return cells