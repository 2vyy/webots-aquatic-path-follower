import numpy as np
import math

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    print("Warning: opencv-python not installed. Visualization disabled.")

class CostmapVisualizer:
    def __init__(self, costmap, scale=4):
        """
        Visualizer for Costmap2D using OpenCV.
        scale: Pixels per grid cell.
        """
        self.costmap = costmap
        self.scale = scale
        self.window_name = "Costmap Live View"
        
    def show(self, robot_pose, lidar_points=None, path_points=None, trajectory_points=None, waypoints=None):
        """
        Render the current state of the planner/costmap.
        
        robot_pose: (x, z, yaw) in World coordinates.
        lidar_points: List/Array of (x, z) points in World coordinates.
        path_points: Tuple of lists ([xs], [zs]) representing the planned path.
        trajectory_points: Tuple of lists ([xs], [zs]) representing the actual traversed path.
        """
        if not CV2_AVAILABLE:
            return

        grid_flipped = np.flipud(self.costmap.grid.T)
        
        # Create grayscale image
        grid_img = np.zeros_like(grid_flipped, dtype=np.uint8)
        grid_img[grid_flipped == -1] = 128  # Unknown = gray
        grid_img[grid_flipped == 0] = 255   # Free = white
        grid_img[grid_flipped == 1] = 0     # Occupied = black
        
        # Resize for visibility
        h, w = grid_img.shape
        viz_img = cv2.resize(grid_img, (w * self.scale, h * self.scale), interpolation=cv2.INTER_NEAREST)
        
        # Convert to BGR for color drawing
        viz_img = cv2.cvtColor(viz_img, cv2.COLOR_GRAY2BGR)
            
        if path_points:
            xs, zs = path_points
            if len(xs) > 1:
                pts = []
                for i in range(len(xs)):
                    px, py = self.world_to_pix(xs[i], zs[i])
                    pts.append((px, py))
                
                # Draw lines
                for i in range(len(pts) - 1):
                    cv2.line(viz_img, pts[i], pts[i+1], (0, 255, 0), 1)
        
        if trajectory_points:
            xs, zs = trajectory_points
            if len(xs) > 1:
                pts = []
                for i in range(len(xs)):
                    px, py = self.world_to_pix(xs[i], zs[i])
                    pts.append((px, py))
                
                # Draw lines
                for i in range(len(pts) - 1):
                    cv2.line(viz_img, pts[i], pts[i+1], (255, 0, 0), 2)

        rx_world, rz_world, r_yaw = robot_pose
        px, py = self.world_to_pix(rx_world, rz_world)
        
        cv2.circle(viz_img, (px, py), 5, (0, 0, 255), -1)
        
        # Heading Arrow
        # Assuming Yaw 0 = +Z (North), increasing CCW
        arrow_len = 0.6 # meters
        tip_x = rx_world + arrow_len * math.cos(r_yaw)
        tip_z = rz_world + arrow_len * math.sin(r_yaw)
        tx, ty = self.world_to_pix(tip_x, tip_z)
        
        cv2.line(viz_img, (px, py), (tx, ty), (0, 0, 255), 2)
        
        if lidar_points is not None and len(lidar_points) > 0:
            try:
                if isinstance(lidar_points, np.ndarray) and lidar_points.ndim == 2:
                    step = max(1, len(lidar_points) // 1000)
                    for i in range(0, len(lidar_points), step):
                        lx, ly = lidar_points[i, 0], lidar_points[i, 1]
                        lpx, lpy = self.world_to_pix(lx, ly)
                        cv2.circle(viz_img, (lpx, lpy), 2, (255, 255, 0), -1)  # Cyan
            except Exception as e:
                pass  # Ignore errors in debug visualization

        if waypoints:
            for i, wp in enumerate(waypoints):
                wx, wy, wyaw = wp
                px, py = self.world_to_pix(wx, wy)
                
                # Draw Circle
                cv2.circle(viz_img, (px, py), 6, (255, 0, 255), 2)
                
                # Draw Heading Arrow
                arrow_len = 0.5 
                tx = wx + arrow_len * math.cos(wyaw)
                ty = wy + arrow_len * math.sin(wyaw)
                tpx, tpy = self.world_to_pix(tx, ty)
                
                cv2.line(viz_img, (px, py), (tpx, tpy), (255, 0, 255), 2)
                
                # Label
                cv2.putText(viz_img, str(i+1), (px+5, py-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)
        
        cv2.imshow(self.window_name, viz_img)
        cv2.waitKey(1)

    def world_to_pix(self, wx, wz):
        """
        Convert World (x, z) to Image (col, row).
        """
        gx, gz = self.costmap.world_to_grid(wx, wz)
        
        # Clip to valid range
        gx = max(0, min(self.costmap.width - 1, gx))
        gz = max(0, min(self.costmap.height - 1, gz))
        
        # Coordinate mapping: Col is X (non-inverted), Row is inverted Z
        col = int(gx * self.scale)
        row = int((self.costmap.height - 1 - gz) * self.scale)
        
        return col, row

    def _draw_grid(self, img):
        h, w, _ = img.shape
        # Vertical lines
        for x in range(0, w, self.scale):
            cv2.line(img, (x, 0), (x, h), (200, 200, 200), 1)
        # Horizontal lines
        for y in range(0, h, self.scale):
            cv2.line(img, (0, y), (w, y), (200, 200, 200), 1)
