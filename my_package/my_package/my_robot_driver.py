import math
from my_package.dubins_path_planner import plan_dubins_path
from my_package.angle import angle_mod

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

class MyRobotDriver():
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        # x, y, heading
        self.waypoints = [
            [2.5, 2.5, -math.pi/2.0],
            [2.0, 2.0, 0.0],
            [-3, 4, -math.pi/4.0]
        ]
        # dubins path constants
        self.v_max = 0.75
        self.curvature = 0.5 
        self.omega_max = self.curvature * self.v_max
        self.path_step_size = 0.1
        self.wheel_base = 0.5

        # pure pursuit constants
        self.Ld = 0.1
        self.arrival_radius = 0.2

        # states
        self.last_target_idx = 0
        
        # store computed path
        self.full_path_x = []
        self.full_path_y = [] 
        self.full_path_yaw = []

        self.left_prop_motor = self.robot.getDevice('left_prop_motor')
        self.right_prop_motor = self.robot.getDevice('right_prop_motor')
        self.left_prop_motor.setPosition(float('inf'))
        self.right_prop_motor.setPosition(float('inf'))

        timestep = int(self.robot.getBasicTimeStep())
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(timestep)
        self.compass = self.robot.getDevice('compass')
        self.compass.enable(timestep)
        self.plan_path(0, 0, -math.pi/2) # angle orientation of dubins path graph is different from webots idk

    def step(self):
        # update pose
        gps_values = self.gps.getValues()
        curr_x = gps_values[0]
        curr_y = gps_values[1]
        compass_values = self.compass.getValues()
        curr_yaw = angle_mod(math.atan2(compass_values[0], compass_values[2]))
        
        # pure pursuit implementation
        # find closest point on path, get angle and dist displacement to it 
        (target_idx, target_x, target_y) = self.find_target_path_point(curr_x, curr_y) 

        # check if at final point (done?)
        final_x = self.full_path_x[-1]
        final_y = self.full_path_y[-1]
        dist_to_end = math.sqrt((curr_x - final_x)**2 + (curr_y - final_y)**2)
        if dist_to_end < self.arrival_radius:
            print(f"*** Reached End of Path ***")
            self.left_prop_motor.setVelocity(0.0)
            self.right_prop_motor.setVelocity(0.0)
            return

        angle_error = angle_mod(math.atan2(target_x - curr_x, target_y - curr_y) - curr_yaw)
        dist = math.sqrt((curr_x - target_x)**2 + (curr_y - target_y)**2)

        # get required angular velocity
        omega = (self.v_max * math.sin(angle_error)) / dist
        omega_clamp = max(-self.omega_max, min(self.omega_max, omega))
        self.left_prop_motor.setVelocity(self.v_max - omega_clamp * self.wheel_base)
        self.right_prop_motor.setVelocity(self.v_max + omega_clamp * self.wheel_base)
        
        print(f"Pose: (x:{curr_x:.2f}, y:{curr_y:.2f}, yaw:{math.degrees(curr_yaw):.1f}°) | "
              f"Target: (idx:{target_idx}, x:{target_x:.2f}, y:{target_y:.2f}) | "
              f"Angle Error: {math.degrees(angle_error):.1f}° | "
              f"Omega: {omega_clamp:.2f}", flush=True)

    def plan_path(self, plan_start_x, plan_start_y, plan_start_yaw):
        """
        Generates the full path by combining together Dubins paths
        from the robot's start pose to each subsequent waypoint.
        """
        print("--- Planning full route ---", flush=True)

        self.full_path_x.append(plan_start_x)
        self.full_path_y.append(plan_start_y)
        self.full_path_yaw.append(plan_start_yaw)

        for i, pose in enumerate(self.waypoints):
            goal_x, goal_y, goal_yaw = pose[0], pose[1], pose[2]
            print(f"Planning segment {i+1}: from ({plan_start_x:.1f}, {plan_start_y:.1f}) to ({goal_x:.1f}, {goal_y:.1f})")

            # for this pair of waypoints, get the dubins path between them and add them ontoo full_plot
            path_x, path_y, path_yaw, _, _ = plan_dubins_path(
                plan_start_x, plan_start_y, plan_start_yaw,
                goal_x, goal_y, goal_yaw,
                self.curvature,
                step_size=self.path_step_size
            )
            # skip first point bc it is the end of the previous path
            self.full_path_x.extend(path_x[1:])
            self.full_path_y.extend(path_y[1:])
            self.full_path_yaw.extend(path_yaw[1:])
            # set new start of next path
            plan_start_x = path_x[-1]
            plan_start_y = path_y[-1]
            plan_start_yaw = path_yaw[-1]
        self.generate_full_path_visualization()

    def find_target_path_point(self, curr_x, curr_y):
        """
        Finds the next target point on the path for the Pure Pursuit controller.
        
        - Find the path point *closest* to the robot.
        - From that closest point, search *forward* until a point is
           found that is at least `Ld` (lookahead distance) away.
        """
        closest_dist = float('inf')
        closest_idx = self.last_target_idx
        # start from last known index
        for i in range(self.last_target_idx, len(self.full_path_x)):
            dx = curr_x - self.full_path_x[i]
            dy = curr_y - self.full_path_y[i]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
            else:
                # if distance in increasing, there is no point checking more points
                break
        # search forward from found closest point for the lookahead target
        target_idx = closest_idx
        for i in range(closest_idx, len(self.full_path_x)):
            dx = curr_x - self.full_path_x[i]
            dy = curr_y - self.full_path_y[i]
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= self.Ld:
                target_idx = i
                break
        else:
            # resort to last point of path if no other choice
            target_idx = len(self.full_path_x) - 1
        
        self.last_target_idx = target_idx
        target_x = self.full_path_x[target_idx]
        target_y = self.full_path_y[target_idx]
        return target_idx, target_x, target_y
    
    # these are for generating the graph of the planned path
    # (you do have to run the program twice if you make a change to the computed path for it to render in webots)
    def _plot_arrow(self, ax, x, y, yaw, length=0.5, width=0.1, fc="r", ec="k"):
        if not isinstance(x, (float, int)): x = x[0]
        if not isinstance(y, (float, int)): y = y[0]
        if not isinstance(yaw, (float, int)): yaw = yaw[0]
        ax.arrow(x, y,
                 length * math.cos(yaw), length * math.sin(yaw),
                 fc=fc, ec=ec, head_width=width, head_length=width, zorder=5)
        ax.plot(x, y, 'o', color=fc, markersize=5, zorder=5) # Add a dot

    def generate_full_path_visualization(self):
        try:
            print("--- Generating full path visualization... ---", flush=True)
            fig, ax = plt.subplots(figsize=(8, 8)) 
            ax.set_xlim(-6, 6)
            ax.set_ylim(-6, 6)
            ax.set_aspect('equal')
            ax.grid(True, zorder=0, linestyle=':', color='gray')
            ax.plot(self.full_path_x, self.full_path_y, "b--", zorder=2, label="Planned Path")
            label_every_n_points = 2 
            for i in range(len(self.full_path_x)):
                if i > 0 and i % label_every_n_points == 0:
                    x = self.full_path_x[i]
                    y = self.full_path_y[i]
                    ax.plot(x, y, 'o', color='#00008B', markersize=3, zorder=3) # Dark blue dot                    
                    ax.text(x, y + 0.15, str(i), ha='center', va='bottom', 
                            fontsize=7, color='black', zorder=6)
            start_x = self.full_path_x[0]
            start_y = self.full_path_y[0]
            self._plot_arrow(ax, start_x, start_y, 
                             self.full_path_yaw[0], fc='g') # Green for start
            ax.text(start_x, start_y + 0.2, "Start (Idx 0)", ha='center', va='bottom', 
                    fontsize=9, color='g', zorder=6)
            for i, pose in enumerate(self.waypoints):
                x, y, yaw = pose[0], pose[1], pose[2]
                self._plot_arrow(ax, x, y, yaw, fc='r')
                ax.text(x + 0.3, y, f"Goal {i}", ha='center', va='bottom', 
                        fontsize=9, color='r', zorder=6)
            ax.text(0, -5.8, "X coordinate", 
                    ha='center', va='bottom', fontsize=10, zorder=10)
            ax.text(-5.8, 0, "Y coordinate", 
                    ha='left', va='center', rotation=90, fontsize=10, zorder=10)
            ticks = [-9, -8, -7, -6, -5, -4, -3, -2, -1, 1, 2, 3, 4, 5, 6, 7, 8, 9] # Skip 0
            ax.set_xticks(ticks)
            ax.set_yticks(ticks)
            ax.tick_params(axis='x', which='both', length=0, pad=-14, colors='gray', labelsize=8)
            ax.tick_params(axis='y', which='both', length=0, pad=-14, colors='gray', labelsize=8)
            ax.set_xlabel(None)
            ax.set_ylabel(None)
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.spines['bottom'].set_visible(False)
            ax.spines['left'].set_visible(False)
            fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
            image_path = "floor_texture.png"            
            fig.savefig(image_path, dpi=200, pad_inches=0) 
            plt.close(fig) # Close the specific figure            
            print(f"--- Path visualization saved to {image_path} ---", flush=True)
        except Exception as e:
            print(f"Error generating path visualization: {e}", flush=True)
