import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- CONFIGURATION ---
TARGET_SPEED = 2.0      # m/s (Boat speed through water)
CURRENT_SPEED = 1.0     # m/s (Water speed)
CURRENT_DIR = -np.pi/2  # South (-90 degrees)

# Target Path (We want to move straight East)
TARGET_COURSE = 0.0     # East (0 degrees)

# Simulation Step
DT = 0.1
FRAMES = 200

class BoatSimulator:
    def __init__(self):
        self.state = np.array([0.0, 0.0, 0.0]) # x, y, heading (yaw)
        self.path_x = []
        self.path_y = []

    def update(self, thrust_speed, target_course_ground):
        # 1. Decompose Current Vector
        v_current = np.array([
            CURRENT_SPEED * np.cos(CURRENT_DIR),
            CURRENT_SPEED * np.sin(CURRENT_DIR)
        ])

        # 2. THE CONTROL ALGORITHM (Ferry Gliding Math)
        # We need to find a Heading (psi) such that our y-velocity cancels the current
        # V_boat_y + V_current_y = 0  (to stay on horizontal line)
        # speed * sin(psi) + current_y = 0
        # sin(psi) = -current_y / speed
        
        # Safety: Check if current is too strong for the boat
        sin_beta = -v_current[1] / thrust_speed
        
        # If current is faster than boat, we can't fully compensate (drift happens)
        if abs(sin_beta) > 1.0:
            print("Current too strong! Drifting...")
            beta = np.sign(sin_beta) * np.pi/2 # Max effort (90 degrees)
        else:
            beta = np.arcsin(sin_beta)
        
        # Apply the crab angle to our target course
        desired_heading = target_course_ground + beta
        
        # Smoothly turn towards desired heading (Simple P-Controller for steering)
        error = desired_heading - self.state[2]
        self.state[2] += error * 0.1 # Turn rate limit
        
        # 3. Calculate Boat Velocity (Relative to Water)
        v_boat = np.array([
            thrust_speed * np.cos(self.state[2]),
            thrust_speed * np.sin(self.state[2])
        ])

        # 4. Calculate Ground Velocity (Vector Addition)
        v_ground = v_boat + v_current

        # 5. Integrate Position
        self.state[0] += v_ground[0] * DT
        self.state[1] += v_ground[1] * DT
        
        # Store history for plotting
        self.path_x.append(self.state[0])
        self.path_y.append(self.state[1])
        
        return v_ground, v_boat, v_current

# --- VISUALIZATION SETUP ---
sim = BoatSimulator()
fig, ax = plt.subplots(figsize=(10, 6))
ax.set_xlim(-2, 50)
ax.set_ylim(-15, 15)
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_title("Ferry Gliding: Using Physics to Drift Straight")
ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")

# Drawing Elements
boat_shape, = ax.plot([], [], 'k-', linewidth=2, label='Boat Hull')
path_line, = ax.plot([], [], 'g--', alpha=0.5, label='Actual Path (Ground)')
vector_current = ax.quiver(0, 0, 0, 0, color='blue', scale=10, label='Current')
vector_thrust = ax.quiver(0, 0, 0, 0, color='red', scale=10, label='Thrust')
vector_result = ax.quiver(0, 0, 0, 0, color='green', scale=10, label='Resultant Velocity')

# Add legend manually to avoid quiver issues
ax.legend(loc='upper left')

def init():
    return boat_shape, path_line

def animate(i):
    # Run Physics Step
    v_ground, v_boat, v_current = sim.update(TARGET_SPEED, TARGET_COURSE)
    
    x, y, psi = sim.state
    
    # 1. Draw Boat Hull (Triangle oriented by heading)
    # Simple geometry rotation
    L = 2.0 # Boat length
    W = 1.0 # Boat width
    hull_x = [L/2, -L/2, -L/2, L/2]
    hull_y = [0, W/2, -W/2, 0]
    
    # Rotate vertices
    rot_x = [x + dx*np.cos(psi) - dy*np.sin(psi) for dx, dy in zip(hull_x, hull_y)]
    rot_y = [y + dx*np.sin(psi) + dy*np.cos(psi) for dx, dy in zip(hull_x, hull_y)]
    
    boat_shape.set_data(rot_x, rot_y)
    
    # 2. Draw Vectors (Anchored to boat center)
    # Quiver arguments: X, Y, U, V
    vector_current.set_offsets([x, y])
    vector_current.set_UVC(v_current[0], v_current[1])
    
    vector_thrust.set_offsets([x, y])
    vector_thrust.set_UVC(v_boat[0], v_boat[1])
    
    vector_result.set_offsets([x, y])
    vector_result.set_UVC(v_ground[0], v_ground[1])
    
    # 3. Draw Trail
    path_line.set_data(sim.path_x, sim.path_y)
    
    return boat_shape, path_line, vector_current, vector_thrust, vector_result

anim = FuncAnimation(fig, animate, init_func=init, frames=FRAMES, interval=50, blit=False)
plt.show()