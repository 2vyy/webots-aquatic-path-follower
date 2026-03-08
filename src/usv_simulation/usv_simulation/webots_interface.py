import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster
import math
import struct

class WebotsInterface:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # Create our own ROS 2 node
        rclpy.init(args=None)
        self.__node = rclpy.create_node('usv_bridge')

        # --- Clock ---
        self.clock_pub = self.__node.create_publisher(Clock, '/clock', 10)

        # --- Robot Constants ---
        self.wheel_base = 0.5

        # --- Device Retrieval ---
        self.left_motor = self.__robot.getDevice('left_prop_motor')
        self.right_motor = self.__robot.getDevice('right_prop_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.lidar = self.__robot.getDevice('lidar')
        self.gps = self.__robot.getDevice('gps')
        self.compass = self.__robot.getDevice('compass')
        self.imu = self.__robot.getDevice('imu')

        # --- Enable Sensors ---
        self.timestep = int(self.__robot.getBasicTimeStep())
        self.lidar.enable(self.timestep)
        self.gps.enable(self.timestep)
        self.compass.enable(self.timestep)
        self.imu.enable(self.timestep)

        # --- ROS 2 Publishers ---
        self.pub_scan = self.__node.create_publisher(LaserScan, '/scan', 10)
        self.pub_odom = self.__node.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self.__node)

        # --- ROS 2 Subscriber ---
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel_callback, 10)

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

        self.__node.get_logger().info('USV Bridge initialised – coordinate frame fixes applied')

        # IMU calibration baseline (set on first valid reading)
        self._imu_baseline = None

    def __cmd_vel_callback(self, msg):
        self.cmd_linear_x = msg.linear.x
        self.cmd_angular_z = msg.angular.z

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        # 1. PUBLISH CLOCK
        # The bridge is the clock SOURCE — it must NOT use use_sim_time.
        # Stamp all messages with the same time published on /clock.
        now = self.__robot.getTime()
        sec = int(now)
        nanosec = int((now - sec) * 1.0e9)

        ros_time = Time()
        ros_time.sec = sec
        ros_time.nanosec = nanosec

        clock_msg = Clock()
        clock_msg.clock = ros_time
        self.clock_pub.publish(clock_msg)

        # 2. READ SENSORS
        gps_vals = self.gps.getValues()
        compass_vals = self.compass.getValues()
        imu_rpy = self.imu.getRollPitchYaw()

        if math.isnan(gps_vals[0]) or math.isnan(compass_vals[0]):
            return

        # Since the robot natively follows the ROS X-forward convention (ENU),
        # the raw IMU Roll (X), Pitch (Y), and Yaw (Z) perfectly match the odom frame.
        cal_roll  = imu_rpy[0]
        cal_pitch = imu_rpy[1]
        yaw       = imu_rpy[2]

        pos_x = gps_vals[0]  # East
        pos_y = gps_vals[1]  # North

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        # 4. PUBLISH TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = ros_time
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.tf_broadcaster.sendTransform(t)



        # 6. PUBLISH ODOMETRY
        msg_odom = Odometry()
        msg_odom.header.stamp = ros_time
        msg_odom.header.frame_id = 'odom'
        msg_odom.child_frame_id = 'base_link'
        msg_odom.pose.pose.position.x = pos_x
        msg_odom.pose.pose.position.y = pos_y
        msg_odom.pose.pose.orientation.z = sy
        msg_odom.pose.pose.orientation.w = cy
        self.pub_odom.publish(msg_odom)

        # --- DEBUG LOGGING (periodic, every ~2 seconds) ---
        if not hasattr(self, '_dbg_counter'):
            self._dbg_counter = 0
        self._dbg_counter += 1
        if self._dbg_counter % 5 == 1:
            self.__node.get_logger().info(
                f'\n--- USV Bridge Debug ---\n'
                f'Time: {ros_time.sec}.{ros_time.nanosec}\n'
                f'GPS Raw: [{gps_vals[0]:.3f}, {gps_vals[1]:.3f}, {gps_vals[2]:.3f}]\n'
                f'Compass Raw: [{compass_vals[0]:.3f}, {compass_vals[1]:.3f}, {compass_vals[2]:.3f}]\n'
                f'IMU RPY (raw): roll={math.degrees(imu_rpy[0]):.2f} deg, pitch={math.degrees(imu_rpy[1]):.2f} deg, yaw={math.degrees(imu_rpy[2]):.2f} deg\n'
                f'IMU RPY (cal): roll={math.degrees(cal_roll):.2f} deg, pitch={math.degrees(cal_pitch):.2f} deg\n'
                f'Computed Pose -> X: {pos_x:.3f}, Y: {pos_y:.3f}, Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f} deg)\n'
                f'cmd_vel -> linear: {self.cmd_linear_x:.3f}, angular: {self.cmd_angular_z:.3f}\n'
                f'------------------------'
            )

        # 7. PUBLISH LIDAR
        if self.lidar.getSamplingPeriod() > 0:
            if not getattr(self, '_pcl_enabled', False):
                self.lidar.enablePointCloud()
                self._pcl_enabled = True

            # Suppress scan entirely if robot is too tilted (scan plane hits water)
            TILT_THRESHOLD = math.radians(10.0)
            if abs(cal_roll) > TILT_THRESHOLD or abs(cal_pitch) > TILT_THRESHOLD:
                pass  # Don't publish anything if too tilted
            else:
                    # --- 7a. Publish LaserScan natively on /scan ---
                    # Webots returns the range image as a flat list. With numberOfLayers=1,
                    # it is exactly one horizontal sweep.
                    range_image = self.lidar.getRangeImage()
                    if range_image:
                        msg_ls = LaserScan()
                        msg_ls.header.stamp = ros_time
                        msg_ls.header.frame_id = 'laser_link'
                        msg_ls.angle_min = -self.lidar.getFov() / 2.0
                        msg_ls.angle_max = self.lidar.getFov() / 2.0
                        msg_ls.angle_increment = self.lidar.getFov() / self.lidar.getHorizontalResolution()
                        msg_ls.time_increment = 0.0
                        msg_ls.scan_time = self.lidar.getSamplingPeriod() / 1000.0
                        msg_ls.range_min = self.lidar.getMinRange()
                        msg_ls.range_max = self.lidar.getMaxRange()
                        
                        # Apply ROS counter-clockwise rotation convention by reversing the array
                        msg_ls.ranges = range_image[::-1]
                        self.pub_scan.publish(msg_ls)

        # 8. WRITE MOTORS
        v = self.cmd_linear_x
        omega = self.cmd_angular_z
        # Scale Nav2's m/s commands to Webots motor rad/s.
        # thrustConstants=10 means each rad/s generates 10N of thrust.
        # A lower multiplier prevents overshooting while keeping strong turning torque.
        multiplier = 3.0
        # Water drag makes turning vastly harder than driving on land.
        # We massively multiply the angular command so the differential thrust can fight the water.
        angular_gain = 5.0
        left_vel  = (v - omega * self.wheel_base * angular_gain) * multiplier
        right_vel = (v + omega * self.wheel_base * angular_gain) * multiplier
        self.left_motor.setVelocity(left_vel)
        self.right_motor.setVelocity(right_vel)
