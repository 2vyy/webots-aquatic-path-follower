#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
import math
import time

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define your list of waypoints here: (x, y, yaw_in_radians)
        # Yaw: 0 = East (+X), PI/2 = North (+Y), PI = West (-X)
        self.waypoints = [
            (8.0, 0.0, 1.57),
        ]
        self.current_waypoint_index = 0

    def wait_for_nav2(self):
        """Wait until Nav2's bt_navigator is fully active before sending goals."""
        self.get_logger().info('Waiting for Nav2 to fully activate...')
        
        # Wait for the action server to be available
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('  navigate_to_pose action server not available, waiting...')
        
        # Wait for bt_navigator lifecycle to reach ACTIVE state
        client = self.create_client(GetState, '/bt_navigator/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('  bt_navigator lifecycle service not available, waiting...')
        
        while True:
            req = GetState.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None:
                state = future.result().current_state.label
                if state == 'active':
                    self.get_logger().info('Nav2 is ACTIVE and ready!')
                    break
                else:
                    self.get_logger().info(f'  bt_navigator state: {state}, waiting...')
            time.sleep(1.0)

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            rclpy.shutdown()
            return

        wp = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Sending Waypoint {self.current_waypoint_index + 1}: x={wp[0]}, y={wp[1]}, yaw={wp[2]}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        # Use time(0) to request the latest available TF, avoiding
        # "extrapolation into the past" errors when SLAM starts late.
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0
        
        goal_msg.pose.pose.position.x = wp[0]
        goal_msg.pose.pose.position.y = wp[1]
        
        # Convert yaw to quaternion
        half_yaw = wp[2] * 0.5
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Print periodic navigation feedback."""
        pose = feedback_msg.feedback.current_pose.pose
        self.get_logger().info(
            f'  Position: x={pose.position.x:.2f}, y={pose.position.y:.2f}',
            throttle_duration_sec=3.0)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected! Nav2 may not be fully ready.')
            return

        self.get_logger().info('Goal accepted, navigating...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached successfully!')
            self.current_waypoint_index += 1
            self.send_next_waypoint()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Waypoint {self.current_waypoint_index + 1} was canceled.')
            rclpy.shutdown()
        else:
            self.get_logger().error(f'Waypoint {self.current_waypoint_index + 1} failed (status={status})')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    node.wait_for_nav2()
    node.send_next_waypoint()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
