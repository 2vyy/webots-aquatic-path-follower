#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math

class WaypointSender(Node):
    def __init__(self):
        super().__init__('waypoint_sender')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True)])
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Define your list of waypoints here: (x, y, yaw_in_radians)
        # Note: 0 radians is East, PI/2 is North
        self.waypoints = [
            (-5.0, 0.0, 0.0),           # Wmove straight forward
        ]
        self.current_waypoint_index = 0

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed!')
            rclpy.shutdown()
            return

        wp = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Sending Waypoint {self.current_waypoint_index + 1}: x={wp[0]}, y={wp[1]}, yaw={wp[2]}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = wp[0]
        goal_msg.pose.pose.position.y = wp[1]
        
        # Convert yaw to quaternion
        half_yaw = wp[2] * 0.5
        goal_msg.pose.pose.orientation.z = math.sin(half_yaw)
        goal_msg.pose.pose.orientation.w = math.cos(half_yaw)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Waypoint {self.current_waypoint_index + 1} reached!')
        self.current_waypoint_index += 1
        self.send_next_waypoint()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointSender()
    node.send_next_waypoint()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
