#!/usr/bin/env python3
"""Sender en sekvens av waypoints via Nav2 NavigateToPose."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time

class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Waypoints: (x, y, yaw_degrees)
        # Arena 3.6 x 2.4 m, origo i midten
        self.waypoints = [
            ( 0.8,  0.5,   0.0),
            ( 0.8, -0.5, -90.0),
            (-0.8, -0.5, 180.0),
            (-0.8,  0.5,  90.0),
            ( 0.0,  0.0,   0.0),
        ]
        self.current = 0

    def yaw_to_quat(self, yaw_deg):
        y = math.radians(yaw_deg)
        return math.sin(y/2), math.cos(y/2)

    def send_next(self):
        if self.current >= len(self.waypoints):
            self.get_logger().info('Oppdrag fullfort — alle waypoints nadd!')
            rclpy.shutdown()
            return

        x, y, yaw = self.waypoints[self.current]
        qz, qw = self.yaw_to_quat(yaw)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Waypoint {self.current+1}/{len(self.waypoints)}: ({x}, {y}) yaw={yaw}'
        )
        self._client.wait_for_server()
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self.goal_cb)

    def goal_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Mal avvist, prover neste')
            self.current += 1
            self.send_next()
            return
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'Waypoint {self.current+1} nadd!')
        else:
            self.get_logger().warn(f'Waypoint {self.current+1} feilet status={status}')
        self.current += 1
        time.sleep(1.0)
        self.send_next()

def main():
    rclpy.init()
    node = WaypointMission()
    node.get_logger().info('Starter waypoint-oppdrag om 3 sekunder...')
    time.sleep(3.0)
    node.send_next()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
