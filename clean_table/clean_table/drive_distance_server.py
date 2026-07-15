import time

import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from custom_actions.action import DriveDistance


class DriveDistanceServer(Node):
    """
    Action server that drives the robot forward or backward by a target
    distance, measured from the change in the middle LaserScan beam.

    This is the original drive_robot_forward / drive_robot_backward logic
    from clean_the_table.py, unified into a single direction-aware routine
    and exposed as an action instead of two near-duplicate helper methods.
    """

    def __init__(self):
        super().__init__('drive_distance_server')

        self._callback_group = ReentrantCallbackGroup()

        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.latest_scan = None
        self.create_subscription(
            LaserScan, '/scan_raw', self._scan_callback, 10,
            callback_group=self._callback_group)

        self._action_server = ActionServer(
            self,
            DriveDistance,
            'drive_distance',
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info('drive_distance_server ready')

    def _scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def _middle_range(self):
        index = len(self.latest_scan.ranges) // 2
        return self.latest_scan.ranges[index]

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        distance = goal_handle.request.distance
        direction = goal_handle.request.direction  # 1 forward, -1 backward
        speed = 0.1 * direction

        twist = Twist()
        feedback = DriveDistance.Feedback()
        result = DriveDistance.Result()

        # wait for the first scan so we have a baseline reading
        while self.latest_scan is None and rclpy.ok():
            time.sleep(0.05)

        initial_dist = self._middle_range()
        travelled = 0.0

        while rclpy.ok():
            straight_dist = self._middle_range()

            # same formula for both directions: for direction=+1 (forward)
            # this is (initial - current); for direction=-1 (backward) it
            # matches (current - initial) from the original code.
            travelled = direction * (initial_dist - straight_dist)

            if distance + 0.02 > travelled > distance - 0.02:
                twist.linear.x = 0.0
                self.drive_publisher.publish(twist)
                result.success = True
                result.distance_travelled = travelled
                goal_handle.succeed()
                return result

            twist.linear.x = speed
            self.drive_publisher.publish(twist)

            feedback.distance_travelled = travelled
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        result.success = False
        result.distance_travelled = travelled
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DriveDistanceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
