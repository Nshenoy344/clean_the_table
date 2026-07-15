import time

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from tf_transformations import euler_from_quaternion

from custom_actions.action import AlignToMarker
from custom_actions.srv import GetMarkerTransform
from clean_table.blocking import call_service_and_wait 


class AlignToMarkerServer(Node):
    """
    Action server that rotates the robot in place until it is aligned
    with a given AprilTag marker, within angle_accuracy degrees.

    This is the original allign_to_marker logic from clean_the_table.py,
    exposed as an action. Marker transforms are no longer looked up
    locally; instead this node calls the marker_lookup_service's
    GetMarkerTransform service, keeping TF/perception logic out of the
    actuation nodes.
    """

    def __init__(self):
        super().__init__('align_to_marker_server')

        self._callback_group = ReentrantCallbackGroup()

        self.drive_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self._marker_client = self.create_client(
            GetMarkerTransform, 'get_marker_transform',
            callback_group=self._callback_group)

        self._action_server = ActionServer(
            self,
            AlignToMarker,
            'align_to_marker',
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info('align_to_marker_server ready')

    def _lookup_marker(self, marker_id):
        if not self._marker_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('get_marker_transform service not available')
            return None
        request = GetMarkerTransform.Request()
        request.marker_id = marker_id
        response = call_service_and_wait(self._marker_client, request, timeout=10.0)
        self.get_logger().info(f'the response after looking for marker is {response}')
        if response is None or not response.found:
            return None
        return response.transform

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        marker_id = goal_handle.request.marker_id
        angle_accuracy = goal_handle.request.angle_accuracy

        twist = Twist()
        feedback = AlignToMarker.Feedback()
        result = AlignToMarker.Result()

        while rclpy.ok():
            transform = self._lookup_marker(marker_id)
            if transform is None:
                self.get_logger().info(
                    f'Could not get transform for marker {marker_id}, retrying')
                time.sleep(0.1)
                continue

            q = transform.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            roll, pitch, yaw = np.degrees([roll, pitch, yaw])

            if yaw > 0:
                yaw = 180 - yaw
            elif yaw < 0:
                yaw = np.abs(yaw) - 180

            feedback.yaw_error = float(yaw)
            goal_handle.publish_feedback(feedback)

            if 0.0 + angle_accuracy > yaw > 0.0 - angle_accuracy:
                twist.angular.z = 0.0
                twist.linear.x = 0.0
                self.drive_publisher.publish(twist)
                result.success = True
                goal_handle.succeed()
                return result

            elif yaw > 0 + angle_accuracy:
                twist.angular.z = -0.2
            elif yaw < 0 - angle_accuracy:
                twist.angular.z = 0.2
            twist.linear.x = 0.0
            self.drive_publisher.publish(twist)

            time.sleep(0.05)

        result.success = False
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = AlignToMarkerServer()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
