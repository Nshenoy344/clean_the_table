import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Int32MultiArray
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from custom_actions.srv import GetMarkerTransform


class MarkerLookupService(Node):
    """
    Owns AprilTag detection processing and the TF buffer/listener that
    used to live directly on clean_the_table's CleanTable node.

    Publishes the currently visible marker ids on /visible_markers so
    the orchestrator can decide what to do (e.g. trigger a head search
    when nothing is visible), and offers the get_marker_transform
    service so any other node can ask for a specific marker's transform
    on demand instead of doing its own TF lookup.
    """

    def __init__(self):
        super().__init__('marker_lookup_service')

        self.target_frame = 'base_link'
        self._callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.visible_marker_ids = []
        self.visible_pub = self.create_publisher(Int32MultiArray, '/visible_markers', 10)

        self.create_subscription(
            AprilTagDetectionArray, '/detections', self._detections_callback, 10,
            callback_group=self._callback_group)

        self.create_service(
            GetMarkerTransform, 'get_marker_transform', self._get_marker_transform,
            callback_group=self._callback_group)

        self.get_logger().info('marker_lookup_service ready')

    def _detections_callback(self, msg: AprilTagDetectionArray):
        self.visible_marker_ids = [detection.id for detection in msg.detections]
        out = Int32MultiArray()
        out.data = self.visible_marker_ids
        self.visible_pub.publish(out)

    def _get_marker_transform(self, request, response):
        source_frame = f'tag36h11_{request.marker_id}'
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time())
            response.found = True
            response.transform = transform
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {source_frame} to {self.target_frame}: {ex}')
            response.found = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MarkerLookupService()
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
