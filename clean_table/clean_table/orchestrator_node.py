import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from apriltag_msgs.msg import AprilTagDetectionArray
from std_msgs.msg import Int32MultiArray
from teleop_tools_msgs.action import Increment
from nav2_msgs.action import NavigateToPose

from custom_actions.action import PickPlace, DriveDistance, AlignToMarker
from custom_actions.srv import GetMarkerTransform
from clean_table.blocking import send_goal_and_wait, call_service_and_wait 

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType



# Robot/site-specific waypoints and marker ids, unchanged from clean_the_table.py
PICK_TABLE_MARKER = 11
DROP_TABLE_MARKER = 10

INITIAL_POSE_POSITION = [-6.76, 2.06, 0.0]
INITIAL_POSE_ORIENTATION = [0.0, 0.0, 0.04214326068243119, 0.9991115781428281]

WAYPOINT_APPROACH_PICK_TABLE = ([0.17162578105926514, 6.043226737976074, 0.0],
                                 [0.0, 0.0, 0.0273277800975909, 0.9996265264762324])
WAYPOINT_TO_DROP_TABLE_1 = ([0.0387272834777832, 4.4026079177856445, 0.0],
                             [0.0, 0.0, 0.0, 1.0])
WAYPOINT_TO_DROP_TABLE_2 = ([1.5217665433883667, 4.392, 0.0],
                             [0.0, 0.0, 0.034301134166758326, 0.9994115429565911])
WAYPOINT_BACK_TO_PICK_TABLE_1 = WAYPOINT_TO_DROP_TABLE_1
WAYPOINT_BACK_TO_PICK_TABLE_2 = ([0.17162578105926514, 5.973226737976074, 0.0],
                                  [0.0, 0.0, 0.0273277800975909, 0.9996265264762324])


class Orchestrator(Node):
    """
    The state machine that used to be laser_scan_callback in
    clean_the_table.py. It no longer owns TF, MoveIt2, the gripper, or
    raw /cmd_vel driving logic directly - it only tracks task state and
    delegates each step to an action/service server:

      - NavigateToPose (nav2)      : all point-to-point driving
      - drive_distance             : the backward nudge before rotating to detect
      - align_to_marker            : rotating to face the pick/drop table
      - get_marker_transform       : "where is marker X right now"
      - pick_place                 : the whole grasp/retreat or place/retreat sequence
      - /head_controller/increment : head search sweep (unchanged from the original)

    NOTE on concurrency: each state below blocks (via
    spin_until_future_complete) until its action/service call finishes,
    which is a big simplification over the original's non-blocking,
    poll-every-tick style. This works because the node uses a
    ReentrantCallbackGroup + MultiThreadedExecutor (same as the original
    already did), but it does tie up one executor thread per in-flight
    state. For a fully non-blocking version, replace these blocking
    calls with goal/result callbacks that set state on completion.
    """

    def __init__(self):
        super().__init__('orchestrator')

        self._callback_group = ReentrantCallbackGroup()

        self.state = 'initial'
        self.object_count = 0
        self.object_marker_list = []
        self.picked_markers = []
        self.visible_marker_ids = []
        self.moving = None

        self._pose_settle_start = None

        self.action_in_progress = False

        self.param_client = self.create_client(
            SetParameters, '/controller_server/set_parameters', callback_group=self._callback_group)
        
        self._default_xy_tolerance = None
        self._default_yaw_tolerance = None

        # publishers / subscriptions
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(
            LaserScan, '/scan_raw', self._state_machine_callback, 10,
            callback_group=self._callback_group)
        self.create_subscription(
            Twist, '/cmd_vel', self._vel_callback, 10,
            callback_group=self._callback_group)
        # self.create_subscription(
        #     Int32MultiArray, '/visible_markers', self._visible_markers_callback, 10,
        #     callback_group=self._callback_group)
        self.subscription=self.create_subscription(AprilTagDetectionArray,'/detections',self._visible_markers_callback,10)

        # action / service clients
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self._callback_group)
        self.drive_client = ActionClient(
            self, DriveDistance, 'drive_distance', callback_group=self._callback_group)
        self.align_client = ActionClient(
            self, AlignToMarker, 'align_to_marker', callback_group=self._callback_group)
        self.pick_place_client = ActionClient(
            self, PickPlace, 'pick_place', callback_group=self._callback_group)
        self.head_client = ActionClient(
            self, Increment, '/head_controller/increment', callback_group=self._callback_group)
        self.marker_client = self.create_client(
            GetMarkerTransform, 'get_marker_transform', callback_group=self._callback_group)
        
        # self._cache_default_tolerances()

        self.get_logger().info('orchestrator ready')

    # ---- subscriptions ----

    def _vel_callback(self, msg: Twist):
        self.moving = not (msg.linear.x == 0.0 and msg.angular.z == 0.0)

    def _visible_markers_callback(self, msg):

        HEAD_SEARCH_STATES = {'Detection', 'table_allignment', 'Allignment', 'Allign_to_drop_table'}
        self.visible_marker_ids = [detection.id for detection in msg.detections]
        
        if self.state == 'Allignment' and len(self.object_marker_list) == 0:
            self.object_marker_list = [
                int(m) for m in np.unique(self.visible_marker_ids) if m != PICK_TABLE_MARKER]

        # if not self.visible_marker_ids and self.state != 'move_back_to_adjust_position':
        #     self._send_head_increment(0.0, -0.1)

        if not self.visible_marker_ids and self.state in HEAD_SEARCH_STATES:
            self._send_head_increment(0.0, -0.1)

    # ---- blocking helpers to the action/service servers ----

    def _publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = INITIAL_POSE_POSITION
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = INITIAL_POSE_ORIENTATION
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        self.initial_pose_pub.publish(msg)

    def _send_head_increment(self, head_1, head_2):
        self.head_client.wait_for_server()
        goal = Increment.Goal()
        goal.increment_by = [head_1, head_2]
        self.head_client.send_goal_async(goal)  # fire-and-forget, same as original

    def _navigate_to(self, position, orientation):
        self.nav_client.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.position.z = position
        (goal.pose.pose.orientation.x, goal.pose.pose.orientation.y,
         goal.pose.pose.orientation.z, goal.pose.pose.orientation.w) = orientation

        accepted, result = send_goal_and_wait(self.nav_client, goal, timeout=60.0)
        if not accepted or result is None:
            return False
        return result.status == 4  # STATUS_SUCCEEDED

    def _drive(self, distance, direction):
        self.drive_client.wait_for_server()
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.direction = direction
        accepted, result = send_goal_and_wait(self.drive_client, goal, timeout=30.0)
        if not accepted or result is None:
            return False
        return result.result.success

    def _align_to(self, marker_id, angle_accuracy):
        # self.align_client.wait_for_server()
        # goal = AlignToMarker.Goal()
        # goal.marker_id = marker_id
        # goal.angle_accuracy = angle_accuracy
        # send_future = self.align_client.send_goal_async(goal)
        # rclpy.spin_until_future_complete(self, send_future)
        # goal_handle = send_future.result()
        # if not goal_handle.accepted:
        #     return False
        # result_future = goal_handle.get_result_async()
        # rclpy.spin_until_future_complete(self, result_future)
        # return result_future.result().result.success

        self.align_client.wait_for_server()
        goal = AlignToMarker.Goal()
        goal.marker_id = marker_id
        goal.angle_accuracy = angle_accuracy
        accepted, result = send_goal_and_wait(self.align_client, goal, timeout=30.0)
        if not accepted or result is None:
            return False
        return result.result.success

    def _pick_place(self, operation, marker_id, drop_index=0, non_target_markers=None):
        self.pick_place_client.wait_for_server()
        goal = PickPlace.Goal()
        goal.operation = operation
        goal.marker_id = marker_id
        goal.drop_index = drop_index
        goal.non_target_markers = non_target_markers or []
        accepted, result = send_goal_and_wait(self.pick_place_client, goal, timeout=60.0)
        if not accepted or result is None:
            return False
        return result.result.success

    def _nearest_marker_id(self, marker_id_list):
        if not marker_id_list:
            self.get_logger().info('The marker id list is empty, no markers passed')
            return None

        shortest_dist = 1000
        closest_marker = None
        for marker in marker_id_list:
            request = GetMarkerTransform.Request()
            request.marker_id = marker
            response = call_service_and_wait(self.marker_client, request, timeout=2.0)
            if response is None or not response.found:
                self.get_logger().info('The response from the service client for the queried marker is empty')
                continue
            t = response.transform.transform.translation
            distance = np.sqrt(t.x ** 2 + t.y ** 2 + t.z ** 2)
            if distance < shortest_dist:
                shortest_dist = distance
                closest_marker = marker
        return closest_marker
    
    def _cache_default_tolerances(self):
        request = GetParameters.Request()
        request.names = [
            'general_goal_checker.xy_goal_tolerance',
            'general_goal_checker.yaw_goal_tolerance',
        ]
        response = call_service_and_wait(self.get_param_client, request, timeout=2.0)
        if response is None:
            self.get_logger().warn('Could not fetch default goal tolerances, keeping None')
            return
        self._default_xy_tolerance = response.values[0].double_value
        self._default_yaw_tolerance = response.values[1].double_value

    def _restore_default_tolerance(self):
        if self._default_xy_tolerance is None:
            return  # never successfully cached - leave whatever's currently set
        self._set_goal_tolerance(self._default_xy_tolerance, self._default_yaw_tolerance)
    

    def _set_goal_tolerance(self, xy_tolerance, yaw_tolerance):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(name='general_goal_checker.xy_goal_tolerance',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=xy_tolerance)),
            Parameter(name='general_goal_checker.yaw_goal_tolerance',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=yaw_tolerance)),
        ]
        call_service_and_wait(self.param_client, request, timeout=2.0)

    # ---- the state machine itself ----

    def _state_machine_callback(self, msg: LaserScan):

        # Prevent ReentrantCallbackGroup from exploding threads
        if getattr(self, 'action_in_progress', False):
            return

        self.action_in_progress = True
        try:

            index = len(msg.ranges) // 2
            straight_dist = msg.ranges[index]
            pick_table_detected = PICK_TABLE_MARKER in self.visible_marker_ids
            marker_detected = len(self.visible_marker_ids) > 0

            if self.state == 'initial':
                self._publish_initial_pose()
                self._pose_settle_start = self.get_clock().now()
                self.state = 'intermediate'

            elif self.state == 'intermediate':
                elapsed = (self.get_clock().now() - self._pose_settle_start).nanoseconds / 1e9
                if elapsed > 2.0:  
                    self.state = 'Navigation'

            elif self.state == 'Navigation' and straight_dist > 1.0:
                self._set_goal_tolerance(0.6, 0.3) 
                if self._navigate_to(WAYPOINT_APPROACH_PICK_TABLE[0],WAYPOINT_APPROACH_PICK_TABLE[1]):
                    self.state = 'Detection'
                self._restore_default_tolerance() 

            elif self.state == 'Detection':
                self.state = 'table_allignment'

            elif self.state == 'table_allignment' and pick_table_detected:
                self.get_logger().info(f' table allignment status = {self._align_to(PICK_TABLE_MARKER, 2.0)}')
                if self._align_to(PICK_TABLE_MARKER, 2.0):
                    self.state = 'Allignment'

            elif self.state == 'Allignment' and marker_detected:
                target_marker_id = self._nearest_marker_id(self.object_marker_list)
                self.get_logger().info(f' marker id list = {self.object_marker_list} ,target marker id = {target_marker_id} ')
                if target_marker_id is None:
                    return
                
                if self._align_to(target_marker_id, 1.0):
                    non_target_objects = [
                        m for m in self.object_marker_list if m not in self.picked_markers]
                    if self._pick_place('pick', target_marker_id,
                                        non_target_markers=non_target_objects):
                        self.picked_markers.append(target_marker_id)
                        if target_marker_id in self.object_marker_list:
                            self.object_marker_list.remove(target_marker_id)
                        self.state = 'Move_to_table'

            elif self.state == 'Move_to_table':
                self._set_goal_tolerance(0.5, 45.0)   # loose - just get roughly there
                reached = self._navigate_to(WAYPOINT_TO_DROP_TABLE_1[0],WAYPOINT_TO_DROP_TABLE_1[1])
                self._restore_default_tolerance() 
                # self._set_goal_tolerance(0.05, 0.1)  # restore tight tolerance for later states
                # if self._navigate_to(WAYPOINT_TO_DROP_TABLE_1[0],WAYPOINT_TO_DROP_TABLE_1[1]):
                if reached:
                    self.state = 'Move_to_table_position_2'

            elif self.state == 'Move_to_table_position_2':
                self._set_goal_tolerance(0.5, 0.2) 
                if self._navigate_to(WAYPOINT_TO_DROP_TABLE_2[0],WAYPOINT_TO_DROP_TABLE_2[1]):
                    self.state = 'Allign_to_drop_table'
                self._restore_default_tolerance() 

            elif self.state == 'Allign_to_drop_table' and marker_detected:
                if self._align_to(DROP_TABLE_MARKER, 1.0) and self.moving is False:
                    self.state = 'Placing'

            elif self.state == 'Placing':
                self.object_count += 1
                if self._pick_place('place', DROP_TABLE_MARKER, drop_index=self.object_count):
                    self.state = 'Head_back_to_pick_table_intermediate'

            elif self.state == 'Head_back_to_pick_table_intermediate':
                self._set_goal_tolerance(0.5, 45.0)   # loose - just get roughly there
                reached = self._navigate_to(WAYPOINT_BACK_TO_PICK_TABLE_1[0],WAYPOINT_BACK_TO_PICK_TABLE_1[1])
                self._restore_default_tolerance()   # restore tight tolerance for later states
                if reached and len(self.object_marker_list) != 0:
                    self.state = 'Head_back_to_pick_table_final'
                # if self._navigate_to(WAYPOINT_BACK_TO_PICK_TABLE_1[0],WAYPOINT_BACK_TO_PICK_TABLE_1[1]) and len(self.object_marker_list) != 0:
                #     self.state = 'Head_back_to_pick_table_final'

            elif self.state == 'Head_back_to_pick_table_final':
                self._set_goal_tolerance(0.8, 0.2)
                reached=self._navigate_to(WAYPOINT_BACK_TO_PICK_TABLE_2[0],WAYPOINT_BACK_TO_PICK_TABLE_2[1])
                self._restore_default_tolerance() 
                if reached and self.moving is False:
                # if self._navigate_to(WAYPOINT_APPROACH_PICK_TABLE[0],WAYPOINT_APPROACH_PICK_TABLE[1]) and self.moving is False:
                    self.state = 'move_back_to_adjust_position'

            elif self.state == 'move_back_to_adjust_position':
                self._drive(0.05, -1)
                if pick_table_detected:
                    self.state = 'Detection'

            self.get_logger().info(
                f'Current stage {self.state}, remaining object list = {self.object_marker_list}, , visible_markers = {self.visible_marker_ids} , pick table detected = {pick_table_detected} '
                f'object count = {self.object_count}')
        finally:
                    self.action_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
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
