import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from pymoveit2 import GripperInterface, MoveIt2
from pymoveit2.robots import tiago as robot

from custom_actions.action import PickPlace, DriveDistance
from clean_table.blocking import send_goal_and_wait

# Robot-specific calibrated constants, unchanged from clean_the_table.py
GRASP_ORIENTATION = [0.4848080665203961, 0.519817350525095, -0.5030804664430538, -0.4915903833612542]
STOW_POSITION = [0.27, 0.13585252380568053, 0.5383405000001573]
DROP_OFFSETS = [[-0.05, 0.0], [0.0, -0.1], [-0.1, -0.1], [-0.15, -0.15], [0.0, -0.15], [0.0, 0.0]]


class PickPlaceServer(Node):
    """
    Action server wrapping every MoveIt2/gripper interaction that used to
    live inline in clean_the_table.py's Grasp_object, Move_backward,
    Drop_object and Move_back_from_drop_table states.

    A single "pick" goal now performs: add collision boxes, open
    gripper, move arm to the object, drive forward, close gripper,
    retreat, and stow the arm. A single "place" goal performs the
    equivalent sequence at the drop table. The orchestrator no longer
    tracks move_arm_status / gripper_status / collision_object_added as
    loose booleans - that bookkeeping is internal to this server now.
    """

    def __init__(self):
        super().__init__('pick_place_server')

        self.target_frame = 'base_link'
        self._callback_group = ReentrantCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._moveit2 = MoveIt2(
            node=self,
            joint_names=robot.joint_names(),
            base_link_name=robot.base_link_name(),
            end_effector_name=robot.end_effector_name(),
            group_name=robot.MOVE_GROUP_ARM,
            callback_group=self._callback_group,
        )
        self._gripper = GripperInterface(
            node=self,
            gripper_joint_names=robot.gripper_joint_names(),
            open_gripper_joint_positions=robot.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=robot.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=robot.MOVE_GROUP_GRIPPER,
            callback_group=self._callback_group,
            gripper_command_action_name='gripper_action_controller/gripper_cmd',
        )
        self._moveit2.planner_id = 'RRTConnectkConfigDefault'
        self._moveit2.max_velocity = 0.5
        self._moveit2.max_acceleration = 0.5
        self._moveit2.cartesian_avoid_collisions = False
        self._moveit2.cartesian_jump_threshold = 0.0
        self._synchronous = True
        self._cartesian = False
        self._cartesian_max_step = 0.0025
        self._cartesian_fraction_threshold = 0.0

        self._drive_client = ActionClient(
            self, DriveDistance, 'drive_distance', callback_group=self._callback_group)

        self._action_server = ActionServer(
            self,
            PickPlace,
            'pick_place',
            execute_callback=self._execute_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info('pick_place_server ready')

    # ---- shared helpers, ported from clean_the_table.py ----

    def _lookup_transform(self, marker_id):
        source_frame = f'tag36h11_{marker_id}'
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {source_frame} to {self.target_frame}: {ex}')
            return None

    def _add_collision_box(self, object_id, position, orientation, dimensions):
        self._moveit2.add_collision_box(
            id=object_id, position=position, quat_xyzw=orientation, size=dimensions)

    def _remove_collision_object(self, object_id):
        self._moveit2.remove_collision_object(id=object_id)

    def _move_arm_to_pose(self, position, orientation):
        self._moveit2.move_to_pose(
            position=position,
            quat_xyzw=orientation,
            cartesian=self._cartesian,
            cartesian_max_step=self._cartesian_max_step,
            cartesian_fraction_threshold=self._cartesian_fraction_threshold,
        )
        if self._synchronous:
            self._moveit2.wait_until_executed()

    def _open_gripper(self):
        self._gripper.open()
        self._gripper.wait_until_executed()

    def _close_gripper(self):
        self._gripper.close()
        self._gripper.wait_until_executed()

    def _drive(self, distance, direction):
        """Blocking call out to the drive_distance_server action."""
        self._drive_client.wait_for_server()
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.direction = direction
        accepted, result = send_goal_and_wait(self._drive_client, goal, timeout=30.0)
        if not accepted or result is None:
            return False
        return result.result.success

    # ---- pick / place sequences ----

    def _do_pick(self, goal_handle, marker_id, non_target_markers):
        feedback = PickPlace.Feedback()

        feedback.stage = 'adding_collision_objects'
        goal_handle.publish_feedback(feedback)
        for marker in non_target_markers:
            trans = self._lookup_transform(marker)
            if trans is not None:
                self._add_collision_box(
                    f'Box_{marker}',
                    [trans.transform.translation.x + 0.07,
                     trans.transform.translation.y,
                     trans.transform.translation.z + 0.2],
                    [0.0, 0.0, 0.0, 1.0],
                    dimensions=[0.06, 0.06, 0.15])

        object_transform = self._lookup_transform(marker_id)
        if object_transform is None:
            return False, f'Could not locate marker {marker_id}'

        position = [
            object_transform.transform.translation.x - 0.1,
            object_transform.transform.translation.y,
            object_transform.transform.translation.z + 0.15,
        ]
        self._add_collision_box(
            'table',
            [position[0] + 0.1, position[1], position[2] - 0.06],
            [0.0, 0.0, 0.0, 1.0],
            dimensions=[0.5, 0.5, 0.025])

        feedback.stage = 'opening_gripper'
        goal_handle.publish_feedback(feedback)
        self._open_gripper()

        feedback.stage = 'moving_arm'
        goal_handle.publish_feedback(feedback)
        self._move_arm_to_pose(
            [position[0] - 0.1, position[1], position[2]], GRASP_ORIENTATION)

        self._remove_collision_object('table')
        for marker in non_target_markers:
            self._remove_collision_object(f'Box_{marker}')

        feedback.stage = 'approaching_object'
        goal_handle.publish_feedback(feedback)
        if not self._drive(0.08, 1):
            return False, 'Failed to approach object'

        feedback.stage = 'grasping'
        goal_handle.publish_feedback(feedback)
        self._close_gripper()

        feedback.stage = 'retreating'
        goal_handle.publish_feedback(feedback)
        if not self._drive(0.33, -1):
            return False, 'Failed to retreat after grasp'

        feedback.stage = 'stowing_arm'
        goal_handle.publish_feedback(feedback)
        self._move_arm_to_pose(STOW_POSITION, GRASP_ORIENTATION)

        return True, 'Pick complete'

    def _do_place(self, goal_handle, marker_id, drop_index):
        feedback = PickPlace.Feedback()

        table_transform = self._lookup_transform(marker_id)
        if table_transform is None:
            return False, f'Could not locate drop table marker {marker_id}'

        feedback.stage = 'adding_collision_objects'
        goal_handle.publish_feedback(feedback)
        self._add_collision_box(
            'table_pick',
            [table_transform.transform.translation.x + 0.15,
             table_transform.transform.translation.y,
             table_transform.transform.translation.z - 0.15],
            [0.0, 0.0, 0.0, 1.0],
            dimensions=[0.5, 0.5, 0.5])

        x_offset, y_offset = DROP_OFFSETS[drop_index - 1]

        feedback.stage = 'moving_arm'
        goal_handle.publish_feedback(feedback)
        self._move_arm_to_pose(
            [table_transform.transform.translation.x + x_offset,
             table_transform.transform.translation.y + y_offset,
             table_transform.transform.translation.z + 0.2],
            GRASP_ORIENTATION)

        feedback.stage = 'releasing'
        goal_handle.publish_feedback(feedback)
        self._open_gripper()
        self._remove_collision_object('table_pick')

        feedback.stage = 'retreating'
        goal_handle.publish_feedback(feedback)
        if not self._drive(0.33, -1):
            return False, 'Failed to retreat after placing'

        feedback.stage = 'stowing_arm'
        goal_handle.publish_feedback(feedback)
        self._move_arm_to_pose(STOW_POSITION, GRASP_ORIENTATION)
        self._close_gripper()

        return True, 'Place complete'

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        request = goal_handle.request
        result = PickPlace.Result()

        if request.operation == 'pick':
            success, message = self._do_pick(
                goal_handle, request.marker_id, list(request.non_target_markers))
        elif request.operation == 'place':
            success, message = self._do_place(
                goal_handle, request.marker_id, request.drop_index)
        else:
            success, message = False, f'Unknown operation "{request.operation}"'

        result.success = success
        result.message = message
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceServer()
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
