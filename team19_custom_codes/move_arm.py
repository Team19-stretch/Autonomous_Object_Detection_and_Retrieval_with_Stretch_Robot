#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.action import ActionClient
import math
import time

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, TransformStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf2_ros import Buffer, TransformListener, TransformException
import tf_transformations

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class StretchArucoGrasp(Node):
    def __init__(self):
        super().__init__('stretch_aruco_grasp')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.trajectory_client = ActionClient(self, FollowJointTrajectory, '/stretch_controller/follow_joint_trajectory')
        self.get_logger().info("Waiting for joint trajectory action server...")
        if not self.trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Trajectory action server not available.")
            rclpy.shutdown()
            return
        self.get_logger().info("Joint trajectory action server connected.")

        self.subscription = self.create_subscription(MarkerArray, '/aruco/marker_array', self.marker_callback, 10)

        self.latest_marker = None
        self.goal_stage = 0
        self.timer = self.create_timer(1.0, self.try_execute_grasp)

    def marker_callback(self, msg: MarkerArray):
        if not msg.markers:
            return
        self.latest_marker = msg.markers[0]
        self.get_logger().info(f"Marker {self.latest_marker.id} detected.")

    def try_execute_grasp(self):
        if self.latest_marker is None or self.goal_stage != 0:
            return

        from_frame = self.latest_marker.header.frame_id
        to_frame = 'base_link'

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(to_frame, from_frame, Time())
        except TransformException as ex:
            self.get_logger().info(f'Transform unavailable: {ex}')
            return

        self.get_logger().info(f"Transform from {from_frame} to {to_frame} available.")

        marker = self.latest_marker
        p = marker.pose.position
        q = transform.transform.translation
        r = transform.transform.rotation

        trans = [q.x, q.y, q.z]
        rot = [r.x, r.y, r.z, r.w]
        tf_mat = tf_transformations.quaternion_matrix(rot)
        tf_mat[0:3, 3] = trans
        marker_vec = [p.x, p.y, p.z, 1.0]
        base_vec = tf_mat @ marker_vec

        pos_x = base_vec[0]
        pos_y = base_vec[1]
        pos_z = base_vec[2]

        self.get_logger().info(f"Marker in base_link: x={pos_x:.3f}, y={pos_y:.3f}, z={pos_z:.3f}")

        self.lift = clamp(pos_z, 0.15, 1.10)

        total_ext = clamp(pos_x, 0.0, 0.5)
        self.l0 = total_ext * 0.4
        self.l1 = total_ext * 0.3
        self.l2 = total_ext * 0.2
        self.l3 = total_ext * 0.1

        self.yaw = clamp(math.atan2(pos_y, pos_x), -1.75, 4.0)

        if pos_z < 0.5:
            self.pitch = -1.2
        elif pos_z < 0.8:
            self.pitch = -0.9
        else:
            self.pitch = -0.6

        self.roll = 0.0

        self.goal_stage = 1
        self.send_trajectory(['joint_gripper_finger_left'], [0.165], 2.0)

    def send_trajectory(self, joint_names, positions, time_sec):
        self.get_logger().info(f"Sending trajectory: {joint_names} -> {positions}")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=time_sec).to_msg()
        goal.trajectory.points = [point]
        goal.trajectory.header.stamp = self.get_clock().now().to_msg()

        send_future = self.trajectory_client.send_goal_async(goal)
        send_future.add_done_callback(lambda f: self.handle_goal_response(f, joint_names, positions, time_sec))

    def handle_goal_response(self, future, joint_names, positions, time_sec):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Trajectory goal rejected: {joint_names}")
            return

        self.get_logger().info(f"Trajectory goal accepted: {joint_names}")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda r: self.handle_result(r, joint_names))

    def handle_result(self, future, joint_names):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info(f"Trajectory for {joint_names} executed successfully.")
        else:
            self.get_logger().warn(f"Execution failed for {joint_names} with error code {result.error_code}")

        if self.goal_stage == 1:
            self.goal_stage = 2
            time.sleep(0.5)
            self.send_trajectory(
                ['joint_lift', 'joint_arm_l0', 'joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3',
                 'joint_wrist_yaw', 'joint_wrist_pitch', 'joint_wrist_roll'],
                [self.lift, self.l0, self.l1, self.l2, self.l3, self.yaw, self.pitch, self.roll],
                5.0
            )
        elif self.goal_stage == 2:
            self.goal_stage = 3
            time.sleep(0.5)
            self.send_trajectory(['joint_gripper_finger_left'], [-0.3], 2.0)
        elif self.goal_stage == 3:
            self.get_logger().info("Grasp sequence complete ✅")
            self.goal_stage = 4


def main(args=None):
    rclpy.init(args=args)
    node = StretchArucoGrasp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
