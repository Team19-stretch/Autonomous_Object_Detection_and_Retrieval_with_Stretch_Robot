#!/usr/bin/env python3
import math
import time
import rclpy
import hello_helpers.hello_misc as hm

def main():
    # Create the Stretch ROS2 node
    node = hm.HelloNode.quick_create('stretch_target_demo')
    node.get_logger().info("HelloNode initialized for Stretch&#8203;:contentReference[oaicite:4]{index=4}")

    # Wait for the first joint state message to ensure current positions are known
    node.get_logger().info("Waiting for initial joint state...")
    start_time = time.time()
    while (node.joint_state is None or len(node.joint_state.name) == 0) and time.time() - start_time < 5.0:
        time.sleep(0.1)
    if node.joint_state is None or len(node.joint_state.name) == 0:
        node.get_logger().error("No joint state received; check if the Stretch driver is running.")
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info("Joint state received. Current joints: %s" % list(node.joint_state.name))

    # Look up the transform from camera frame to base frame (geometry_msgs/TransformStamped)
    try:
        tf_cam_to_base = node.get_tf('base_link', 'camera_color_optical_frame')  # base_link <- camera transform&#8203;:contentReference[oaicite:5]{index=5}
    except Exception as e:
        node.get_logger().error(f"Transform lookup failed: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return
    if tf_cam_to_base is None:
        node.get_logger().error("Transform between 'camera_color_optical_frame' and 'base_link' not found.")
        node.destroy_node()
        rclpy.shutdown()
        return
    node.get_logger().info(f"Transform acquired: {tf_cam_to_base.transform.translation}")

    # Hardcoded target point in the camera's coordinate frame (x, y, z in meters)
    target_point_camera = (0.04265053836131551, 0.016452928661291753, 0.740810938550108)  # e.g. 0.5 m directly in front of camera
    node.get_logger().info(f"Target point in camera frame: {target_point_camera}")

    # Transform the target point to the base_link frame using the TF
    t = tf_cam_to_base.transform.translation
    q = tf_cam_to_base.transform.rotation
    # Convert quaternion (q.x,q.y,q.z,q.w) to rotation matrix
    # (Using formula for converting unit quaternion to 3x3 rotation matrix)
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    # Rotation matrix elements
    R00 = 1 - 2*(qy*qy + qz*qz)
    R01 = 2*(qx*qy - qz*qw)
    R02 = 2*(qx*qz + qy*qw)
    R10 = 2*(qx*qy + qz*qw)
    R11 = 1 - 2*(qx*qx + qz*qz)
    R12 = 2*(qy*qz - qx*qw)
    R20 = 2*(qx*qz - qy*qw)
    R21 = 2*(qy*qz + qx*qw)
    R22 = 1 - 2*(qx*qx + qy*qy)
    # Apply transform: base_coords = R * camera_coords + translation
    x_cam, y_cam, z_cam = target_point_camera
    x_base = R00*x_cam + R01*y_cam + R02*z_cam + t.x
    y_base = R10*x_cam + R11*y_cam + R12*z_cam + t.y
    z_base = R20*x_cam + R21*y_cam + R22*z_cam + t.z
    target_point_base = (x_base, y_base, z_base)
    node.get_logger().info(f"Target point in base_link frame: ({x_base:.3f}, {y_base:.3f}, {z_base:.3f})")

    # Compute joint targets based on transformed target
    # 1. Lift: set to target's Z height in base frame
    joint_lift_goal = z_base
    # 2. Wrist extension: horizontal distance to target (sqrt(x^2 + y^2)), clamp to 0.48 m max
    horizontal_dist = math.sqrt(x_base**2 + y_base**2)
    wrist_extension_goal = horizontal_dist if horizontal_dist < 0.48 else 0.48
    # 3. Wrist yaw: horizontal angle to target = atan2(y, x), clamp to [-0.5, 0.5] rad
    yaw = math.atan2(y_base, x_base)
    joint_wrist_yaw_goal = min(0.5, max(-0.5, yaw))
    node.get_logger().info(f"Computed joint targets - lift: {joint_lift_goal:.3f} m, "
                           f"extension: {wrist_extension_goal:.3f} m, yaw: {joint_wrist_yaw_goal:.3f} rad")

    # Command the arm using FollowJointTrajectory action via move_to_pose&#8203;:contentReference[oaicite:6]{index=6}
    goal_pose = {
        'joint_lift': joint_lift_goal,
        'wrist_extension': wrist_extension_goal,
        'joint_wrist_yaw': joint_wrist_yaw_goal
    }
    node.get_logger().info("Sending trajectory to reach target...")
    try:
        node.move_to_pose(goal_pose, blocking=True)  # This will block until motion completes
        node.get_logger().info("Motion command executed successfully.")
    except Exception as e:
        node.get_logger().error(f"Failed to execute motion: {e}")

    # Clean up and shut down the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user, shutting down.")

