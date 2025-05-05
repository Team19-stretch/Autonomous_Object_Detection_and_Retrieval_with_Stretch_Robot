#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray
import hello_helpers.hello_misc as hm
import time
import threading
import math

class HeadAndAlignToMarkerNode(hm.HelloNode):
    def __init__(self):
        super().__init__()
        self.marker_detected = False
        self.marker_x = None
        self.marker_y = None
        self.marker_z = None
        self.cmd_vel_pub = None

    def marker_callback(self, msg):
        for marker in msg.markers:
            if marker.id == 4 and marker.pose.position.x != 0.0000 and marker.pose.position.y != 0.0000 and marker.pose.position.z != 0.0000 :
                self.marker_detected = True
                self.marker_x = marker.pose.position.x
                self.marker_y = marker.pose.position.y
                self.marker_z = marker.pose.position.z


    def main(self):
        super().main('head_marker_align_node', 'head_marker_align_node')

        # Step 1: Center head pan and tilt
        self.get_logger().info("Centering head pan and tilt...")
        self.move_to_pose({'joint_head_pan': 0.0, 'joint_head_tilt': -0.3}, blocking=True)

        # Step 2: Setup publisher & subscriber
        self.cmd_vel_pub = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        self.create_subscription(MarkerArray, '/objects/marker_array', self.marker_callback, 10)

        # Step 3: Begin main logic in background
        threading.Thread(target=self.rotate_and_align).start()

    def rotate_and_align(self):
        # Step 3: Rotate to scan for marker
        twist = Twist()
        twist.angular.z = 0.1  # Slow scan
        duration = (2 * math.pi) / twist.angular.z  # 360° rotation duration
        start_time = time.time()

        self.get_logger().info("Scanning for marker ID 6 at 0.1 rad/s...")

        while rclpy.ok() and not self.marker_detected and (time.time() - start_time < duration):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self.cmd_vel_pub.publish(Twist())

        if not self.marker_detected:
            self.get_logger().warn("Marker not found after full rotation.")
            rclpy.shutdown()
            return

        # Step 4: Align Y axis
        self.get_logger().info("Marker found. Starting fine alignment on Y...")

        while abs(self.marker_y) > 0.01 and rclpy.ok():
            correction_twist = Twist()
            if self.marker_y > 0.01:
                correction_twist.angular.z = 0.05
                self.get_logger().info(f"Y={self.marker_y:.4f} → Turning left")
            elif self.marker_y < -0.01:
                correction_twist.angular.z = -0.05
                self.get_logger().info(f"Y={self.marker_y:.4f} → Turning right")
            self.cmd_vel_pub.publish(correction_twist)
            time.sleep(0.2)
            self.cmd_vel_pub.publish(Twist())
            time.sleep(0.2)

        self.get_logger().info("Y alignment complete! Y axis ≈ 0.00.")

        # Step 5: Align X and Z
        target_z = 0.75
        x_threshold = 0.03
        z_threshold = 0.01
        head_tilt = 0.0

        self.get_logger().info("Starting X-Z alignment loop...")

        while rclpy.ok():
            if self.marker_x is None or self.marker_z is None:
                self.get_logger().warn("Marker lost during X/Z alignment. Stopping.")
                break

            # Adjust X using tilt
            if abs(self.marker_x) > x_threshold:
                if self.marker_x > 0.01 and head_tilt > -2.02:
                    head_tilt -= 0.02  # Tilt down (right)
                    head_tilt = max(head_tilt, -2.02)
                    self.get_logger().info(f"X={self.marker_x:.4f} → Tilting down to {head_tilt:.2f}")
                elif self.marker_x < -0.01 and head_tilt < 0.49:
                    head_tilt += 0.02  # Tilt up (left)
                    head_tilt = min(head_tilt, 0.49)
                    self.get_logger().info(f"X={self.marker_x:.4f} → Tilting up to {head_tilt:.2f}")
                self.move_to_pose({'joint_head_tilt': head_tilt}, blocking=True)
                time.sleep(0.2)
                continue

            # Adjust Z using base
            if abs(self.marker_z - target_z) > z_threshold:
                move_twist = Twist()
                if self.marker_z > target_z:
                    move_twist.linear.x = 0.05  # forward
                    self.get_logger().info(f"Z={self.marker_z:.4f} → Moving forward")
                elif self.marker_z < target_z:
                    move_twist.linear.x = -0.05  # backward
                    self.get_logger().info(f"Z={self.marker_z:.4f} → Moving backward")
                self.cmd_vel_pub.publish(move_twist)
                time.sleep(0.3)
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.3)
                continue

            self.get_logger().info("X and Z alignment complete.")
            break

        # Step 6: Rotate robot base 90° left
        self.get_logger().info("Rotating robot base 90° to the left...")
        rotate_twist = Twist()
        rotate_twist.angular.z = 0.2
        rotate_duration = (math.pi / 2) / rotate_twist.angular.z
        start_time = time.time()
        while time.time() - start_time < rotate_duration:
            self.cmd_vel_pub.publish(rotate_twist)
            time.sleep(0.1)
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("Robot base rotation complete.")

        # Step 7: Pan head 90° to the right
        self.get_logger().info("Panning head 90° to the right (1.57 rad)...")
        self.move_to_pose({'joint_head_pan': -1.57}, blocking=True)

        self.get_logger().info("✅ Final orientation complete. Arm is facing object, head is watching it.")
        rclpy.shutdown()


def main():
    node = HeadAndAlignToMarkerNode()
    node.main()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
