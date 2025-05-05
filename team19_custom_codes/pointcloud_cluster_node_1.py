import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray, GoalStatus
import subprocess
import time

class NavGoalMonitor(Node):
    def __init__(self):
        super().__init__('stretch_nav_monitor')
        # Subscribe to the NavigateToPose action status topic
        self.subscription = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10  # QoS depth
        )
        self.triggered = False  # Flag to avoid retriggering actions
        self.get_logger().info("NavGoalMonitor node started, listening for goal status...")

    def status_callback(self, msg: GoalStatusArray):
        # If there are any statuses, check the first (newest) goal's status code
        if not msg.status_list:
            return  # No status entries to evaluate
        latest_status = msg.status_list[0]   # assume first entry is the latest goal&#8203;:contentReference[oaicite:5]{index=5}
        status_code = latest_status.status

        # Check if the goal succeeded (STATUS_SUCCEEDED is represented by code 3)
        if status_code == GoalStatus.STATUS_SUCCEEDED:  # status_code == 3
            if not self.triggered:
                self.triggered = True  # mark as handled
                self.get_logger().info("Navigation goal SUCCEEDED. Triggering post-goal actions...")

                # Send SIGINT to the visual_navigation.launch.py process (simulate Ctrl+C)
                try:
                    subprocess.run(['pkill', '-SIGINT', '-f', 'visual_navigation.launch.py'], check=False)
                    self.get_logger().info("Sent SIGINT to 'visual_navigation.launch.py' process. Waiting 10 seconds for shutdown...")
                except Exception as e:
                    self.get_logger().error(f"Failed to send SIGINT: {e}")
                    return  # Exit callback if we can't send the signal

                # Wait for 10 seconds to allow the navigation launch to shut down gracefully
                time.sleep(10)

                # Launch the automate_robot.sh script
                try:
                    subprocess.run(['/home/team19/automate_robot.sh'], check=True)
                    self.get_logger().info("Launched '/home/team19/automate_robot.sh' successfully.")
                except Exception as e:
                    self.get_logger().error(f"Failed to launch automate_robot.sh: {e}")
            else:
                # Already triggered for this goal, ignore additional succeeded statuses
                self.get_logger().info("Success event already handled. Ignoring duplicate status.")
        # (If needed, you could handle other status codes like aborted or canceled here as elif)

def main(args=None):
    rclpy.init(args=args)
    node = NavGoalMonitor()
    try:
        rclpy.spin(node)  # Keep the node running to monitor statuses
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down.")
    finally:
        # Clean up on exit
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

