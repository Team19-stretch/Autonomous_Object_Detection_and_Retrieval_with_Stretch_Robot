import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__('frame_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Create a timer to periodically lookup transforms
        self.timer = self.create_timer(1.0, self.lookup_transform)

    def lookup_transform(self):
        from_frame_rel = 'camera_color_optical_frame'
        to_frame_rel = 'base_link'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
            self.get_logger().info(
                f'Transform from {from_frame_rel} to {to_frame_rel}:'
                f'translation={t.transform.translation},'
                f'rotation={t.transform.rotation}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()