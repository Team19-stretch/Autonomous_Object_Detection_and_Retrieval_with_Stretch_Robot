import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudCentroidNode(Node):
    def __init__(self):
        super().__init__('pointcloud_centroid_node')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/objects/point_cloud2',
            self.pointcloud_callback,
            qos_profile_sensor_data)
        self.get_logger().info('PointCloudCentroidNode started, subscribed to /objects/point_cloud2')

    def pointcloud_callback(self, msg: PointCloud2):
        self.get_logger().info("Point cloud received")

        try:
            # Extract valid (x, y, z) points, skipping NaNs
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if not points:
                self.get_logger().info('No valid points received.')
                return
            points_array = np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f'Failed to parse point cloud: {e}')
            return

        # Compute centroid of all valid points
        centroid = np.mean(points_array, axis=0)
        x, y, z = centroid
        self.get_logger().info(f'Point cloud centroid (camera_color_optical_frame): '
                               f'x={x:.3f}, y={y:.3f}, z={z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudCentroidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

