import sys

import open3d as o3d
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import Marker

def quaternion_to_rotation_matrix(q):
    """Converts a ROS Quaternion message to a 3x3 NumPy rotation matrix."""
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z); t1 = +1.0 - 2.0 * (x * x + y * y)
    r00 = t1; r10 = t0; r20 = +2.0 * (w * y - z * x)
    t2 = +2.0 * (w * y + z * x); t3 = +1.0 - 2.0 * (y * y + z * z)
    r01 = +2.0 * (x * y - w * z); r11 = t3; r21 = t2
    t4 = +2.0 * (w * z + x * y); t5 = +1.0 - 2.0 * (z * z + x * x)
    r02 = +2.0 * (x * z + w * y); r12 = t4; r22 = t5
    return np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

class PointCloudTransformerNode(Node):
    def __init__(self, ply_file_path):
        super().__init__('point_cloud_transformer_node')
        
        self.get_logger().info(f"Loading PLY file: {ply_file_path}")
        try:
            pcd = o3d.io.read_point_cloud(ply_file_path)
            if not pcd.has_points(): raise ValueError("PLY file contains no points.")
            points_original = np.asarray(pcd.points, dtype=np.float32)
        except Exception as e:
            self.get_logger().error(f"Failed to load or process file: {e}"); raise e

        # ========================= MODIFIED: Center using Bounding Box =========================
        # 1. Get the axis-aligned bounding box (AABB) of the original point cloud.
        aabb = pcd.get_axis_aligned_bounding_box()
        # 2. Get the center of the AABB. This matches the (min + max) * 0.5 logic.
        bbox_center = aabb.get_center()
        self.get_logger().info(f"Calculated AABB center: {bbox_center}")
        # 3. Translate all points by subtracting the AABB center.
        points_centered = points_original - bbox_center
        self.get_logger().info("Centered the point cloud based on its bounding box.")
        # =======================================================================================

        # Perform initial coordinate system swap (Y-up to Z-up) on the *centered* points.
        self.base_points_ros = np.zeros_like(points_centered)
        self.base_points_ros[:, 0] = points_centered[:, 2]
        self.base_points_ros[:, 1] = -points_centered[:, 0]
        self.base_points_ros[:, 2] = points_centered[:, 1]
        self.get_logger().info(f"Successfully loaded and converted {len(self.base_points_ros)} points.")

        # --- Initialize transform variables and ROS communication ---
        self.latest_transform = {}
        self.transform_received = False

        self.transform_subscriber = self.create_subscription(
            Marker,
            '/background_tranform',
            self.transform_callback,
            10)
        
        self.cloud_publisher = self.create_publisher(PointCloud2, '/background_cloud_reliable', 10)
        self.publish_timer = self.create_timer(0.1, self.publish_transformed_cloud)
        self.get_logger().info("Node started. Waiting for transform on /background_tranform...")

    def transform_callback(self, msg):
        """Callback to store the latest transform from the Marker message."""
        if not self.transform_received:
            self.get_logger().info("Success! First transform received from /background_tranform.")
            self.get_logger().info(f"  > Translation: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}")
            self.get_logger().info(f"  > Rotation (Quat): x={msg.pose.orientation.x:.3f}, y={msg.pose.orientation.y:.3f}, z={msg.pose.orientation.z:.3f}, w={msg.pose.orientation.w:.3f}")
            self.get_logger().info(f"  > Scale: x={msg.scale.x:.3f}, y={msg.scale.y:.3f}, z={msg.scale.z:.3f}")
        
        self.latest_transform["translation"] = msg.pose.position
        self.latest_transform["rotation"] = msg.pose.orientation
        self.latest_transform["scale"] = msg.scale
        self.transform_received = True

    def publish_transformed_cloud(self):
        """Applies the stored transform and publishes the point cloud."""
        points_to_publish = self.base_points_ros.copy()

        if self.transform_received:
            # Apply transform in Scale -> Rotate -> Translate order.
            # 1. Scale (around the now-centered origin)
            scale_vec = self.latest_transform["scale"]
            points_to_publish *= np.array([scale_vec.x, scale_vec.y, scale_vec.z])
            # 2. Rotate (around the now-centered origin)
            quat = self.latest_transform["rotation"]
            rot_matrix = quaternion_to_rotation_matrix(quat)
            points_to_publish = np.dot(points_to_publish, rot_matrix.T)
            # 3. Translate (move the transformed object to its final position)
            trans = self.latest_transform["translation"]
            points_to_publish += np.array([trans.x, trans.y, trans.z])
        
        msg = self.create_point_cloud2_msg("world", points_to_publish.astype(np.float32))
        self.cloud_publisher.publish(msg)

    def create_point_cloud2_msg(self, frame_id, points):
        """Helper function to create a PointCloud2 message."""
        header = Header(frame_id=frame_id, stamp=self.get_clock().now().to_msg())
        fields = [ PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz') ]
        return PointCloud2(
            header=header, height=1, width=points.shape[0], is_dense=True, is_bigendian=False,
            fields=fields, point_step=12, row_step=12 * points.shape[0], data=points.tobytes()
        )

def main(args=None):
    argv = remove_ros_args(args=sys.argv)
    if len(argv) < 2:
        sys.exit("usage: ros2 run point_cloud public_ply <background.ply>")
    ply_file_path = argv[1]
    rclpy.init(args=args)
    node = None
    try:
        node = PointCloudTransformerNode(ply_file_path)
        rclpy.spin(node)
    except Exception as e:
        if node: node.get_logger().fatal(f"Node runtime error: {e}")
        else: print(f"Node initialization error: {e}")
    finally:
        if node: node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()