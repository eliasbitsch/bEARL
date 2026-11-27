"""Demo only: grow a live map from GLIM's aligned scans for RViz.

GLIM publishes /glim_ros/map only when a submap is finished. In a small room (M3DGR indoor) the whole
sequence fits into one submap, so the map would stay empty until the end. This node collects every
/glim_ros/aligned_points scan, keeps one point per voxel (10 cm) and republishes the result once per
second on /demo/map. Visualisation only, it does not touch GLIM's estimate.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

VOXEL = 0.1


class AccumulateMap(Node):
    def __init__(self):
        super().__init__("demo_accumulate_map")
        self.keys = np.empty(0, dtype=np.int64)
        self.points = np.empty((0, 3), dtype=np.float32)
        self.header = None
        self.create_subscription(PointCloud2, "/glim_ros/aligned_points", self.on_scan,
                                 QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.pub = self.create_publisher(PointCloud2, "/demo/map", QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(1.0, self.publish)

    def on_scan(self, msg):
        xyz = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z")).astype(np.float32)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        v = np.floor(xyz / VOXEL).astype(np.int64) + 2**20          # voxel index, offset to stay positive
        keys = (v[:, 0] << 42) | (v[:, 1] << 21) | v[:, 2]
        keys, idx = np.unique(keys, return_index=True)
        new = ~np.isin(keys, self.keys)
        self.keys = np.concatenate([self.keys, keys[new]])
        self.points = np.concatenate([self.points, xyz[idx[new]]])
        self.header = msg.header

    def publish(self):
        if self.header is not None:
            self.pub.publish(point_cloud2.create_cloud_xyz32(self.header, self.points))


def main():
    rclpy.init()
    rclpy.spin(AccumulateMap())


if __name__ == "__main__":
    main()
