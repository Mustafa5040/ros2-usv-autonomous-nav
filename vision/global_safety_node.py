#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2026.
#
#  File:       G L O B A L _ S A F E T Y _ N O D E . P Y
#
#  Contents:   Runs continuously in the background independent of missions.
#              Tracks ALL buoys and publishes them to local costmap to 
#              prevent collisions even in manual mode or when no task is running.
#
# -----------------------------------------------------------------------------

from pathlib import Path
import sys
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from depthai_ros_msgs.msg import SpatialDetectionArray
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import tf2_ros

from vision.perception_system_ex import PerceptionSystem
from utils.objects import Buoy  # Duba sınıfını kontrol etmek için

class GlobalSafetyNode(Node):
    def __init__(self):
        super().__init__("global_safety_node")
        self.global_frame = "odom"
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.perception_system = PerceptionSystem(self.tf_buffer, self, self.global_frame)
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.obstacle_publisher = self.create_publisher(PointCloud2, "/ida/virtual_obstacles", self.qos_profile)
        
        self.create_subscription(
            SpatialDetectionArray,
            "/oak/nn/spatial_detections",
            self.fusion_callback,
            qos_profile_sensor_data,
        )
        
        self.timer = self.create_timer(0.2, self.publish_obstacles_cb)
        self.get_logger().info("Global Safety Node is READY! Universal Shield is active.")

    def fusion_callback(self, msg):
        self.perception_system.update(msg)

    def publish_obstacles_cb(self):
        points_2d = []
        
        for obj in self.perception_system.detected_objects:
            if isinstance(obj, Buoy): 
                points_2d.append((obj.global_x, obj.global_y))
        
        self.publish_points_as_obstacles(points_2d)
        
    def publish_points_as_obstacles(self, points_2d):

        if points_2d is None:
            points_2d = []

        header = Header()
        header.frame_id = self.global_frame

        header.stamp = self.get_clock().now().to_msg()

        points3d = [(x, y, 0.0) for x, y in points_2d]
        cloud = self.create_pointcloud2_xyz32(header, points3d)

        self.obstacle_publisher.publish(cloud)

    def create_pointcloud2_xyz32(self, header, points):
        """
        Create a PointCloud2 message from [(x,y,z), ...] without sensor_msgs_py
        """
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_step = 12  # 3 * float32
        data = bytearray()

        for x, y, z in points:
            data.extend(struct.pack("fff", x, y, z))

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = point_step
        cloud.row_step = point_step * len(points)
        cloud.data = bytes(data)
        cloud.is_dense = True

        return cloud

def main(args=None):
    rclpy.init(args=args)
    node = GlobalSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()