#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       G P S _ I N I T I A L . P O S E
#
#  Contents:   sets the initial pose for nav2 based on gps data.
#
#  Notes:      Could be added timeout instead of static trying count
#
#  Author:     mustafa5040   28 dec 2025
#             
#  Changes:    mustafa5040 - 03 jul 2026 - Changed to publish more than once to wait initialization of Ardupilot/DDS insteaf of immediately closing.
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class GPSInitialPose(Node):
    def __init__(self):
        super().__init__('gps_initialpose_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            qos_profile
        )

        self.sub = self.create_subscription(
            PoseStamped,
            '/ap/pose/filtered',
            self.cb,
            qos_profile
        )
        self.pub_count = 0
        self.limit = 5
        self.get_logger().info("Waiting for GPS pose to publish initialpose...")

    def cb(self, msg: PoseStamped):
        if self.pub_count > self.limit:
            self.get_logger().info(f"Initial pose already published {self.limit} times. Initial pose already set. Destroying node...")
            self.destroy_node()
            rclpy.shutdown()
            return

        init = PoseWithCovarianceStamped()
        init.header.frame_id = "odom"
        init.header.stamp = self.get_clock().now().to_msg()

        init.pose.pose = msg.pose

        init.pose.covariance = [
            1.0, 0, 0, 0, 0, 0,
            0, 1.0, 0, 0, 0, 0,
            0, 0, 10.0, 0, 0, 0,
            0, 0, 0, 0.5, 0, 0,
            0, 0, 0, 0, 0.5, 0,
            0, 0, 0, 0, 0, 0.5
        ]

        self.pub.publish(init)
        self.pub_count += 1
        self.get_logger().info(f"Initial pose published {self.pub_count} times")
        self.done = True