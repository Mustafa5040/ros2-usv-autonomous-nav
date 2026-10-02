#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       C M D _ V E L _ B R I D G E . P Y
#
#  Contents:   Gets /cmd_vel values from other nodes and nav2 then publishes to ardupilot via /ap/cmd_vel
#
#  Notes:      
#
#  Author:     mustafa5040 - 28 dec 2025
#
#  Changes:    Removed tf and odom publishers, added safety checks, rewritten stop msg - 11 jun 2026 - mustafa5040 
# -----------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
class Nav2Bridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.get_logger().info("CMD_VEL BRIDGE STARTED..")

        self.cmd_cb_group = ReentrantCallbackGroup()
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10, callback_group=self.cmd_cb_group)
        self.pub_ap_vel = self.create_publisher(TwistStamped, '/ap/cmd_vel', qos_profile_sensor_data)
        self.latest_cmd_vel = TwistStamped()
        self.last_cmd_time = self.get_clock().now()

        self.timer = self.create_timer(0.1, self.safety_check)
        self.cmd_timeout = 2.0  # seconds

    def cmd_vel_cb(self, msg: Twist):
        cmd_msg = TwistStamped()
        cmd_msg.header.stamp.sec = 0
        cmd_msg.header.stamp.nanosec = 0
        cmd_msg.header.frame_id = 'base_link'
        cmd_msg.twist = msg
        self.latest_cmd_vel = cmd_msg
        self.last_cmd_time = self.get_clock().now()
        self.pub_ap_vel.publish(cmd_msg)

    def safety_check(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9 #seconds
        if elapsed > self.cmd_timeout:
            self.stop_cmd()

    def stop_cmd(self):
        stop = TwistStamped()
        stop.header.stamp.sec = 0
        stop.header.stamp.nanosec = 0
        stop.header.frame_id = 'base_link'
        stop.twist.linear.x = 0.0
        stop.twist.angular.z = 0.0
        self.pub_ap_vel.publish(stop)
def main(args=None):
    rclpy.init(args=args)
    node = Nav2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_cmd()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()