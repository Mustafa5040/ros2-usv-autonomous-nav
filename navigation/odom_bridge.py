#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2026.
#
#  File:       O D O M _ B R I D G E . P Y
#
#  Contents:   Bridge to publish nav_msgs/Odometry + odom->base_link TF from
#              ArduPilot's AP_DDS pose/twist topics.
#
#  NOTE ON FRAMES (this is where the old version was wrong):
#    /ap/pose/filtered  and  /ap/twist/filtered  are published by AP_DDS
#    ALREADY IN LOCAL ENU, relative to home. AP_DDS does the NED->ENU
#    conversion internally. Do NOT convert again -- a second conversion
#    mirrors the frame and leaves the reported heading a constant 90 deg
#    away from the actual direction of travel.
#
#    However, /ap/twist/filtered is in the *world* ENU frame, while
#    nav_msgs/Odometry.twist is defined in child_frame_id (base_link).
#    So the linear velocity still has to be rotated by -yaw into body frame.
# -----------------------------------------------------------------------------
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


TWIST_TIMEOUT_S = 0.5      # older than this -> report zero velocity
JUMP_POS_M = 0.75          # position discontinuity warning threshold
JUMP_YAW_RAD = 0.6         # heading discontinuity warning threshold (~34 deg)


def yaw_from_quaternion(q):
    """Planar yaw (rotation about Z) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
    """Shortest signed difference a - b, wrapped to [-pi, pi]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


class OdomBridge(Node):
    def __init__(self):
        super().__init__("odom_bridge")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("warn_on_jump", True)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.warn_on_jump = self.get_parameter("warn_on_jump").value

        # AP_DDS publishes best-effort; match it or you get nothing.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        cb_group = ReentrantCallbackGroup()

        self.create_subscription(
            PoseStamped,
            "/ap/pose/filtered",
            self.pose_cb,
            qos_profile,
            callback_group=cb_group,
        )
        self.create_subscription(
            TwistStamped,
            "/ap/twist/filtered",
            self.twist_cb,
            qos_profile,
            callback_group=cb_group,
        )

        self.pub_odom = self.create_publisher(Odometry, "/odometry", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_twist = None
        self.last_twist_recv_time = None

        self.prev_pos = None
        self.prev_yaw = None

        self.stale_warned = False

        self.get_logger().info(
            "Odom Bridge started (AP_DDS is already ENU -- no NED conversion applied). "
            f"frames: {self.odom_frame} -> {self.base_frame}"
        )

    # ------------------------------------------------------------------ #

    def twist_cb(self, msg):
        self.last_twist = msg
        self.last_twist_recv_time = self.get_clock().now()

    def pose_cb(self, msg):
        now = self.get_clock().now()

        # AP_DDS pose is already ENU relative to home -- pass it through as-is.
        pose = msg.pose
        yaw = yaw_from_quaternion(pose.orientation)

        self.check_for_jump(pose, yaw)

        odom = Odometry()
        # Re-stamp with the ROS clock. AP_DDS stamps with ArduPilot's boot
        # clock, which would make every TF lookup in Nav2 fail.
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose

        self.fill_body_twist(odom, yaw, now)

        # Rough, static covariances. Tighten these if you later feed this into
        # robot_localization; Nav2 alone does not use them.
        odom.pose.covariance[0] = 0.5      # x
        odom.pose.covariance[7] = 0.5      # y
        odom.pose.covariance[14] = 0.5     # z
        odom.pose.covariance[35] = 0.05    # yaw
        odom.twist.covariance[0] = 0.1     # vx
        odom.twist.covariance[7] = 0.1     # vy
        odom.twist.covariance[35] = 0.05   # wz

        self.pub_odom.publish(odom)

        # TF is broadcast unconditionally. Never gate this on twist freshness:
        # a dropped TF freezes the robot in RViz and makes it appear to
        # teleport when the stream resumes.
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = pose.position.x
            t.transform.translation.y = pose.position.y
            t.transform.translation.z = pose.position.z
            t.transform.rotation = pose.orientation
            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------ #

    def fill_body_twist(self, odom, yaw, now):
        """Rotate world-frame ENU velocity into base_link.

        nav_msgs/Odometry.twist is defined in child_frame_id, but ArduPilot
        reports velocity in the world ENU frame. Without this rotation,
        RegulatedPurePursuitController reads a linear.x that depends on
        heading rather than speed, which collapses the velocity-scaled
        lookahead and pins the rover at regulated_linear_scaling_min_speed.
        """
        if self.last_twist is None or self.last_twist_recv_time is None:
            return

        age = (now - self.last_twist_recv_time).nanoseconds / 1e9
        if age > TWIST_TIMEOUT_S:
            if not self.stale_warned:
                self.get_logger().warn(
                    f"/ap/twist/filtered stale ({age:.2f}s) -- reporting zero velocity"
                )
                self.stale_warned = True
            return  # leave twist at zero rather than publishing stale values

        if self.stale_warned:
            self.get_logger().info("/ap/twist/filtered recovered")
            self.stale_warned = False

        vx_w = self.last_twist.twist.linear.x
        vy_w = self.last_twist.twist.linear.y

        c = math.cos(yaw)
        s = math.sin(yaw)

        odom.twist.twist.linear.x = c * vx_w + s * vy_w
        odom.twist.twist.linear.y = -s * vx_w + c * vy_w
        odom.twist.twist.linear.z = self.last_twist.twist.linear.z

        # For a planar vehicle the yaw rate is identical in body and world.
        odom.twist.twist.angular.x = self.last_twist.twist.angular.x
        odom.twist.twist.angular.y = self.last_twist.twist.angular.y
        odom.twist.twist.angular.z = self.last_twist.twist.angular.z

    def check_for_jump(self, pose, yaw):
        """Log pose discontinuities.

        AP_DDS pose is relative to HOME, not the EKF origin. If home is re-set
        mid-run (rearm, GCS 'set home here', arming update) the pose shifts
        discontinuously -- and because the Nav2 stack here is entirely in the
        odom frame, the goal effectively teleports with it.
        """
        if not self.warn_on_jump:
            return

        pos = (pose.position.x, pose.position.y)

        if self.prev_pos is not None:
            d = math.hypot(pos[0] - self.prev_pos[0], pos[1] - self.prev_pos[1])
            dyaw = abs(angle_diff(yaw, self.prev_yaw))
            if d > JUMP_POS_M or dyaw > JUMP_YAW_RAD:
                self.get_logger().warn(
                    f"Pose discontinuity: {d:.2f} m, {math.degrees(dyaw):.1f} deg "
                    "in one update -- check whether ArduPilot re-set HOME."
                )

        self.prev_pos = pos
        self.prev_yaw = yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()