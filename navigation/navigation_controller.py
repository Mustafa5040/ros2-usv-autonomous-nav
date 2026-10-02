#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       N A V I G A T I O N _ C O N T R O L L E R . P Y
#
#  Contents:   Wraps nav2 simple commander and other ros2/nav2 messages and provides
#              simple functions to perform navigation and nav2 related operations
#  Notes:
#
#  Author:     mustafa5040   28 dec 2025
#
#  Changes:    Added safety check functionality
#              mustafa5040 - 02 jul 2026 - Ported WGS84, ENU and ECEF Conversion from Hybrid A*
#                                          Added GPS-ENU origin saving mechanism from ardupilot
# -----------------------------------------------------------------------------

from pathlib import Path
import sys

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

from math import cos, sin, sqrt, atan2, asin, radians, hypot, pi
import rclpy
from utils.objects import WGS84GPS, ENU, ECEF
from utils.parameters import GPS
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist, Quaternion, TwistStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
import struct
import tf2_ros
import time

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geographic_msgs.msg import GeoPoseStamped, GeoPointStamped
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup


class NavigationController:
    def __init__(self, parent_node, tf2_buffer, global_frame, robot_frame):

        self.nav = BasicNavigator()
        self.node = parent_node

        self.nav_cb_group = ReentrantCallbackGroup()

        self.cmd_vel_pub = self.node.create_publisher(
            TwistStamped, "/ap/cmd_vel", qos_profile_sensor_data
        )

        self.geo_origin_sub = self.node.create_subscription(
            GeoPointStamped,
            "ap/gps_global_origin/filtered",
            self._origin_callback,
            qos_profile_sensor_data,
            callback_group=self.nav_cb_group,
        )
        self.geo_sub = self.node.create_subscription(
            GeoPoseStamped,
            "/ap/geopose/filtered",
            self.geopose_cb,
            qos_profile_sensor_data,
            callback_group=self.nav_cb_group,
        )

        self.latest_geopose = None
        self.origin_geopose = None
        self.obstacle_publisher = self.node.create_publisher(
            PointCloud2, "/ida/virtual_obstacles", qos_profile_sensor_data
        )

        # Reference Frame
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.tf2_buffer = tf2_buffer

        self.stop()
        self.node.get_logger().info("Navigation Controller is ready...")

    def euler_from_quaternion(self, quaternion):

        x = quaternion[0]
        y = quaternion[1]
        z = quaternion[2]
        w = quaternion[3]
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
            pitch / 2
        ) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
            pitch / 2
        ) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
            pitch / 2
        ) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
            pitch / 2
        ) * sin(yaw / 2)
        return [qx, qy, qz, qw]

    def move_linear(self, distance_meters):
        current_pose = self.get_vehicle_position()
        if current_pose is None:
            self.node.get_logger().error("Could not get the location!")
            return False

        current_yaw = self._get_yaw(current_pose)

        start_x = current_pose.pose.position.x
        start_y = current_pose.pose.position.y

        target_x = start_x + (distance_meters * cos(current_yaw))
        target_y = start_y + (distance_meters * sin(current_yaw))

        self.node.get_logger().info(f"Moving forward: {distance_meters}m")
        return self.go_to_waypoint(target_x, target_y, current_yaw)

    def cancel_current_nav2_task(self):
        """
        Cancels the current pending nav2 tasks
        """
        self.nav.cancelTask()

    def turn_relative_with_nav(self, degrees):
        """
        Turns the boat around itself for +/- degrees.
        Positive: To the left (CCW)
        Negatif: TO the right (CW)
        """
        self.node.get_logger().info(f"Spinning in place: {degrees} degrees")
        
        spin_rad = radians(degrees)
        
        self.nav.spin(spin_dist=spin_rad)
        return True

    def stop(self):
        """
        Emergency stop, both cancel current nav2 task and sends 0 to cmd_vel topic
        """
        self.node.get_logger().warn("[Navigation Controller] Emergency Stop")
        self.nav.cancelTask()
        msg = TwistStamped()
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self.robot_frame
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def is_current_nav_task_completed(self):
        """
        Is the current nav2 task has completed?
        """
        if not self.nav.isTaskComplete():
            return False

        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            return True
        elif result == TaskResult.CANCELED or result == TaskResult.FAILED:
            self.node.get_logger().warn("[NAV2] Could not complete the task! [FAILED]")
            return True

        return False

    def go_to_waypoint(self, x, y, yaw):
        """
        nav.goToPose()
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.global_frame
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()

        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0

        # Euler (Yaw) -> Quaternion dönüşümü
        q = self.quaternion_from_euler(0.0, 0.0, float(yaw))
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.nav.goToPose(goal_pose)
        return True

    def spin_blindly(self, angular_speed=0.5):
        """
        Turn the vehicle around itself blindly, without even using nav2
        """
        msg = TwistStamped()
        msg.header.frame_id = self.robot_frame
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.twist.angular.z = float(angular_speed)
        self.cmd_vel_pub.publish(msg)

    def get_vehicle_position(self):
        try:
            now = rclpy.time.Time()

            if self.tf2_buffer.can_transform(
                self.global_frame, self.robot_frame, now, timeout=Duration(seconds=0.5)
            ):
                trans = self.tf2_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, now
                )
            else:
                self.node.get_logger().warn(
                    "TF Global to Robot Transformation waiting...",
                    throttle_duration_sec=0.2,
                )
                return None

            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            return pose

        except Exception as e:
            self.node.get_logger().error(f"TF Error: {e}")
            return None

    def geopose_cb(self, msg):
        self.latest_geopose = msg

    def get_vehicle_gps(self):
        if self.latest_geopose:
            return (
                self.latest_geopose.pose.position.latitude,
                self.latest_geopose.pose.position.longitude,
            )
        return (0.0, 0.0)

    def _get_yaw(self, pose_stamped):
        q = pose_stamped.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, yaw = self.euler_from_quaternion(orientation_list)
        return yaw

    def publish_points_as_obstacles(self, points_2d):
        if points_2d is None:
            points_2d = []
        header = Header()
        header.frame_id = self.global_frame
        header.stamp = self.node.get_clock().now().to_msg()

        points3d = [(x, y, 0.0) for x, y in points_2d]
        cloud = self.create_pointcloud2_xyz32(header, points3d)

        self.obstacle_publisher.publish(cloud)

    def clear_virtual_obstacles(self):
        # Boş bir liste göndererek haritadaki sanal engelleri temizle
        self.publish_points_as_obstacles([])

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

    def _generate_half_circle_waypoints(
        self, cx, cy, radius, start_angle_rad, num_points=6
    ):
        waypoints = []
        z = False
        for i in range(num_points + 1):
            theta = start_angle_rad + pi * (i / num_points)
            x = cx + radius * cos(theta)
            y = cy + radius * sin(theta)
            yaw = theta + pi / 2
            if z:
                waypoints.append((x, y, yaw))
            z = True

        return waypoints

    def _generate_full_circle_waypoints(
        self, cx, cy, radius, start_angle_rad, num_points=12
    ):
        waypoints = []
        # Start at 1 to skip the 0th point (the boat's current position)
        for i in range(1, num_points + 1):
            # 2 * pi gives us the full 360 degrees
            theta = start_angle_rad + (2 * pi) * (i / num_points)
            x = cx + radius * cos(theta)
            y = cy + radius * sin(theta)
            yaw = theta + pi / 2
            waypoints.append((x, y, yaw))

        return waypoints

    def half_turn_around_point(self, center_x, center_y, radius=2.0):

        current_pose = self.get_vehicle_position()
        if current_pose is None:
            return False

        dx = current_pose.pose.position.x - center_x
        dy = current_pose.pose.position.y - center_y
        start_angle = atan2(dy, dx)

        waypoints = self._generate_half_circle_waypoints(
            center_x, center_y, radius, start_angle, num_points=6
        )

        poses = []
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            q = self.quaternion_from_euler(0.0, 0.0, float(yaw))
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            poses.append(pose)

        self.node.get_logger().info(
            f"[NAV2] Half turn around a point sent... ({len(poses)} points)..."
        )
        self.nav.followWaypoints(poses)
        return True

    def full_turn_around_point(self, center_x, center_y, radius=2.0):
        current_pose = self.get_vehicle_position()
        if current_pose is None:
            return False

        dx = current_pose.pose.position.x - center_x
        dy = current_pose.pose.position.y - center_y
        start_angle = atan2(dy, dx)

        waypoints = self._generate_full_circle_waypoints(
            center_x, center_y, radius, start_angle, num_points=12
        )

        poses = []
        for x, y, yaw in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = self.global_frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0

            q = self.quaternion_from_euler(0.0, 0.0, float(yaw))
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            poses.append(pose)

        self.node.get_logger().info(
            f"[NAV2] Full turn around a point sent.. ({len(poses)} points)..."
        )
        self.nav.followWaypoints(poses)
        return True

    def _origin_callback(self, msg: GeoPointStamped):
        if not msg:
            return
        self.origin_geopose = msg

    def get_origin_gps(self):
        if not self.origin_geopose:
            return None
        point = self.origin_geopose.position
        return [point.latitude, point.longitude, point.altitude]

    def gps_to_local_xy(self, lat, lon, alt):
        origin_gps = self.get_origin_gps()
        if not origin_gps:
            return None
        origin_lat, origin_lon, origin_alt = origin_gps
        enu = self.GEOtoENU(
            WGS84GPS(lat, lon, alt), WGS84GPS(origin_lat, origin_lon, origin_alt)
        )
        if not enu:
            return None
        return enu.east, enu.north, enu.up

    def dist(self, x1, y1, x2, y2):
        return hypot(x1 - x2, y1 - y2)

    def GEOtoECEF(self, referance_gps: WGS84GPS):
        ecef = ECEF(0, 0, 0)
        lat = radians(referance_gps.lat)
        lon = radians(referance_gps.lon)
        alt = referance_gps.alt
        N = GPS.WGS84_AADC / sqrt(cos(lat) * cos(lat) + GPS.WGS84_BBDCC)
        d = (N + alt) * cos(lat)
        ecef.x = d * cos(lon)
        ecef.y = d * sin(lon)
        ecef.z = (GPS.WGS84_P1MEE * N + alt) * sin(lat)
        return ecef

    def ECEFtoENU(self, ecef: ECEF, referance_gps: WGS84GPS):
        x0, y0, z0 = tuple(self.GEOtoECEF(referance_gps))
        dx = ecef.x - x0
        dy = ecef.y - y0
        dz = ecef.z - z0
        lam = radians(referance_gps.lon)
        phi = radians(referance_gps.lat)
        sinp = sin(phi)
        cosp = cos(phi)
        sinl = sin(lam)
        cosl = cos(lam)
        east = -sinl * dx + cosl * dy
        north = -sinp * cosl * dx - sinp * sinl * dy + cosp * dz
        up = cosp * cosl * dx + cosp * sinl * dy + sinp * dz
        return ENU(east, north, up)

    def GEOtoENU(self, gps: WGS84GPS, referance_gps: WGS84GPS):
        ecef = self.GEOtoECEF(gps)
        return self.ECEFtoENU(ecef, referance_gps)


def main(args=None):
    rclpy.init(args=args)
    test_node = rclpy.create_node("nav_test_runner")

    tf_buffer = Buffer(clock=test_node.get_clock())
    tf_listener = TransformListener(tf_buffer, test_node)

    navigator = NavigationController(
        test_node, tf_buffer, global_frame="odom", robot_frame="base_link"
    )

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(test_node)

    print("--- TEST MOD: NAVIGATION CONTROLLER ---")
    print("Test: Going 1 meters forward after 2 seconds...")

    import threading

    def run_test():
        time.sleep(2.0)
        navigator.move_linear(1.0)
        time.sleep(1.0)
        navigator.spin_blindly(1.57)  # 90 degree
        print("Test Over.")

    thread = threading.Thread(target=run_test)
    thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
