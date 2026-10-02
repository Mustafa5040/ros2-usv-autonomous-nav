#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2026.
#
#  File:       N A V I G A T I O N _ C O N T R O L L E R _ M O C K . P Y
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


class NavigationControllerMock:
    def __init__(self, parent_node, tf2_buffer, global_frame, robot_frame):

        self.nav = BasicNavigator()
        self.node = parent_node
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.obstacle_publisher = self.node.create_publisher(
            PointCloud2, "/ida/virtual_obstacles", self.qos_profile
        )

        # Reference Frame
        self.global_frame = global_frame
        self.robot_frame = robot_frame
        self.tf2_buffer = tf2_buffer

        self.stop()
        self.node.get_logger().info("NavigationDriverMock is ready...")

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
        qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(
            roll / 2
        ) * sin(pitch / 2) * sin(yaw / 2)
        qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(
            roll / 2
        ) * cos(pitch / 2) * sin(yaw / 2)
        qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(
            roll / 2
        ) * sin(pitch / 2) * cos(yaw / 2)
        qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(
            roll / 2
        ) * sin(pitch / 2) * sin(yaw / 2)
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
        current_pose = self.get_vehicle_position()
        if current_pose is None:
            return False

        current_yaw = self._get_yaw(current_pose)
        target_yaw = current_yaw + radians(degrees)

        # normalize, (-pi,+pi)
        target_yaw = atan2(sin(target_yaw), cos(target_yaw))

        self.node.get_logger().info(f"Turning: {degrees} degrees")

        return self.go_to_waypoint(
            current_pose.pose.position.x, current_pose.pose.position.y, target_yaw
        )

    def stop(self):
        """
        Emergency stop, both cancel current nav2 task and sends 0 to cmd_vel topic
        """
        self.node.get_logger().warn("[Navigation Controller] Emergency Stop")
        self.nav.cancelTask()

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


    def get_vehicle_position(self):
        try:
            now = rclpy.time.Time() 

            if self.tf2_buffer.can_transform(
                self.global_frame, 
                self.robot_frame, 
                now,
                timeout=Duration(seconds=0.02)
            ):
                trans = self.tf2_buffer.lookup_transform(
                    self.global_frame, self.robot_frame, now
                )
            else:
                self.node.get_logger().warn("TF Global to Robot Transformation waiting...", throttle_duration_sec=1.0)
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
        
    def _get_yaw(self, pose_stamped):
        q = pose_stamped.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = self.euler_from_quaternion(orientation_list)
        return yaw

    def publish_points_as_obstacles(self, points_2d):
        if not points_2d:
            return

        header = Header()
        header.frame_id = self.global_frame
        header.stamp = self.node.get_clock().now().to_msg()

        points3d = [(x, y, 0.0) for x, y in points_2d]
        cloud = self.create_pointcloud2_xyz32(header, points3d)

        self.obstacle_publisher.publish(cloud)

    def clear_virtual_obstacles(self):
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

    def _generate_half_circle_waypoints(self, cx, cy, radius, start_angle_rad, num_points=6):
        waypoints = []
        z=False
        for i in range(num_points + 1):
            theta = start_angle_rad + pi * (i / num_points)
            x = cx + radius * cos(theta)
            y = cy + radius * sin(theta)
            yaw = theta + pi / 2
            if z:
                waypoints.append((x, y, yaw))
            z = True
            
        return waypoints


    def half_turn_around_point(self, center_x, center_y, radius=2.0):
        current_pose = self.get_vehicle_position()
        if current_pose is None:
            return False

        dx = current_pose.pose.position.x - center_x
        dy = current_pose.pose.position.y - center_y
        start_angle = atan2(dy, dx)

        waypoints = self._generate_half_circle_waypoints(
            center_x,
            center_y,
            radius,
            start_angle,
            num_points=6
        )

        for x, y, yaw in waypoints:
            self.go_to_waypoint(x, y, yaw)

            while not self.nav.isTaskComplete():
                time.sleep(0.1)
            
            if self.nav.getResult() != TaskResult.SUCCEEDED:
                self.node.get_logger().warn("ERROR ON HALF TURN!")
                return False

        return True
    def full_turn_around_point(self, center_x, center_y, radius=2.0):
        current_pose = self.get_vehicle_position()
        if current_pose is None:
            return False

        dx = current_pose.pose.position.x - center_x
        dy = current_pose.pose.position.y - center_y
        start_angle = atan2(dy, dx)

        # 12 points usually provide a smooth enough curve for Nav2 without overwhelming it
        waypoints = self._generate_full_circle_waypoints(
            center_x,
            center_y,
            radius,
            start_angle,
            num_points=12
        )

        for x, y, yaw in waypoints:
            self.go_to_waypoint(x, y, yaw)

            while not self.nav.isTaskComplete():
                time.sleep(0.1)
            
            if self.nav.getResult() != TaskResult.SUCCEEDED:
                self.node.get_logger().warn("ERROR ON FULL TURN!")
                return False

        return True
    def _generate_full_circle_waypoints(self, cx, cy, radius, start_angle_rad, num_points=12):
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
    
    def dist(self,x1, y1, x2, y2):
        return hypot(x1 - x2, y1 - y2)
    

def main(args=None):
    rclpy.init(args=args)
    test_node = rclpy.create_node("nav_test_runner")

    tf_buffer = Buffer(clock=test_node.get_clock())
    tf_listener = TransformListener(tf_buffer, test_node)

    navigator = NavigationControllerMock(
        test_node, tf_buffer, global_frame="odom", robot_frame="base_link"
    )

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(test_node)

    print("--- TEST MOD: NAVIGATION CONTROLLER ---")
    print("Test: Going 1 meters forward after 2 seconds...")

    import threading
    def run_test():
        time.sleep(2.0)
        navigator.move_linear(1.0)
        time.sleep(1.0)
        print("Test bitti.")

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