#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       S T A T E _ M A C H I N E . P Y
#
#  Contents:   Main entrypoint of whole autonomus system, manages states, tasks, provides
#              necessary modules and classes etc.
#
#  Notes:
#
#  Author:     mustafa5040   25 dec 2025
#
#  Changes:    Removed Roboboat-related legacy code, Updated objects topic to /oak/nn/spatial_detections - 18 jun 2026 - mustafa5040
#              mustafa5040 - 30 jun 2026 - Removed old roboboat releated legacy code, fixed global framr
#
# -----------------------------------------------------------------------------
from pathlib import Path
import sys

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from vision_msgs.msg import Detection2DArray
from depthai_ros_msgs.msg import SpatialDetectionArray
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from utils.objects import *
from utils.parameters import *
from vision.perception_system_ex import PerceptionSystem
from navigation.navigation_controller import NavigationController
from tasks.task1 import Task1
from tasks.task2 import Task2
from tasks.task3 import Task3
from rclpy.time import Time
from simulation.navigation_controller_mock import NavigationControllerMock

# from simulation.oak_node_mock import OakDNodeMock
import tf2_ros
from tasks.base_gate_task import GateResult
from nav2_simple_commander.robot_navigator import TaskResult as NavTaskResult


class StateMachine(Node):
    def __init__(self):
        super().__init__("state_machine")
        self.SIM_MODE = False
        self.get_logger().info("Starting StateMachine...")

        self.global_frame = "odom"
        self.robot_frame = "base_link"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.perception_system = PerceptionSystem(
            self.tf_buffer, self, self.global_frame
        )
        self.nav_controller = NavigationController(
            self, self.tf_buffer, self.global_frame, self.robot_frame
        )

        self.tasks = [Task1(self.nav_controller, self.tf_buffer, self),  Task2(self.nav_controller, self.tf_buffer, self), Task3(self.nav_controller, self.tf_buffer, self)]
        self.current_task_idx = 0

        self.sub_cb_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            SpatialDetectionArray,
            "/oak/nn/spatial_detections",
            self.fusion_callback,
            qos_profile_sensor_data,
            callback_group=self.sub_cb_group,
        )

        self.timer = self.create_timer(
            SYS.logic_period, self.control_loop, callback_group=self.timer_group
        )

        self.start_pos = None
        try:
            self.tf_buffer.lookup_transform(
                self.global_frame,
                "oak_rgb_camera_optical_frame",
                Time(0),
                timeout=Duration(seconds=0.02),
            )
        except Exception:
            self.get_logger().warn(
                "Camera TF not ready yet...", throttle_duration_sec=2.0
            )
            pass

        self.get_logger().info("State Machine is ready...")

    def fusion_callback(self, msg):
        self.perception_system.update(msg)

    def finish_everything(self):
        self.nav_controller.stop()
        self.timer.cancel()

    def control_loop(self):
        if self.start_pos is None:
            self.start_pos = self.nav_controller.get_vehicle_position()
        if self.current_task_idx >= len(self.tasks):
            self.get_logger().info("All tasks has finished...")
            self.finish_everything()
            # TODO: what to do when finished? Maybe disarm with Ros2 service calls?
            return
        current_task = self.tasks[self.current_task_idx]

        self.get_logger().info(
            f"[STATE MACHINE] Current task: {self.current_task_idx+1} "
        )

        pose_stamped = self.nav_controller.get_vehicle_position()

        if pose_stamped is None:
            self.get_logger().warn(
                "TF Location is waiting for 2 seconds...", throttle_duration_sec=2.0
            )
            return

        if self.start_pos is None:
            self.start_pos = pose_stamped

        current_yaw = self.nav_controller._get_yaw(pose_stamped)

        vehicle_pos = (
            pose_stamped.pose.position.x,
            pose_stamped.pose.position.y,
            current_yaw,
        )
        result = current_task.run(self.perception_system.detected_objects, vehicle_pos)

        self.get_logger().info(f"THE CURRENT TASK'S RESULT IS: {str(result)} ")
        if result == TaskResult.SUCCESS:
            self.get_logger().info(
                f"TASK {self.current_task_idx} FINISHED! Starting the next task..."
            )
            self.nav_controller.stop()
            self.perception_system.detected_objects.clear()
            self.current_task_idx += 1

        elif result == TaskResult.FAILED:
            self.get_logger().error(f"TASK {self.current_task_idx+1} HAS FAILED!")
            self.nav_controller.stop()
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    try:
        state_machine_node = StateMachine()

        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(state_machine_node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            state_machine_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
