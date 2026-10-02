# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       T A S K 1 . P Y
#
#  Contents:   Logic for Task 1 of Teknofest 2026
#
#  Notes:
#
#  Author:     mustafa5040   25 dec 2025
#
#  Changes:    sefasahin   - 16 jun 2026 - Adapted to teknofest
#              mustafa5040 - 30 jun 2026 - Fixed certain bugs, prevented U turns, Added tracking check
#              mustafa5040 - 08 jul 2026 - Fixed Logical errors, tested on simulation and fixed certain bugs
# -----------------------------------------------------------------------------
from pathlib import Path
import sys
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

from tasks.base_gate_task import BaseGateTask, GateInternalState, GateResult, TaskResult
from utils.parameters import *
from utils.objects import *
from math import hypot, atan2
import time

class Task1(BaseGateTask):
    def __init__(self, navigation_controller, tf_buffer, parent_node):
        super().__init__(navigation_controller, tf_buffer, parent_node)
        self.parent_node = parent_node
        self.is_tracking_waypoint = False
        self.current_waypoint_index = 0
        self.reached_all_waypoints = False
        self.last_gate_pass_time = 0
        self.is_passing_gate = False

        if TASK.sim_mode:
            self.parent_node.get_logger().info("[Task1] Simulation Mode!!")
            # Simülasyon için lokal koordinatlar (x, y, yaw)
            self.task1_waypoints = [
                (1.65, 0.80, 0.0),
                (3.32, 1.40, 0.0),
                (5.46, 1.03, 0.0),
                (7.31, 0.37, 0.0),
            ]
        else:
            self.task1_waypoints = [
                # Sahada hakemlerin vereceği GPS koordinatları (Lat, Lon)
                # (41.12345, 29.12345),
                # (41.12346, 29.12346),
            ]

    def run(self, detected_objects, vehicle_pos):
        vx, vy, _ = vehicle_pos
        if self.current_waypoint_index >= len(self.task1_waypoints):
            self.parent_node.get_logger().info(
                "[Task 1] All waypoints completed! SUCCESS"
            )
            self.parent_node.get_logger().info("[Task 1] Final Control")
            self.nav_controller.full_turn_around_point(vx, vy, 1.0)
            self.nav_controller.stop()
            return TaskResult.SUCCESS

        wp = self.task1_waypoints[self.current_waypoint_index]

        if TASK.sim_mode:
            target_x, target_y, target_yaw = wp[0], wp[1], wp[2]
        else:
            lat, lon = wp[0], wp[1]
            enu_wp = self.nav_controller.gps_to_local_xy(lat, lon, 0.0)
            if not enu_wp:
                self.parent_node.get_logger().warn("[Task 1] GPS Origin bekleniyor...", throttle_duration_sec=2.0)
                return TaskResult.RUNNING
            target_x, target_y = enu_wp[0], enu_wp[1]
            target_yaw = atan2(target_y - vy, target_x - vx)

        dist_to_wp = hypot(vehicle_pos[0] - target_x, vehicle_pos[1] - target_y)

        if dist_to_wp < GATE.goal_success_threshold_meters:
            self.parent_node.get_logger().info(
                f"[Task 1] REACHED WAYPOINT {self.current_waypoint_index + 1}\n\n"
            )
            self.current_waypoint_index += 1
            self.is_tracking_waypoint = False
            self.is_passing_gate = False
            return TaskResult.RUNNING

        pair = self.find_best_buoy_pair(detected_objects, vehicle_pos)
        has_pair = pair[0] is not None or pair[1] is not None

        self.parent_node.get_logger().info(
            f"[Task 1 DEBUG] Has Pair: {has_pair}, Finished WP Index: {self.current_waypoint_index} Next WP Index: {self.current_waypoint_index+1}"
        )

        if has_pair:
            if self.is_tracking_waypoint:
                self.parent_node.get_logger().info(
                    "[Task 1] Gate spotted! Switching to Gate Navigation."
                )
            result = super().run_base(detected_objects, vehicle_pos)

            if result == GateResult.GATE_PASSED:
                self.parent_node.get_logger().info(f"[TASK 1] GATE PASSED!")
                self.is_tracking_waypoint = False
                self.current_waypoint_index += 1
            return TaskResult.RUNNING
        else:
            if not self.is_tracking_waypoint:
                self.nav_controller.go_to_waypoint(target_x, target_y, target_yaw)
                self.is_tracking_waypoint = True
                self.parent_node.get_logger().info(
                    f"[Task 1] Switched to tracking waypoint {self.current_waypoint_index + 1}"
                )
            return TaskResult.RUNNING