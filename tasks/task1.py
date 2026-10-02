from pathlib import Path
import sys
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

from tasks.base_gate_task import BaseGateTask, TaskResult
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
        self.last_wp_sent_time = 0.0 # Spam koruması
        
        if TASK.sim_mode:
            self.task1_waypoints = [(40.7236638, 29.8249893), (40.7237441, 29.8248786), (40.7238497, 29.8249284), (40.7240384, 29.8251016)]
        else:
            self.task1_waypoints = [(40.7236638, 29.8249893), (40.7237441, 29.8248786), (40.7238497, 29.8249284), (40.7240384, 29.8251016)]

    def run(self, detected_objects, vehicle_pos):
        vx, vy, _ = vehicle_pos
        current_time = time.time()
        
        if self.current_waypoint_index >= len(self.task1_waypoints):
            self.parent_node.get_logger().info("[Task 1] All waypoints completed! SUCCESS")
            self.nav_controller.stop()
            return TaskResult.SUCCESS

        wp = self.task1_waypoints[self.current_waypoint_index]

        if TASK.sim_mode:
            target_x, target_y = wp[0], wp[1]
        else:
            enu_wp = self.nav_controller.gps_to_local_xy(wp[0], wp[1], 0.0)
            if not enu_wp:
                self.parent_node.get_logger().warn("[Task 1] GPS Origin bekleniyor...", throttle_duration_sec=2.0)
                return TaskResult.RUNNING
            target_x, target_y = enu_wp[0], enu_wp[1]

        dist_to_wp = hypot(vx - target_x, vy - target_y)

        # TORELANS 2.5 METRE (Titremeyi ve sonsuz döngüyü engeller)
        if dist_to_wp < 2.5:
            self.parent_node.get_logger().info(f"\n[Task 1] REACHED WAYPOINT {self.current_waypoint_index + 1}\n")
            self.current_waypoint_index += 1
            self.is_tracking_waypoint = False
            try:
                self.nav_controller.nav.clearLocalCostmap()
            except Exception:
                pass
            return TaskResult.RUNNING

        filtered_objects = self.filter_objects_behind(detected_objects, vehicle_pos)
        points_to_publish = []
        for obj in filtered_objects:
            if isinstance(obj, Buoy):
                points_to_publish.append((obj.global_x, obj.global_y))
        
        self.nav_controller.publish_points_as_obstacles(points_to_publish)

        # SPAM KORUMASI (Nav2'yi kitlemez)
        if not self.is_tracking_waypoint or (self.nav_controller.nav.isTaskComplete() and (current_time - self.last_wp_sent_time) > 2.0):
            target_yaw = atan2(target_y - vy, target_x - vx)
            self.nav_controller.go_to_waypoint(target_x, target_y, target_yaw)
            self.is_tracking_waypoint = True
            self.last_wp_sent_time = current_time
            self.parent_node.get_logger().info(f"[Task 1] Tracking Waypoint {self.current_waypoint_index + 1}...")

        return TaskResult.RUNNING