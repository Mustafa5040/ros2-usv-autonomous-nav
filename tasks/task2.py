from tasks.base_gate_task import BaseGateTask, GateInternalState, GateResult
from math import hypot, atan2, cos, sin
from utils.objects import Buoy, Color, TaskResult
from utils.helpers import *
from utils.parameters import *
import time, os

class Task2(BaseGateTask):
    def __init__(self, navigation_controller, tf_buffer, parent_node):
        super().__init__(navigation_controller, tf_buffer, parent_node)
        self.parent_node = parent_node
        self.last_time_buoys_seen = time.time()
        self.gates_passed_count = 0
        self.emergency_gps_mode = False
        self.emergency_start_time = None
        self.target_gps = (40.7235886, 29.8254987) # GN5

    def run(self, detected_objects, vehicle_pos):
        current_time = time.time()

        if self.target_gps is not None:
            local_target = self.nav_controller.gps_to_local_xy(self.target_gps[0], self.target_gps[1], 0.0)
            
            if local_target is not None:
                target_x, target_y, _ = local_target
                dist_to_final_target = hypot(vehicle_pos[0] - target_x, vehicle_pos[1] - target_y)
                
                # SADECE MESAFEYE BAKARAK BAŞARI (En güvenli yol)
                if dist_to_final_target < 2.5:
                    self.parent_node.get_logger().info(f"\n[TASK 2] GN5 ULAŞILDI! SUCCESS!\n")
                    self.nav_controller.cancel_current_nav2_task()
                    return TaskResult.SUCCESS
                
                # TIMEOUT DURUMU (State Machine'in Task 3'e geçmesi için mecburen SUCCESS dönüyoruz)
                if self.emergency_gps_mode and self.emergency_start_time is not None:
                    if (current_time - self.emergency_start_time) > 40.0:
                        self.parent_node.get_logger().error("\n[TASK 2] GN5 kurtarma zaman aşımı! Task 3'e atlanıyor.\n")
                        self.nav_controller.cancel_current_nav2_task()
                        return TaskResult.SUCCESS


        if self.emergency_gps_mode:
            return TaskResult.RUNNING
        result = self.run_base(detected_objects, vehicle_pos)

        if result == GateResult.GATE_PASSED:
            #self.gates_passed_count += 1
            self.nav_controller.cancel_current_nav2_task()
            self.parent_node.get_logger().info(f"[TASK 2] GATE PASSED! ({self.gates_passed_count})")
            try:
                self.nav_controller.nav.clearGlobalCostmap()
                self.nav_controller.nav.clearLocalCostmap()
                self.parent_node.get_logger().info("[Task 2] Costmap temizlendi, yeni kapıya/hedefe zihin açık gidiliyor.")
            except Exception as e:
                pass
            return TaskResult.RUNNING

        if self.current_state == GateInternalState.BLIND_PASS:
            timer_start = getattr(self, 'blind_phase_start', None) # DÜZELTİLDİ
            if timer_start is not None and (current_time - timer_start) > SYS.max_blind_pass_duration:
                self.parent_node.get_logger().warn("[Task 2] Blind pass timeout! GN5'e sürülüyor...")
                local_target = self.nav_controller.gps_to_local_xy(self.target_gps[0], self.target_gps[1], 0.0)
                if local_target is not None:
                    self.emergency_gps_mode = True
                    self.emergency_start_time = current_time
                    self.nav_controller.go_to_waypoint(local_target[0], local_target[1], 0.0)
                return TaskResult.RUNNING

        return TaskResult.RUNNING

    def calculate_goal(self, l_edge_buoy, r_edge_buoy, detected_objects: list, vehicle_pos):
        x, y, yaw = vehicle_pos

        orig_mid_x = (l_edge_buoy.global_x + r_edge_buoy.global_x) / 2.0
        orig_mid_y = (l_edge_buoy.global_y + r_edge_buoy.global_y) / 2.0

        self.last_pair = (l_edge_buoy, r_edge_buoy)
        self.last_corridor_width = hypot(
            l_edge_buoy.global_x - r_edge_buoy.global_x,
            l_edge_buoy.global_y - r_edge_buoy.global_y
        )

        # 1. İleri vektör
        dx = r_edge_buoy.global_x - l_edge_buoy.global_x
        dy = r_edge_buoy.global_y - l_edge_buoy.global_y
        forward_x = -dy
        forward_y = dx
        length = hypot(forward_x, forward_y)

        if length < 1e-3:
            return None, getattr(self, 'last_published_walls', [])
        if length != 0:
            forward_x /= length
            forward_y /= length

        # 2. Yön kilidi (BaseGateTask ile aynı - U-dönüş koruması)
        if getattr(self, 'last_forward_dir', None) is not None:
            continuity_dot = (forward_x * self.last_forward_dir[0]) + (forward_y * self.last_forward_dir[1])
            if continuity_dot < 0.0:
                forward_x = -forward_x
                forward_y = -forward_y
        else:
            # Tekne pruvasına göre karar ver
            if (forward_x * cos(yaw) + forward_y * sin(yaw)) < 0.0:
                forward_x = -forward_x
                forward_y = -forward_y

        self.last_forward_dir = (forward_x, forward_y)

        # 3. Sarı duba ortaya yakınsa hedefi biraz ileri kaydır
        mid_x, mid_y = orig_mid_x, orig_mid_y
        yellow_buoys = [
            b for b in detected_objects
            if isinstance(b, Buoy) and b.color == Color.YELLOW
        ]
        yellow_buoys = sorted(
            yellow_buoys,
            key=lambda b: hypot(b.global_x - orig_mid_x, b.global_y - orig_mid_y)
        )

        for b in yellow_buoys:
            dist_to_mid = hypot(b.global_x - mid_x, b.global_y - mid_y)
            if dist_to_mid < TASK.buoy_mid_yellow_threshold:
                mid_x += forward_x * TASK.buoy_mid_yellow_shift
                mid_y += forward_y * TASK.buoy_mid_yellow_shift
                self.parent_node.get_logger().info(
                    "[Task 2] Yellow obstacle near centerline. Goal shifted forward."
                )
                break

        # 4. Hedef noktası
        raw_goal_x = mid_x + (forward_x * GATE.goal_push_default_meters)
        raw_goal_y = mid_y + (forward_y * GATE.goal_push_default_meters)

        if getattr(self, 'smooth_goal_x', None) is None or getattr(self, 'smooth_goal_y', None) is None:
            self.smooth_goal_x = raw_goal_x
            self.smooth_goal_y = raw_goal_y
        else:
            self.smooth_goal_x = self.smooth_goal_x + (GATE.yaw_filter_alpha * (raw_goal_x - self.smooth_goal_x))
            self.smooth_goal_y = self.smooth_goal_y + (GATE.yaw_filter_alpha * (raw_goal_y - self.smooth_goal_y))

        goal_x = self.smooth_goal_x
        goal_y = self.smooth_goal_y

        # 5. Yaw
        raw_target_yaw = atan2(forward_y, forward_x)
        if getattr(self, 'smooth_yaw', None) is None:
            self.smooth_yaw = raw_target_yaw
        else:
            diff = normalize_angle(raw_target_yaw - self.smooth_yaw)
            self.smooth_yaw = normalize_angle(self.smooth_yaw + (GATE.yaw_filter_alpha * diff))

        target_yaw = self.smooth_yaw

        # 6. Walls (sarıları tekrar ekleme, zaten run_base engel olarak yayınlıyor)
        walls = generate_virtual_walls((l_edge_buoy, r_edge_buoy), (x, y))
        walls.append((l_edge_buoy.global_x, l_edge_buoy.global_y))
        walls.append((r_edge_buoy.global_x, r_edge_buoy.global_y))

        return (goal_x, goal_y, target_yaw), walls