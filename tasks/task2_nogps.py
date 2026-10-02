from tasks.base_gate_task import BaseGateTask, GateInternalState, GateResult
from math import hypot, atan2
from utils.objects import Buoy, Color, TaskResult
from utils.helpers import *
from utils.parameters import *
import time

class Task2(BaseGateTask):
    def __init__(self, navigation_controller, tf_buffer, parent_node):
        super().__init__(navigation_controller, tf_buffer, parent_node)
        self.parent_node = parent_node
        self.last_time_buoys_seen = time.time()
        self.gates_passed_count = 0

    def run(self, detected_objects, vehicle_pos):
        current_time = time.time()

        result = self.run_base(detected_objects, vehicle_pos)

        if result == GateResult.GATE_PASSED:
            self.gates_passed_count += 1
            self.nav_controller.cancel_current_nav2_task()
            self.parent_node.get_logger().info(f"[TASK 2] GATE PASSED! ({self.gates_passed_count})")
            try:
                self.nav_controller.nav.clearGlobalCostmap()
                self.nav_controller.nav.clearLocalCostmap()
                self.parent_node.get_logger().info("[Task 2] Costmap temizlendi, yeni kapıya zihin açık gidiliyor.")
            except Exception as e:
                pass
            return TaskResult.RUNNING

        if self.current_state == GateInternalState.BLIND_PASS:
            timer_start = getattr(self, 'blind_drive_start_time', None)
            if timer_start is not None and (current_time - timer_start) > SYS.max_blind_pass_duration:
                self.parent_node.get_logger().error("[Task 2] Blind pass timeout! FAILED.")
                self.nav_controller.stop()
                return TaskResult.FAILED

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

        yellow_buoys = [
            b for b in detected_objects
            if isinstance(b, Buoy) and b.color == Color.YELLOW
        ]

        yellow_buoys = sorted(
            yellow_buoys,
            key=lambda b: hypot(b.global_x - orig_mid_x, b.global_y - orig_mid_y)
        )

        mid_x, mid_y = orig_mid_x, orig_mid_y

        # --- 1. KUSURSUZ İLERİ VEKTÖRÜ HESABI ---
        dx = r_edge_buoy.global_x - l_edge_buoy.global_x
        dy = r_edge_buoy.global_y - l_edge_buoy.global_y
        
        forward_x = -dy
        forward_y = dx
        
        length = hypot(forward_x, forward_y)
        if length != 0:
            forward_x /= length
            forward_y /= length

        # --- 2. YÖN KİLİDİ (HAFIZALI / STICKY VECTOR) ---
        if getattr(self, 'last_forward_dir', None) is not None:
            continuity_dot = (forward_x * self.last_forward_dir[0]) + (forward_y * self.last_forward_dir[1])
            if continuity_dot < 0:
                forward_x = -forward_x
                forward_y = -forward_y
        else:
            vx_to_mid = orig_mid_x - x
            vy_to_mid = orig_mid_y - y
            dot_product = (forward_x * vx_to_mid) + (forward_y * vy_to_mid)
            if dot_product < 0:
                forward_x = -forward_x
                forward_y = -forward_y

        self.last_forward_dir = (forward_x, forward_y)

        # --- 3. TASK 2 ÖZEL: SARI DUBA (ENGEL) KAÇINMA MANTIĞI ---
        for b in yellow_buoys:
            dist_to_mid = hypot(b.global_x - mid_x, b.global_y - mid_y)
            if dist_to_mid < TASK.buoy_mid_yellow_threshold:
                # Sarı duba kapı merkezine çok yakınsa, hedefi ileriye ötele (shift)
                mid_x += forward_x * TASK.buoy_mid_yellow_shift
                mid_y += forward_y * TASK.buoy_mid_yellow_shift

                self.parent_node.get_logger().info(
                    "[Task 2] Yellow obstacle near centerline. Goal shifted forward."
                )
                break

        right_offset_x = forward_y
        right_offset_y = -forward_x


        shift_distance = 0.2  # 60 cm sağa öteleme payı

        # Hedefe hem ileri itme (push) hem de sağa kaydırma (shift) ekleniyor
        raw_goal_x = mid_x + (forward_x * GATE.goal_push_default_meters) + (right_offset_x * shift_distance)
        raw_goal_y = mid_y + (forward_y * GATE.goal_push_default_meters) + (right_offset_y * shift_distance)

        if getattr(self, 'smooth_goal_x', None) is None or getattr(self, 'smooth_goal_y', None) is None:
            self.smooth_goal_x = raw_goal_x
            self.smooth_goal_y = raw_goal_y
        else:
            self.smooth_goal_x = self.smooth_goal_x + (GATE.yaw_filter_alpha * (raw_goal_x - self.smooth_goal_x))
            self.smooth_goal_y = self.smooth_goal_y + (GATE.yaw_filter_alpha * (raw_goal_y - self.smooth_goal_y))

        goal_x = self.smooth_goal_x
        goal_y = self.smooth_goal_y

        # --- 5. YAW HESABI VE YUMUŞATMA ---
        raw_target_yaw = atan2(forward_y, forward_x)

        if getattr(self, 'smooth_yaw', None) is None:
            self.smooth_yaw = raw_target_yaw
        else:
            diff = normalize_angle(raw_target_yaw - self.smooth_yaw)
            self.smooth_yaw = normalize_angle(
                self.smooth_yaw + (GATE.yaw_filter_alpha * diff)
            )

        target_yaw = self.smooth_yaw

        # --- 6. SANAL DUVAR (ENGEL) ÜRETİMİ ---
        walls = generate_virtual_walls((l_edge_buoy, r_edge_buoy), vehicle_pos[:2])
        walls.append((l_edge_buoy.global_x, l_edge_buoy.global_y))
        walls.append((r_edge_buoy.global_x, r_edge_buoy.global_y))

        for b in yellow_buoys:
            walls.append((b.global_x, b.global_y))

        goal = (goal_x, goal_y, target_yaw)
        return goal, walls