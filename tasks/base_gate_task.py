#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       B A S E _ G A T E _ T A S K . P Y
#
# -----------------------------------------------------------------------------

from math import hypot, atan2, sin, cos, degrees, pi
from utils.objects import *
from utils.helpers import *
from utils.parameters import GATE, TRACK, SYS, PHYSICAL
import time

class GateInternalState:
    SEARCHING = 0
    NAVIGATING = 1
    FAILED = 2
    BLIND_PASS = 3 

class GateResult:
    RUNNING = 0
    GATE_PASSED = 1
    LOST_VISUAL = 2
    FINISHED = 3
    SUCCESS = 4
    FAILED = 5

class BaseGateTask:
    def __init__(self, navigation_controller, tf_buffer, parent_node):
        self.nav_controller = navigation_controller
        self.current_state = GateInternalState.SEARCHING

        self.tf2_buffer = tf_buffer

        self.gates_passed_count = 0
        self.last_gate_pass_time = time.time()

        self.latest_gate_approach_angle = None
        self.latest_goal = None
        self.last_goal_publish_time = time.time()
        self.last_buoy_seen_time = time.time()

        self.last_forward_dir = None
        self.smooth_goal_x = None
        self.smooth_goal_y = None

        self.ghost_search_start_time = None
        self.blind_search_start_time = None
        self.blind_drive_start_time = time.time()
        self.is_ghost_searching = False
        self.is_blind_searching = False
        self.blind_phase_start = None
        self.is_ghost_buoy_search_done = False
        self.is_blind_search_done = False 
        
        self.parent_node = parent_node

        self.smooth_yaw = None 
        self.search_step = 0

        self.last_pair = None
        self.last_corridor_width = GATE.width_estimate
        self.last_published_walls = []
        self.ghost_locked = False

    def _reset_search_flags_only(self):
        self.is_ghost_searching = False
        self.is_blind_searching = False
        self.is_ghost_buoy_search_done = False
        self.is_blind_search_done = False
        self.ghost_search_start_time = None
        self.blind_search_start_time = None
        self.ghost_locked = False

    def run_base(self, detected_objects: list, vehicle_pos: tuple):
        filtered_objects = detected_objects
        
        points_to_publish = []
        for obj in filtered_objects:
            if isinstance(obj, Buoy) and obj.color == Color.YELLOW and obj.role == BuoyRole.OBSTACLE:
                points_to_publish.append((obj.global_x, obj.global_y))

        if self.last_published_walls:
            points_to_publish.extend(self.last_published_walls)
            
        self.nav_controller.publish_points_as_obstacles(points_to_publish)

        orange_count = sum(1 for obj in filtered_objects if isinstance(obj, Buoy) and obj.color == Color.ORANGE)
        yellow_count = sum(1 for obj in filtered_objects if isinstance(obj, Buoy) and obj.color == Color.YELLOW)

        pair = self.find_best_buoy_pair_ex(filtered_objects, vehicle_pos)
        left_edge_buoy, right_edge_buoy = pair
        
        if left_edge_buoy or right_edge_buoy:
            self.last_buoy_seen_time = time.time()

        state_names = ["(SEARCH)", "(NAV)", "(FAILED)", "(BLIND)"]
        current_state_str = state_names[self.current_state]
        pair_status = "FOUND" if (left_edge_buoy and right_edge_buoy) else ("ONE BUOY" if (left_edge_buoy or right_edge_buoy) else "NO BUOY")
        
        self.parent_node.get_logger().info(
            f"[TASK] State: {current_state_str} | PAIR STATUS:: {pair_status} | Orange Count: {orange_count}"
        )
        
        time_since_loss = time.time() - self.last_buoy_seen_time
        objects_visible = time_since_loss < SYS.visual_loss_timeout

        if objects_visible:
            self.blind_phase_start = None
        elif self.blind_phase_start is None:
            self.blind_phase_start = time.time()

        if (left_edge_buoy or right_edge_buoy) and self.current_state == GateInternalState.BLIND_PASS:
             self.parent_node.get_logger().info("[BaseGate] Buoy re-acquired! Canceling Blind Pass.")
             self.blind_phase_start = None
             self.current_state = GateInternalState.NAVIGATING
             self.nav_controller.cancel_current_nav2_task()
             self.latest_goal = None

        if self.current_state == GateInternalState.SEARCHING:
            self.search(filtered_objects, vehicle_pos)
            return GateResult.RUNNING

        elif self.current_state == GateInternalState.NAVIGATING:
            if not objects_visible:
                if self.latest_goal is not None:
                    dist_to_last_goal = hypot(vehicle_pos[0] - self.latest_goal[0], vehicle_pos[1] - self.latest_goal[1])
                    if dist_to_last_goal < 3.5:
                        self.is_blind_search_done = True

                if not self.is_blind_search_done:
                    self.parent_node.get_logger().info(f"Visual lost for {time_since_loss:.1f}s. SEARCHING...")
                    self.current_state = GateInternalState.SEARCHING
                    return GateResult.LOST_VISUAL
                else:
                    self.current_state = GateInternalState.BLIND_PASS
                    if getattr(self, 'blind_phase_start', None) is None:
                        self.blind_phase_start = time.time()
                    self.blind_drive_start_time = time.time()
                    return GateResult.RUNNING
                
            if not left_edge_buoy and not right_edge_buoy and objects_visible:
                return GateResult.RUNNING

            goal = None
            walls = None
            
            # Case A: Both Visible
            if left_edge_buoy and right_edge_buoy:
                self.ghost_search_start_time = None
                self._reset_search_flags_only() 
                goal, walls = self.calculate_goal(left_edge_buoy, right_edge_buoy, detected_objects, vehicle_pos)
                if goal is None:
                    self.parent_node.get_logger().warn("[BaseGate] Dejenere kapı (NaN tehlikesi), bu frame atlanıyor.")
                    return GateResult.RUNNING
                self.latest_gate_approach_angle = goal[2]
                self.last_published_walls = walls

            # Case B: One Visible (Hayalet Duba)
            elif left_edge_buoy or right_edge_buoy:
                if self.ghost_search_start_time is None:
                    self.ghost_search_start_time = time.time()
                    return GateResult.RUNNING
                
                wait_time = 0.0 if self.is_ghost_buoy_search_done else SYS.ghost_timeout_duration
                
                if (time.time() - self.ghost_search_start_time) > wait_time:
                    if self.last_pair is None or not getattr(self, 'ghost_locked', False):
                        self.parent_node.get_logger().info("GHOST CREATED & LOCKED")
                        ref_buoy = left_edge_buoy if left_edge_buoy else right_edge_buoy
                        ghost = self.create_ghost_edge_buoy(ref_buoy, vehicle_pos)
                        
                        if left_edge_buoy:
                            goal, walls = self.calculate_goal(left_edge_buoy, ghost, detected_objects, vehicle_pos)
                        else: 
                            goal, walls = self.calculate_goal(ghost, right_edge_buoy, detected_objects, vehicle_pos)
                        
                        if goal:
                            self.latest_gate_approach_angle = goal[2]
                        self.last_published_walls = walls
                        self.ghost_locked = True
                    else:
                        if (time.time() - self.ghost_search_start_time) > (SYS.ghost_timeout_duration + 8.0):
                            self.parent_node.get_logger().warn("[BaseGate] Ghost lock timeout.")
                            self.ghost_locked = False
                            self.ghost_search_start_time = None
            else:
                self.current_state = GateInternalState.SEARCHING
                return GateResult.RUNNING

            if goal or self.latest_goal is not None:
                goal_x, goal_y, goal_yaw = goal if goal else self.latest_goal
                
                if goal:
                    if self.last_pair is not None:
                        l_buoy, r_buoy = self.last_pair
                        mid_x = (l_buoy.global_x + r_buoy.global_x) / 2.0
                        mid_y = (l_buoy.global_y + r_buoy.global_y) / 2.0
                        dist_to_gate_line = hypot(vehicle_pos[0] - mid_x, vehicle_pos[1] - mid_y)
    
                        if dist_to_gate_line < 3.0 and self.latest_goal is not None:
                            pass
                        elif self.should_update_goal(goal_x, goal_y):
                            self.latest_goal = (goal_x, goal_y, goal_yaw)
                            self.last_goal_publish_time = time.time()
                            self.nav_controller.go_to_waypoint(goal_x, goal_y, goal_yaw)
                            self.parent_node.get_logger().info(
                                f"[GOAL] New goal sent -> X: {goal_x:.2f}, Y: {goal_y:.2f}"
                            )

                dist_to_goal = hypot(vehicle_pos[0] - goal_x, vehicle_pos[1] - goal_y)
                if dist_to_goal < GATE.goal_success_threshold_meters and (time.time() - self.last_gate_pass_time > 5.0):
                    self.gates_passed_count += 1
                    self.parent_node.get_logger().info(f"\nGATE PASSED ({self.gates_passed_count})\n")
                    self.last_gate_pass_time = time.time()
                    self.nav_controller.clear_virtual_obstacles()
                    self.last_published_walls = [] 
                    self._reset_search_flags_only()
                    self._reset_goal_filter()
                    return GateResult.GATE_PASSED

            return GateResult.RUNNING
        
        elif self.current_state == GateInternalState.BLIND_PASS:
            if self.nav_controller.nav.isTaskComplete():
                safe_yaw = self.latest_gate_approach_angle if self.latest_gate_approach_angle is not None else vehicle_pos[2]
                target_x = vehicle_pos[0] + (GATE.goal_push_default_meters * cos(safe_yaw))
                target_y = vehicle_pos[1] + (GATE.goal_push_default_meters * sin(safe_yaw))
                
                self.parent_node.get_logger().info(f"[BaseGate] BLIND PASS ACTIVATED! Driving safely along last known approach vector.")
                self.nav_controller.go_to_waypoint(target_x, target_y, safe_yaw)
                self.current_state = GateInternalState.NAVIGATING
            return GateResult.RUNNING
            
        return GateResult.RUNNING

    def search(self, detected_objects: list, vehicle_pos: tuple):
        pair = self.find_best_buoy_pair_ex(detected_objects, vehicle_pos)
        r, g = pair
        reference_yaw = self.latest_gate_approach_angle if self.latest_gate_approach_angle is not None else vehicle_pos[2]

        if r and g:
            self.search_step = 0
            self._reset_search_flags_only()
            if not self.nav_controller.nav.isTaskComplete():
                self.nav_controller.cancel_current_nav2_task()
                self.latest_goal = None
            self.current_state = GateInternalState.NAVIGATING
            return

        elif r or g:
            if self.is_blind_searching:
                self._reset_blind_search_for_search()

            if not self.is_ghost_searching and not self.is_ghost_buoy_search_done:
                self.is_ghost_searching = True
                self.ghost_search_start_time = time.time()
                self.nav_controller.go_to_waypoint(vehicle_pos[0], vehicle_pos[1], reference_yaw)
                return

            if not self.ghost_search_start_time:
                self.ghost_search_start_time = time.time()

            if (time.time() - self.ghost_search_start_time ) > TRACK.ghost_search_timeout:
                self.is_ghost_buoy_search_done = True
                self.is_ghost_searching = False
                self.current_state = GateInternalState.NAVIGATING
            return

        else:
            if not self.is_blind_searching:
                self.is_blind_searching = True
                self.blind_search_start_time = time.time()
                self.search_step = 1

            if self.nav_controller.nav.isTaskComplete():
                if self.search_step == 1:
                    self.nav_controller.turn_relative_with_nav(45.0)
                    self.search_step = 2
                elif self.search_step == 2:
                    self.nav_controller.turn_relative_with_nav(-90.0)
                    self.search_step = 3
                elif self.search_step == 3:
                    self.nav_controller.turn_relative_with_nav(45.0) 
                    self.search_step = 4
                    
            if not self.blind_search_start_time:
                self.blind_search_start_time = time.time()
            if (time.time() - self.blind_search_start_time) > TRACK.blind_search_timeout:
                self._reset_search_flags_only()
                self.is_blind_search_done = True
                self.current_state = GateInternalState.NAVIGATING

            return

    def should_update_goal(self, new_x, new_y):
        if self.latest_goal is None:
            return True
        
        if (time.time() - self.last_goal_publish_time) < GATE.base_task_goal_time_threshold:
            return False

        old_x, old_y, _ = self.latest_goal
        dist = hypot(new_x - old_x, new_y - old_y)
        return dist > GATE.goal_update_threshold

    def find_best_buoy_pair_ex(self, detected_objects: list, vehicle_pos: tuple):
        vx, vy, v_yaw = vehicle_pos

        edge_buoys = [
            obj for obj in detected_objects
            if isinstance(obj, Buoy)
            and obj.color == Color.ORANGE
            and obj.role == BuoyRole.EDGE
        ]

        if len(edge_buoys) == 0:
            return (None, None)

        if len(edge_buoys) == 1:
            b = edge_buoys[0]
            ref = self.latest_gate_approach_angle if self.latest_gate_approach_angle is not None else v_yaw
            rel = normalize_angle(atan2(b.global_y - vy, b.global_x - vx) - ref)
            return (b, None) if rel > 0.0 else (None, b)

        best_pair = (None, None)
        best_score = float('inf')

        ref_yaw = (
            self.latest_gate_approach_angle
            if self.latest_gate_approach_angle is not None
            else v_yaw
        )

        for left_buoy in edge_buoys:
            for right_buoy in edge_buoys:
                if left_buoy is right_buoy:
                    continue

                real_dist_left = hypot(left_buoy.global_x - vx, left_buoy.global_y - vy)
                real_dist_right = hypot(right_buoy.global_x - vx, right_buoy.global_y - vy)
                
                if real_dist_left > TRACK.max_detection_dist or real_dist_right > TRACK.max_detection_dist:
                    continue

                dist_between = hypot(
                    left_buoy.global_x - right_buoy.global_x,
                    left_buoy.global_y - right_buoy.global_y
                )

                mid_x = (left_buoy.global_x + right_buoy.global_x) / 2.0
                mid_y = (left_buoy.global_y + right_buoy.global_y) / 2.0
                angle_to_mid = atan2(mid_y - vy, mid_x - vx)
                angle_diff = abs(normalize_angle(angle_to_mid - ref_yaw))

                dynamic_depth_threshold = 2.0 + (dist_between * sin(angle_diff))
                max_allowable_diff = GATE.width_max * 0.85
                dynamic_depth_threshold = min(dynamic_depth_threshold, max_allowable_diff)
                
                depth_diff = abs(real_dist_left - real_dist_right)
                
                if depth_diff > dynamic_depth_threshold:
                    continue
                
                if dist_between < GATE.width_min:
                    continue

                if dist_between > GATE.width_max:
                    continue

                if angle_diff > GATE.alignment_threshold:
                    continue

                avg_distance = (real_dist_left + real_dist_right) / 2.0
                score = (avg_distance * 2.0) + (depth_diff * 4.0)
                
                if score < best_score:
                    best_score = score
                    best_pair = (left_buoy, right_buoy)
                    
        if best_pair[0] is not None and best_pair[1] is not None:
            b1, b2 = best_pair
            angle1 = atan2(b1.global_y - vy, b1.global_x - vx)
            angle2 = atan2(b2.global_y - vy, b2.global_x - vx)

            rel1 = normalize_angle(angle1 - ref_yaw)
            rel2 = normalize_angle(angle2 - ref_yaw)

            if rel1 > rel2:
                best_pair = (b1, b2)  
            else:
                best_pair = (b2, b1)  

        return best_pair

    def calculate_goal(self, l_edge_buoy, r_edge_buoy, detected_objects: list, vehicle_pos):
        x, y, yaw = vehicle_pos
        mid_x = (l_edge_buoy.global_x + r_edge_buoy.global_x) / 2.0
        mid_y = (l_edge_buoy.global_y + r_edge_buoy.global_y) / 2.0

        self.last_pair = (l_edge_buoy, r_edge_buoy)
        self.last_corridor_width = hypot(
            l_edge_buoy.global_x - r_edge_buoy.global_x,
            l_edge_buoy.global_y - r_edge_buoy.global_y
        )

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

        # U-DÖNÜŞÜ (180 DERECE) HATASI GİDERİLDİ! 0.3'ten 0.0'a çekildi, PRUVA önceliklendirildi.
        if getattr(self, 'last_forward_dir', None) is not None:
            continuity_dot = (forward_x * self.last_forward_dir[0]) + (forward_y * self.last_forward_dir[1])
            if continuity_dot < 0.0:
                forward_x = -forward_x
                forward_y = -forward_y
        else:
            # TEKNE PRUVASINA GÖRE hesapla (orta noktaya göre değil!)
            if (forward_x * cos(yaw) + forward_y * sin(yaw)) < 0.0:
                forward_x = -forward_x
                forward_y = -forward_y

        self.last_forward_dir = (forward_x, forward_y)
        right_offset_x = -forward_y 
        right_offset_y = forward_x

        corrected_mid_x = mid_x + (right_offset_x * 0.6) 
        corrected_mid_y = mid_y + (right_offset_y * 0.6)

        raw_goal_x = corrected_mid_x + (forward_x * GATE.goal_push_default_meters)
        raw_goal_y = corrected_mid_y + (forward_y * GATE.goal_push_default_meters)

        if self.smooth_goal_x is None or self.smooth_goal_y is None:
            self.smooth_goal_x = raw_goal_x
            self.smooth_goal_y = raw_goal_y
        else:
            self.smooth_goal_x = self.smooth_goal_x + (GATE.yaw_filter_alpha * (raw_goal_x - self.smooth_goal_x))
            self.smooth_goal_y = self.smooth_goal_y + (GATE.yaw_filter_alpha * (raw_goal_y - self.smooth_goal_y))

        goal_x = self.smooth_goal_x
        goal_y = self.smooth_goal_y

        raw_target_yaw = atan2(forward_y, forward_x)

        if self.smooth_yaw is None:
            self.smooth_yaw = raw_target_yaw
        else:
            diff = normalize_angle(raw_target_yaw - self.smooth_yaw)
            self.smooth_yaw = normalize_angle(
                self.smooth_yaw + (GATE.yaw_filter_alpha * diff)
            )

        target_yaw = self.smooth_yaw

        walls = generate_virtual_walls((l_edge_buoy, r_edge_buoy), (x, y))
        walls.append((l_edge_buoy.global_x, l_edge_buoy.global_y))
        walls.append((r_edge_buoy.global_x, r_edge_buoy.global_y))
        
        return (goal_x, goal_y, target_yaw), walls
        
    def _reset_ghost_search_for_search(self):
        self.is_ghost_searching = False
        self.ghost_search_start_time = None
        self.is_ghost_buoy_search_done = True

    def _reset_blind_search_for_search(self):
        self.is_blind_searching = False
        self.blind_search_start_time = None
        self.is_blind_search_done = True
        self.search_step = 0 # Adımı sıfırla ki takılı kalmasın

    def reset_searching_for_search(self):
        self._reset_blind_search_for_search()
        self._reset_ghost_search_for_search()

    def reset_searching_for_new_gate(self):
        self.is_ghost_searching = False
        self.is_blind_searching = False
        self.is_ghost_buoy_search_done = False
        self.is_blind_search_done = False
        self.ghost_search_start_time = None
        self.blind_search_start_time = None

    def _reset_all_search_states(self):
        self.is_ghost_searching = False
        self.is_blind_searching = False
        self.is_ghost_buoy_search_done = False
        self.is_blind_search_done = False
        self.ghost_search_start_time = None
        self.blind_search_start_time = None
        self.last_forward_dir = None
        self.smooth_goal_x = None
        self.smooth_goal_y = None
        self.smooth_yaw = None # Yaw hafızasını da sıfırla
        self.ghost_locked = False

    def create_ghost_edge_buoy(self, reference_buoy: Buoy, vehicle_pos: tuple):
        vx, vy, v_yaw = vehicle_pos
        approach_angle = self.latest_gate_approach_angle if self.latest_gate_approach_angle is not None else v_yaw

        gate_axis_x = sin(approach_angle)
        gate_axis_y = -cos(approach_angle)

        # GÜVENLİK: Genişliği limitle
        width = self.last_corridor_width
        if not (GATE.width_min <= width <= GATE.width_max):
            width = GATE.width_estimate

        direction = 1
        # GHOST YÖN TESPİTİ DÜZELTİLDİ!
        if self.last_pair is not None:
            old_a, old_b = self.last_pair
            dist_to_a = hypot(reference_buoy.global_x - old_a.global_x, reference_buoy.global_y - old_a.global_y)
            dist_to_b = hypot(reference_buoy.global_x - old_b.global_x, reference_buoy.global_y - old_b.global_y)
            direction = 1 if dist_to_a < dist_to_b else -1
        else:
            rel = normalize_angle(atan2(reference_buoy.global_y - vy, reference_buoy.global_x - vx) - approach_angle)
            direction = 1 if rel > 0.0 else -1

        ghost_x = reference_buoy.global_x + (gate_axis_x * width * direction)
        ghost_y = reference_buoy.global_y + (gate_axis_y * width * direction)
        ghost_z = reference_buoy.global_z

        distance = hypot(vx - ghost_x, vy - ghost_y)
        ghost_health = reference_buoy.health * 0.5
        ghost_confidence = 0.5
        return Buoy(ghost_x, ghost_y, ghost_z,
            reference_buoy.cam_x,
            reference_buoy.cam_y,
            reference_buoy.cam_z,
            PHYSICAL.pear_diameter,
            PHYSICAL.pear_height,
            distance,
            ghost_confidence,
            ghost_health,
            time.time(),
            -1,
            True,
            0,
            Color.ORANGE,
            BuoyRole.EDGE
        )
    def _reset_goal_filter(self):
        self.smooth_goal_x = None
        self.smooth_goal_y = None
        self.smooth_yaw = None
    def filter_objects_behind(self, detected_objects, vehicle_pos):
        vx, vy, vyaw = vehicle_pos
        filtered = []

        reference_yaw = self.latest_gate_approach_angle if self.latest_gate_approach_angle is not None else vyaw 

        for obj in detected_objects:
            dx = obj.global_x - vx
            dy = obj.global_y - vy
            angle_to_obj = atan2(dy, dx)
            diff = normalize_angle(angle_to_obj - reference_yaw)

            if abs(diff) < (pi / 2.0 + 0.3):
                filtered.append(obj)
                
        return filtered