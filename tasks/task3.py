#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       T A S K 3 . P Y
#
# -----------------------------------------------------------------------------
from pathlib import Path
import sys
import os
import time
from math import hypot, atan2

parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

from utils.objects import TaskResult, Buoy, Color, BuoyRole
from utils.parameters import GATE, TASK

class Task3:
    def __init__(self, navigation_controller, tf_buffer, parent_node):
        self.nav_controller = navigation_controller
        self.tf2_buffer = tf_buffer
        self.parent_node = parent_node
        
        self.color_file_path = "/tmp/target_color.txt"
        self.target_color_enum = None
        self.last_target_pos = None
        self.reached_target = False
        
        self.start_time = time.time()
        self.target_hits = 0
        self.last_target_seen_time = 0.0
        self.last_wp_sent_time = 0.0
        self.last_sent_target = None
        
        self.search_step = 0
        self.last_search_turn_time = 0.0

    def read_target_color(self):
        if not os.path.exists(self.color_file_path):
            return None
        try:
            with open(self.color_file_path, "r") as f:
                color_str = f.read().strip().upper()
                
            mapping = {"RED": Color.RED, "GREEN": Color.GREEN, "BLACK": Color.BLACK,
                       "KIRMIZI": Color.RED, "YESIL": Color.GREEN, "SIYAH": Color.BLACK}
            return mapping.get(color_str)
        except Exception:
            return None

    def run(self, detected_objects, vehicle_pos):
        vx, vy, vyaw = vehicle_pos
        now = time.time()

        if not hasattr(self, 'task3_actual_start_time'):
            self.task3_actual_start_time = now
            self.parent_node.get_logger().info("[Task 3] Görev başladı, 90 saniyelik renk bekleme süresi sayılıyor...")
        
        self.target_color_enum = self.read_target_color()
        
        if self.target_color_enum is None:
            if now - self.task3_actual_start_time > 90.0:
                self.parent_node.get_logger().error("[Task 3] 90s renk gelmedi, görev sonlandırılıyor.")
                self.nav_controller.stop()
                return TaskResult.SUCCESS
                
            self.parent_node.get_logger().info(
                "[Task 3] İHA'dan renk kodu bekleniyor... Tekne beklemede.", 
                throttle_duration_sec=2.0
            )
            self.nav_controller.stop()
            return TaskResult.RUNNING
            
        best_target_buoy = None
        closest_dist = float('inf')
        obstacles_to_publish = []
        
        for obj in detected_objects:
            if isinstance(obj, Buoy):
                if obj.role == BuoyRole.TARGET and obj.color == self.target_color_enum:
                    dist = hypot(obj.global_x - vx, obj.global_y - vy)
                    if dist < closest_dist:
                        closest_dist = dist
                        best_target_buoy = obj
                else:
                    obstacles_to_publish.append((obj.global_x, obj.global_y))

        self.nav_controller.publish_points_as_obstacles(obstacles_to_publish)

        if best_target_buoy:
            self.target_hits += 1
            if self.target_hits >= 2: 
                self.last_target_pos = (best_target_buoy.global_x, best_target_buoy.global_y)
                self.last_target_seen_time = now
                self.search_step = 0  
        else:
            self.target_hits = max(0, self.target_hits - 1)

        # Bayat hedef kontrolü (6 sn)
        if self.last_target_pos is not None and (now - self.last_target_seen_time) > 6.0:
            self.parent_node.get_logger().warn("[Task 3] Hedef görüşten çıktı, arama moduna geçiliyor.")
            self.last_target_pos = None
            self.target_hits = 0

        # --- DURUM 1: HEDEF VARSA DOĞRUDAN ANGAJMANA GİT ---
        if self.last_target_pos is not None:
            target_x, target_y = self.last_target_pos
            dist_to_target = hypot(vx - target_x, vy - target_y)

            # Şartname vurma / yaklaşma eşiği (2.0 metre)
            if dist_to_target < 2.0:
                self.parent_node.get_logger().info(f"\n[Task 3] HEDEF ({self.target_color_enum.name}) VURULDU! Mesafe: {dist_to_target:.2f}m. GÖREV BAŞARILI!\n")
                self.nav_controller.stop()
                return TaskResult.SUCCESS

            target_yaw = atan2(target_y - vy, target_x - vx)
            
            send = (self.last_sent_target is None
                    or hypot(target_x - self.last_sent_target[0], target_y - self.last_sent_target[1]) > 0.5
                    or (now - self.last_wp_sent_time) > 2.0)
            
            if send:
                self.nav_controller.go_to_waypoint(target_x, target_y, target_yaw)
                self.last_wp_sent_time = now
                self.last_sent_target = (target_x, target_y)
            
            self.parent_node.get_logger().info(
                f"[Task 3] Hedefe Kilitlenildi ({self.target_color_enum.name})! Kalan Mesafe: {dist_to_target:.2f}m", 
                throttle_duration_sec=1.0
            )

        # --- DURUM 2: HEDEF YOKSA ETRAFI SÜPÜR (SEARCHING) ---
        else:
            self.parent_node.get_logger().warn(
                f"[Task 3] Hedef Renk: {self.target_color_enum.name} - Görüş alanında yok! Alan taranıyor...", 
                throttle_duration_sec=2.0
            )
            
            if self.nav_controller.nav.isTaskComplete() and (now - self.last_search_turn_time > 2.5):
                self.nav_controller.turn_relative_with_nav(60.0)
                self.last_search_turn_time = now

        return TaskResult.RUNNING