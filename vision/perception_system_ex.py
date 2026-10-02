#!/usr/bin/env python3
# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2025.
#
#  File:       P E R C E P T I O N _ S Y S T E M . P Y
#
#  Contents:   
#              Handles TF transformations (Local -> Global).
#              Manages object persistence and redundancy.
#
#  Author:     mustafa5040   25 dec 2025
#
#  Changes:    removed lidar releated code, added SpatialDetectionArray msg support - 12 jun 2026 - mustafa5040
#              removed parse and create function, updated update function, removed redunant code - 13 jun 2026 - mustafa5040
#              mustafa5040 - 27 jun 2026 - Updated to match with new parameters.
#              mustafa5040 - 29 jun 2026 - Added claim mechanism to object matching to prevent a object to match with more than one object at memory
#                                          Updated object cleanup function to dynamicly decay objects based on their health instead of a static coefficent.
#                                          Candidate objects now has object id -1 instead of taking new object id before finalizing
#
# -----------------------------------------------------------------------------

import sys

from pathlib import Path
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

from math import hypot, isnan
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time
from depthai_ros_msgs.msg import SpatialDetectionArray
from utils.objects import *
from utils.parameters import TRACK, YOLO_CLASSES

class PerceptionSystem:
    def __init__(self, tf_buffer, parent_node, target_frame="odom", ):
        self.parent_node = parent_node
        self.detected_objects = []
        self.tf_buffer = tf_buffer
        self.target_frame = target_frame
        self.next_obj_id = 1
    def update(self, msg: SpatialDetectionArray):
        source_frame = msg.header.frame_id

        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.parent_node.get_logger().warn("SpatialDetectionArray has unstamped header, skipping.")
            return
        transform = None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.02) 
            )
        except Exception as e:
            self.parent_node.get_logger().error(f"TF EXCEPTION (optical to odom): {e}") 
            return

        claimed_memory_ids = set()
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]

            cam_x = det.position.x
            cam_y = det.position.y
            cam_z = det.position.z


            if isnan(cam_x) or isnan(cam_y) or isnan(cam_z):
                continue
            if cam_z == 0.0:
                continue
    

            dist_to_cam = hypot(cam_x, cam_y,cam_z)
            if dist_to_cam > TRACK.max_detection_dist:
                continue   # çok uzak, muhtemelen yanlış pozitif — reddet
            
            p_cam = PointStamped()
            p_cam.header.frame_id = source_frame
            p_cam.point.x = float(cam_x)
            p_cam.point.y = float(cam_y)
            p_cam.point.z = float(cam_z)

            profile = YOLO_CLASSES.get(str(hyp.class_id))
            #self.parent_node.get_logger().info(f"hyp class id: {hyp.class_id}")
            if not profile:
                continue
            try:
                p_global = do_transform_point(p_cam, transform)
                global_x = p_global.point.x
                global_y = p_global.point.y
                global_z = p_global.point.z
            except Exception as e:
                self.parent_node.get_logger().info(f"TF Do transfrom point exception (p_cam, transform): {e}")
                continue 
            current_ros_time = self.parent_node.get_clock().now().nanoseconds / 1e9
            candidate_obj = Buoy(
                global_x, global_y, global_z,
                cam_x, cam_y, cam_z,
                profile.width, profile.height, hypot(cam_x, cam_y, cam_z),
                hyp.score, TRACK.initial_health, 
                current_ros_time, -1, False, hyp.class_id, profile.color, profile.role, 
            )

            self._match_and_update(candidate_obj, claimed_memory_ids)

        self.cleanup_redundant_objects()

        ghost_count = sum(1 for b in self.detected_objects if getattr(b, 'is_ghost', False))
        orange_count = sum(1 for b in self.detected_objects if isinstance(b, Buoy) and b.color == Color.ORANGE and not getattr(b, 'is_ghost', False))
        
        self.parent_node.get_logger().info(
            f"[PERCEPTION] OAK-D: {len(msg.detections)} | Memory: {len(self.detected_objects)} Buoy "
            f"(Real orange: {orange_count}, Ghost: {ghost_count})"
        )
    def _match_and_update(self, candidate_obj, claimed_memory_ids):
        #self.parent_node.get_logger().info("match and update")
        self.parent_node.get_logger().info(
        f"[DEBUG-NEW] gx={candidate_obj.global_x:.2f} gy={candidate_obj.global_y:.2f} "
        f"camx={candidate_obj.cam_x:.2f} camy={candidate_obj.cam_y:.2f} camz={candidate_obj.cam_z:.2f}"
    )
        existing_obj = None
        closest_distance = 999.0

        for ex_obj in self.detected_objects:

            if type(ex_obj) != type(candidate_obj):
                continue

            if isinstance(ex_obj, Buoy):
                if ex_obj.role != candidate_obj.role: continue
                if ex_obj.color != candidate_obj.color: continue

            if ex_obj.object_id in claimed_memory_ids:
                continue
            dist_between = hypot(ex_obj.global_x - candidate_obj.global_x, ex_obj.global_y - candidate_obj.global_y)
            depth_diff = abs(ex_obj.cam_z - candidate_obj.cam_z)

            # YENİ KURAL: Mesafe olarak yakın olsa bile derinliği aniden 1.5 metreden fazla zıpladıysa onu reddet (suyun yansımasıdır)
            if dist_between < TRACK.max_match_dist and depth_diff < 1.5:
                if dist_between < closest_distance:
                    closest_distance = dist_between
                    existing_obj = ex_obj
        #matched with existing buoy
        if existing_obj:
            #self.parent_node.get_logger().info("existing objects")
            claimed_memory_ids.add(existing_obj.object_id)
            alpha = candidate_obj.confidence * 0.10

            existing_obj.global_x = (1 - alpha) * existing_obj.global_x + alpha * candidate_obj.global_x
            existing_obj.global_y = (1 - alpha) * existing_obj.global_y + alpha * candidate_obj.global_y
            existing_obj.last_seen = self.parent_node.get_clock().now().nanoseconds / 1e9
            existing_obj.health = min(existing_obj.health + 15.0, TRACK.max_health)
            existing_obj.confidence = (1 - alpha) * existing_obj.confidence + alpha * candidate_obj.confidence
            existing_obj.dist = (1 - alpha) * existing_obj.dist + alpha * candidate_obj.dist
            existing_obj.cam_x = candidate_obj.cam_x
            existing_obj.cam_y = candidate_obj.cam_y
            existing_obj.cam_z = candidate_obj.cam_z
        #new buoy
        else:
            #self.parent_node.get_logger().info("new objects")
            if candidate_obj.confidence > TRACK.minimum_confidence:
                candidate_obj.health = TRACK.initial_health
                candidate_obj.object_id = self.next_obj_id
                self.next_obj_id += 1
                self.detected_objects.append(candidate_obj)
    def cleanup_redundant_objects(self):
        now = self.parent_node.get_clock().now().nanoseconds / 1e9
        to_survive = []

        for obj in self.detected_objects:
            age = now - obj.last_seen
            is_ghost = getattr(obj, 'is_ghost', False)
            
            # SADE VE KESİN TEMİZLİK KURALI
            survival_time = 2.0  # Turuncu dubalar (kapılar) 1.5 saniyede silinir
            
            if isinstance(obj, Buoy) and obj.color == Color.YELLOW:
                survival_time = 2.0  # Sarı dubalar (engeller) 2.5 saniye dayanır

            if age < survival_time or is_ghost:
                to_survive.append(obj)

        self.detected_objects = to_survive
        
    def get_buoys(self, role: BuoyRole, color: Color):
        if not (role and color): return None
        return [obj for obj in self.detected_objects 
                if isinstance(obj, Buoy) and obj.color == color and obj.role == role]
    def get_closest_buoy(self,  role: BuoyRole, color: Color):
        candidates = self.get_buoys(role, color)
        if not candidates:
            return None
        candidates.sort(key=lambda b: b.dist)
        return candidates[0]

    def get_obstacles(self):
        obstacles = []
        for obj in self.detected_objects:
            if isinstance(obj, Buoy):
                if obj.role == BuoyRole.OBSTACLE and obj.color == Color.YELLOW:
                    obstacles.append(obj)
        return obstacles

    def get_all_objects(self):
        return self.detected_objects