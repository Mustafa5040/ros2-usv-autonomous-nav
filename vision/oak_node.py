#!/usr/bin/env python3

# ---------------------------------------------------------------------------
#
#  ERTUĞRUL İDA
#  Copyright (C) KAAN Teknoloji Kulübü, 2026.
#
#  File:       O A K _ N O D E . P Y
#
#  Contents:   Handles stereo camera hardware and creates necessary ros2 topics, core of vision system
#
#  Notes:      Runs on Jetson (OAK connected via PoE). YOLO inference moved
#              on-host (this process) via TensorRT .engine instead of
#              on-device SpatialDetectionNetwork.
#              stereo.setDepthAlign() explicitly aligns depth to the RGB
#              output used for inference.
#
#              Dual RGB output (small = inference, large = recording).
#              IMPORTANT: detection/publish is gated on (small RGB + depth)
#              ONLY, not on the large record frame too - requiring all three
#              independently-clocked queues to align in the same tick was
#              tested and dropped detection rate from ~9.3Hz to ~4.7Hz with
#              much worse jitter (max stall 0.8s), which eats most of the
#              margin against SYS.visual_loss_timeout. Instead, the large
#              frame draws the MOST RECENTLY COMPUTED detections whenever it
#              happens to be ready, decoupled from the detection cycle. This
#              means recorded/annotated video boxes can lag real detections
#              by up to one cycle (~100ms) - acceptable since recording is
#              for review, not real-time navigation.
#
#  Author:     mustafa5040   18 jun 2026
#
#  Changes:    mustafa5040 - 20 jun 2026 - Added automatic mp4 conversion at the end. Added video saving parameter to enable or disable.
#              mustafa5040 - 27 jun 2026 - Updated parameters to use new parameters from utils.
#              mustafa5040 - 18 jul 2026 - Added timestamp support on output video.
#              mustafa5040 - 25 jul 2026 - Moved YOLO inference from on-device
#                                          SpatialDetectionNetwork to on-host
#                                          TensorRT engine, since this node
#                                          already runs on Jetson.
#              mustafa5040 - 25 jul 2026 - Added dual RGB output (small for
#                                          inference, large for recording),
#                                          decoupled from the detection cycle
#                                          to avoid the three-way stream
#                                          alignment penalty.
#              mustafa5040 - 27 jul 2026 - Added strict CAM.enable_yolo controls
#                                          to init, inference, and annotated image
#                                          publishing to save GPU/network bandwidth.
# -----------------------------------------------------------------------------

from pathlib import Path
import sys
parent_dir = Path(__file__).resolve().parent.parent
sys.path.append(str(parent_dir))

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from vision_msgs.msg import ObjectHypothesis
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo
import depthai as dai
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
import numpy as np
from utils.helpers import encode_to_mp4
from datetime import datetime
from cv_bridge import CvBridge
import os
from utils.parameters import CAM, TRACK
import cv2 as cv
import queue
import time
import threading
from ultralytics import YOLO


class OakDNode(Node):
    def __init__(self):
        super().__init__('oak_d_node')
        self.bridge = CvBridge()
        self.sensor_qos_rel = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                durability=DurabilityPolicy.VOLATILE
            )
        self.yolo_pt_path = str((Path(CAM.yolo_pt_path)).resolve().absolute())
        self.yolo_engine_path = str((Path(CAM.yolo_engine_path)).resolve().absolute())
        self.yolo_path = None
        self.yolo = None
        self.recording_start_datetime = datetime.now()
        self.is_first_frame = True
        self._logged_depth_shape = False

        self.last_detections = []

        if CAM.enable_yolo:
            if Path(self.yolo_engine_path).exists():
                self.yolo_path = self.yolo_engine_path
                self.get_logger().info("[OAK NODE] .engine model file found")
            elif Path(self.yolo_pt_path).exists():
                self.yolo_path = self.yolo_pt_path
                self.get_logger().info("[OAK NODE] .pt model file found")

            if self.yolo_path:
                self.yolo = YOLO(self.yolo_path, task='detect')
                self.get_logger().info(f"[OAK NODE] YOLO model loaded from: {self.yolo_path}")

                self.get_logger().info("[OAK NODE] Warming up Yolo engine...")
                dummy_frame = np.zeros((640, 640, 3), dtype=np.uint8)
                self.yolo.predict(dummy_frame, verbose=False)
                self.get_logger().info("[OAK NODE] YOLO engine warmed up.")
            else:
                self.get_logger().warn("[OAK NODE] CAM.enable_yolo is True, but no model file found!")
        else:
            self.get_logger().info("[OAK NODE] YOLO inference disabled via parameters (CAM.enable_yolo = False).")

        if CAM.save_video:
            record_dir = Path("/root/ida_ws/logs/")
            record_dir.mkdir(parents=True, exist_ok=True)
            self.filename = str(record_dir / f"atu_ygm_kaan_ertugrul_record_{datetime.now().strftime('%Y_%m_%d__%H.%M.%S')}")
            self.video_file = open(self.filename + '.mjpg', 'wb')
            self.recorded_frame_count = 0

            self.frame_queue = queue.Queue(maxsize=60)
            self.stop_writer_event = threading.Event()

            self.writer_thread = threading.Thread(target=self._async_video_writer, daemon=True)
            self.writer_thread.start()
            self.pcd_pub = self.create_publisher(PointCloud2, '/oak/rgbd/points', self.sensor_qos_rel)

        self.det_pub = self.create_publisher(SpatialDetectionArray, '/oak/nn/spatial_detections', qos_profile_sensor_data)
        self.rgb_pub = self.create_publisher(Image, '/oak/rgb/image_raw', qos_profile_sensor_data)
        self.rgb_annotated_pub = self.create_publisher(Image, '/oak/rgb/image_annotated', qos_profile_sensor_data)
        self.mark_pub = self.create_publisher(MarkerArray, '/oak/yolo/markers', qos_profile_sensor_data)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/oak/rgb/camera_info', qos_profile_sensor_data)

        self.pipeline = self.create_device()
        self.camera_info_msg = self.build_camera_info()
        self.cam_info_pub.publish(self.camera_info_msg)
        self.get_logger().info("OAK-D Node ready!")
        self.create_timer(CAM.hz, self.timer_cb)

    def _async_video_writer(self):
        if self.is_first_frame:
            self.recording_start_datetime = datetime.now()
            self.is_first_frame = False
        while not self.stop_writer_event.is_set() or not self.frame_queue.empty():
            try:
                frame = self.frame_queue.get(timeout=0.2)
                is_success, buffer = cv.imencode('.jpg', frame, [int(cv.IMWRITE_JPEG_QUALITY), 80])
                if is_success:
                    self.video_file.write(buffer.tobytes())
                    self.recorded_frame_count += 1
                self.frame_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"[ASYNC WRITER] Write Error: {e}")

    def build_camera_info(self):
        calibData = self.pipeline.getDefaultDevice().readCalibration()
        intrinsics = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, CAM.resolution)

        msg = CameraInfo()
        msg.header.frame_id = "oak_rgb_camera_optical_frame"
        msg.width, msg.height = CAM.resolution
        msg.distortion_model = "plumb_bob"

        msg.k = [
            intrinsics[0][0], 0.0, intrinsics[0][2],
            0.0, intrinsics[1][1], intrinsics[1][2],
            0.0, 0.0, 1.0
        ]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [
            intrinsics[0][0], 0.0, intrinsics[0][2], 0.0,
            0.0, intrinsics[1][1], intrinsics[1][2], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        self.fx = intrinsics[0][0]
        self.fy = intrinsics[1][1]
        self.cx = intrinsics[0][2]
        self.cy = intrinsics[1][2]

        self.get_logger().info(f"Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")
        return msg

    def create_device(self):
        self.get_logger().info("[OAK NODE] Creating device...")

        pipeline = dai.Pipeline()

        # 1. RGB - dual output: small for inference, large for recording
        camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A, sensorFps=CAM.fps)
        #rgb_infer_out = camRgb.requestOutput(CAM.resolution, type=dai.ImgFrame.Type.BGR888p, fps=CAM.fps, enableUndistortion=True)
        rgb_out = camRgb.requestOutput(CAM.resolution, type=dai.ImgFrame.Type.BGR888p, fps=CAM.fps, enableUndistortion=True)

        # 2. Stereo
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=CAM.fps)
        left_out = monoLeft.requestOutput(CAM.resolution, type=dai.ImgFrame.Type.GRAY8, fps=CAM.fps)

        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=CAM.fps)
        right_out = monoRight.requestOutput(CAM.resolution, type=dai.ImgFrame.Type.GRAY8, fps=CAM.fps)

        # 3. StereoDepth
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setSubpixel(True)
        stereo.enableDistortionCorrection(True)
        # Frame-based alignment (NOT setDepthAlign(socket)) - setDepthAlign()
        # was silently producing ~1920x1200 depth output (full sensor res)
        # regardless of any requestOutput() size, since it aligns to the BOARD
        # SOCKET, not to a specific frame size. Linking a specific output into
        # inputAlignTo sizes depth to match that frame instead - same pattern
        # as the original RVC2 pipeline's
        # spatialDetectionNetwork.passthrough.link(stereo.inputAlignTo).
        left_out.link(stereo.left)
        right_out.link(stereo.right)
        rgb_out.link(stereo.inputAlignTo)

        self.rgb_q = rgb_out.createOutputQueue(maxSize=1, blocking=False)
        self.depth_q = stereo.depth.createOutputQueue(maxSize=1, blocking=False)

        stereo.initialConfig.setConfidenceThreshold(95)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_3x3)

        pipeline.start()

        return pipeline

    def get_laserscan_from_pcd2(self, depth_frame, stamp):
        depth = depth_frame.getFrame().astype(np.float32) / 1000.0
        h, w = depth.shape

        slice_offset = CAM.scan_slice_height_px // 2
        center_y = h // 2 + 150
        scan_slice = depth[center_y - slice_offset : center_y + slice_offset, :]

        depth_filtered = np.where(scan_slice == 0, np.inf, scan_slice)
        ranges = np.min(depth_filtered, axis=0)

        # Optical angle: + = right side of image
        u = np.arange(w, dtype=np.float32)
        optical_angles = np.arctan2(u - self.cx, self.fx)

        # ---------------------------------------------------------
        # U ŞEKLİNİ DÜZELTEN SİHİRLİ SATIR (RADYAL MESAFE DÖNÜŞÜMÜ)
        # ---------------------------------------------------------
        ranges = ranges / np.cos(optical_angles)

        ranges[ranges < (CAM.lower_depth_mm / 1000.0)] = np.inf
        ranges[ranges > (CAM.upper_depth_mm / 1000.0)] = np.inf

        # oak_camera_frame convention: +angle = left (+Y)
        # → negate, then reverse so angle_min < angle_max
        angles = -optical_angles          # left → +, right → -
        angles = angles[::-1]             # now right → left  (neg → pos)
        ranges = ranges[::-1]             # keep correspondence

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "oak_camera_frame"

        scan.angle_min       = float(angles[0])      # most negative (right)
        scan.angle_max       = float(angles[-1])     # most positive (left)
        scan.angle_increment = float(angles[1] - angles[0])

        scan.range_min = CAM.lower_depth_mm / 1000.0
        scan.range_max = CAM.upper_depth_mm / 1000.0
        scan.ranges    = ranges.tolist()

        return scan

    def run_yolo(self, cv_img):
        results = self.yolo.predict(
            cv_img,
            conf=TRACK.minimum_confidence,
            iou=TRACK.oak_iou_threshold,
            verbose=False
        )[0]

        h, w = cv_img.shape[:2]
        detections = []
        if results.boxes is None:
            return detections

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append({
                "xmin": x1 / w,
                "ymin": y1 / h,
                "xmax": x2 / w,
                "ymax": y2 / h,
                "label": int(box.cls[0].item()),
                "confidence": float(box.conf[0].item()),
            })
        return detections

    def _draw_detections(self, img, detections):
        """Draw normalized-coordinate detections onto img, scaled to img's own size."""
        h, w = img.shape[:2]
        for det in detections:
            x1 = int(det["xmin"] * w)
            y1 = int(det["ymin"] * h)
            x2 = int(det["xmax"] * w)
            y2 = int(det["ymax"] * h)
            label = str(det["label"])
            confidence = f"{det['confidence'] * 100:.1f}%"
            cv.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text_y = y1 - 10 if y1 - 10 > 10 else y1 + 20
            cv.putText(img, f"{label} {confidence}", (x1, text_y), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def timer_cb(self):
        if not rclpy.ok():
            return
        
        stamp = self.get_clock().now().to_msg()

        rgb_frames = self.rgb_q.tryGetAll()
        depth_frames = self.depth_q.tryGetAll()

        rgb_frame = rgb_frames[-1] if rgb_frames else None
        depth_frame = depth_frames[-1] if depth_frames else None

        cv_img = rgb_frame.getCvFrame() if rgb_frame is not None else None

        if cv_img is not None and CAM.publish_raw_rgb_image:
            cv_msg = self.bridge.cv2_to_imgmsg(cv_img, "bgr8")
            cv_msg.header.stamp = stamp
            cv_msg.header.frame_id = "oak_rgb_camera_optical_frame"
            self.rgb_pub.publish(cv_msg)

        depth = None
        depth_h, depth_w = None, None
        if depth_frame is not None:
            depth = depth_frame.getFrame().astype(np.float32) / 1000.0
            depth_h, depth_w = depth.shape[:2]

        if not self._logged_depth_shape and cv_img is not None and depth is not None:
            self.get_logger().info(
                f"[OAK NODE] depth shape: {depth_w}x{depth_h}, inference input: {cv_img.shape[1]}x{cv_img.shape[0]}"
            )
            self._logged_depth_shape = True


        if CAM.enable_yolo and self.yolo is not None and cv_img is not None and depth is not None:

            detections = self.run_yolo(cv_img)

            mark_array = MarkerArray()
            marker_id = 0
            det_array = SpatialDetectionArray()
            det_array.header.stamp = stamp
            det_array.header.frame_id = "oak_rgb_camera_optical_frame"

            for det in detections:
                cx_px = int((det["xmin"] + det["xmax"]) / 2 * depth_w)
                cy_px = int((det["ymin"] + det["ymax"]) / 2 * depth_h)

                box_w = (det["xmax"] - det["xmin"]) * depth_w
                box_h = (det["ymax"] - det["ymin"]) * depth_h
                
                # 1. OPTİMİZASYON: PATCH'İ YUKARI KAYDIR (Sudan/yansımadan kaç)
                cy_px = int(cy_px - (box_h * 0.15)) 
                
                # ROI Kutusunu daralt (%50 yerine %30), sadece dubanın gövdesine odaklan
                patch_w = max(4, int(box_w * 0.3))
                patch_h = max(4, int(box_h * 0.3))

                y1_d = max(0, int(cy_px - patch_h / 2))
                y2_d = min(depth_h, int(cy_px + patch_h / 2))
                x1_d = max(0, int(cx_px - patch_w / 2))
                x2_d = min(depth_w, int(cx_px + patch_w / 2))

                depth_patch = depth[y1_d:y2_d, x1_d:x2_d]
                valid_depths = depth_patch[(depth_patch > 0) & (np.isfinite(depth_patch))]

                # 2. OPTİMİZASYON: MONO DERİNLİK HESABINI HER ZAMAN YAP
                px_width = (det["xmax"] - det["xmin"]) * cv_img.shape[1]
                real_width = 0.3  # Dubanın gerçek çapı (30 cm)
                mono_z = (self.fx * real_width) / px_width if px_width > 0 else 2.0
                mono_z = max(CAM.lower_depth_mm / 1000.0, min(mono_z, CAM.upper_depth_mm / 1000.0))

                # 3. OPTİMİZASYON: GÜVENLİK KONTROLÜ (Sanity Check)
                if len(valid_depths) == 0 or np.median(valid_depths) <= 0.0 or not np.isfinite(np.median(valid_depths)):
                    z = mono_z # Stereo hiç okuyamadıysa Mono'yu kullan
                else:
                    stereo_z = float(np.median(valid_depths))
                    # Eğer Stereo, Mono'dan 1.5 metreden fazla sapmışsa, Stereo yansımaya (suya) kanmıştır!
                    # Mono formüle güven ve onu kullan.
                    if abs(stereo_z - mono_z) > 1.5:
                        z = mono_z
                        # self.parent_node.get_logger().info(f"Yansıma Koruması! Stereo: {stereo_z:.2f}, Mono: {mono_z:.2f}")
                    else:
                        z = stereo_z # İkisi birbirine yakınsa, donanıma (stereo'ya) güven
                
                x = (cx_px - self.cx) * z / self.fx
                y = (cy_px - self.cy) * z / self.fy
                
                x = (cx_px - self.cx) * z / self.fx
                y = (cy_px - self.cy) * z / self.fy

                sd = SpatialDetection()
                hyp = ObjectHypothesis()
                hyp.class_id = str(det["label"])
                hyp.score = det["confidence"]
                sd.results.append(hyp)
                sd.bbox.center.position.x = (det["xmin"] + det["xmax"]) / 2 * cv_img.shape[1]
                sd.bbox.center.position.y = (det["ymin"] + det["ymax"]) / 2 * cv_img.shape[0]
                sd.bbox.size_x = (det["xmax"] - det["xmin"]) * cv_img.shape[1]
                sd.bbox.size_y = (det["ymax"] - det["ymin"]) * cv_img.shape[0]
                sd.position.x = x
                sd.position.y = y
                sd.position.z = z
                det_array.detections.append(sd)

                marker = Marker()
                marker.header.frame_id = "oak_rgb_camera_optical_frame"
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = sd.position.x
                marker.pose.position.y = sd.position.y
                marker.pose.position.z = sd.position.z
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                mark_array.markers.append(marker)

            self.det_pub.publish(det_array)
            self.mark_pub.publish(mark_array)

            self.last_detections = detections
        else:
            self.last_detections = []

        if cv_img is not None:

            """
            if CAM.publish_raw_rgb_image:
                clean_msg = self.bridge.cv2_to_imgmsg(record_img, "bgr8")
                clean_msg.header.stamp = stamp
                clean_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                self.rgb_pub.publish(clean_msg)
            """

            need_annotated = (CAM.enable_yolo and CAM.publish_annotated_image) or (CAM.save_video and CAM.annotate_saved_video)
            annotated_img = None
            if need_annotated:
                annotated_img = cv_img.copy()
                if CAM.enable_yolo and CAM.draw_annotations and self.last_detections:
                    self._draw_detections(annotated_img, self.last_detections)
                if CAM.put_timestamp:
                    current_time = datetime.now().strftime("%d/%m/%Y - %H:%M:%S")
                    cv.putText(annotated_img, current_time, (20, 50), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 3. OPTİMİZASYON: YOLO kapalıysa image_annotated ASLA yayınlanmaz!
            if CAM.enable_yolo and CAM.publish_annotated_image and annotated_img is not None:
                ann_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
                ann_msg.header.stamp = stamp
                ann_msg.header.frame_id = "oak_rgb_camera_optical_frame"
                self.rgb_annotated_pub.publish(ann_msg)

            if CAM.save_video:
                frame_to_save = annotated_img if (CAM.annotate_saved_video and annotated_img is not None) else cv_img
                try:
                    self.frame_queue.put_nowait(frame_to_save.copy())
                except queue.Full:
                    pass

        # Depth to LaserScan
        if depth_frame is not None and depth is not None:

            #scan_msg = self.get_laserscan_from_pcd2(depth_frame, stamp)
            #self.sc_pub.publish(scan_msg)

            if CAM.publish_pcd2:
                h, w = depth.shape
                cx, cy = w / 2, h / 2
                u, v = np.meshgrid(np.arange(w), np.arange(h))
                mask = (depth > 0.1) & (depth < 15.0)
                z = depth[mask]
                x = (u[mask] - cx) * z / self.fx
                y = (v[mask] - cy) * z / self.fy
                points = np.column_stack((x, y, z))[::CAM.decimation_subsample]
                header = Header()
                header.stamp = stamp
                header.frame_id = "oak_rgb_camera_optical_frame"
                self.pcd_pub.publish(pc2.create_cloud_xyz32(header, points.tolist()))

    def destroy_node(self):
        if CAM.save_video:
            self.stop_writer_event.set()
            self.writer_thread.join(timeout=3.0)
            self.video_file.close()

            total_time = (datetime.now() - self.recording_start_datetime).total_seconds()

            if total_time > 0 and self.recorded_frame_count > 0:
                actual_fps = round(self.recorded_frame_count / total_time)
                actual_fps = max(1, actual_fps)
            else:
                actual_fps = CAM.fps

            self.get_logger().info(f"[OAK NODE] {self.recorded_frame_count} frames, {total_time:.1f} seconds recorded. Calculated FPS: {actual_fps}")
            mjpg_path = self.filename + '.mjpg'
            mp4_path = self.filename + '.mp4'
            self.get_logger().info("Saving video please wait...")
            if encode_to_mp4(mjpg_path, mp4_path, actual_fps):
                if os.path.exists(mjpg_path):
                    os.remove(mjpg_path)
                self.get_logger().info(f"[OAK NODE] Saved the output video {mp4_path}")
        super().destroy_node()


def main():
    rclpy.init()
    node = OakDNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()