import math
import sys
from pathlib import Path

parent_dir = Path(__file__).resolve().parent
sys.path.append(str(parent_dir))

from objects import *

@dataclass(frozen=True)
class PhysicalConfig:
    pear_diameter: float = 0.3
    pear_height: float = 0.5  # TODO: height with flag?
    target_diameter: float = 0.64
    target_height: float = 0.95
    inflation_radius: float = 0.40


@dataclass(frozen=True)
class SystemConfig:
    logic_period: float = 0.05  # 10 Hz
    ghost_timeout_duration = 5.0
    nav2_wait_timeout: float = 2.0
    visual_loss_timeout: float = 5.0
    max_blind_pass_duration: float = 7.0
    task2_max_blind_duration: float = 5.0


@dataclass(frozen=True)
class CameraConfig:
    device_ip: str = "192.168.1.5"
    yolo_pt_path: str = "/root/ida_ws/vision/models/best.pt"
    yolo_engine_path: str = "/root/ida_ws/vision/models/best.engine"
    resolution: tuple = (848, 640)  # multiplies of 32
    model_size: tuple = (640, 640)
    num_classes: int = 80
    num_shaves: int = 6
    sync_nn: bool = True

    lower_depth_mm: int = 300
    upper_depth_mm: int = 12000
    scan_slice_height_px: int = 100
    publish_pcd2: bool = False
    decimation_subsample: int = 16

    enable_yolo: bool = True
    draw_annotations: bool = True

    publish_raw_rgb_image: bool = False
    publish_annotated_image: bool = False

    save_video: bool = True
    annotate_saved_video: bool = True
    put_timestamp: bool = True
    fps: int = 20
    hz: float = 1.0 / 15.0

@dataclass(frozen=True)
class GPSConfig:
    WGS84_AADC = +7.79540464078689228919e0007
    WGS84_BBDCC = +1.48379031586596594555e0002
    WGS84_P1MEE = +9.93305620009858682943e-0001

@dataclass(frozen=True)
class TrackingConfig:
    max_detection_dist: float = 10.0
    max_valid_width: float = 11.0
    top_n_candidates: int = 4
    max_match_dist: float = 1.0
    minimum_confidence: float = 0.70
    oak_iou_threshold: float = 0.6
    initial_health: float = 20.0
    max_health: float = 100.0

    delete_age_threshold: float = 1.5
    decay_age_threshold: float = 2.0

    ghost_search_timeout: float = 3.0
    blind_search_timeout: float = 8.5
    ghost_search_angle_offset: int = 45
    ghost_color_score_rate: float = 0.5

@dataclass(frozen=True)
class GateConfig:
    width_min: float = 2.0
    width_max: float = 11.0
    width_estimate: float = 6.5
    min_clearance: float = 1.5
    max_clearance: float = 3.5

    alignment_threshold: float = math.radians(
        35
    )
    yaw_filter_alpha: float = 0.3
    goal_update_threshold: float = 1.5
    base_task_goal_time_threshold: float = 2.5
    goal_push_default_meters: float = 3.5
    goal_success_threshold_meters: float = 0.5
    goal_success_time_threshold_seconds: float = 1.5


@dataclass(frozen=True)
class TaskConfig:
    distance_multiplier: float = 10.0
    green_weight: float = 3.0
    gate_center_multiplier: float = 50.0

    wall_density: float = 0.05
    wall_back_ext: float = 1.0
    wall_fwd_ext: float = 0.5
    task1_target_gate_count: int = 2

    buoy_mid_yellow_threshold: float = 2.5
    buoy_mid_yellow_shift: float = 2.5

    sim_mode: bool = False


SYS = SystemConfig()
TRACK = TrackingConfig()
GATE = GateConfig()
TASK = TaskConfig()
PHYSICAL = PhysicalConfig()
CAM = CameraConfig()
GPS = GPSConfig()

PEAR_BUOY_DIAMETER = 0.3
PEAR_BUOY_HEIGHT = 0.5  # TODO: height with flag?
TARGET_BUOY_DIAMETER = 0.64
TARGET_BUOY_HEIGHT = 0.95

# TODO: Uzunluk bayrak dahil mi değil mi?
# TODO: class_id doğrudan class ismi ise?
YOLO_CLASSES = {
    "0": BuoyProfile(
        "ORANGE-EDGE-BUOY",
        PEAR_BUOY_DIAMETER,
        PEAR_BUOY_HEIGHT,
        Color.ORANGE,
        BuoyRole.EDGE,
    ),
    "1": BuoyProfile(
        "YELLOW-OBSTACLE-BUOY",
        PEAR_BUOY_DIAMETER,
        PEAR_BUOY_HEIGHT,
        Color.YELLOW,
        BuoyRole.OBSTACLE,
    ),
    "2": BuoyProfile(
        "RED-TARGET-BUOY",
         TARGET_BUOY_DIAMETER,
         TARGET_BUOY_HEIGHT,
         Color.RED,
         BuoyRole.TARGET
         ),
    "3": BuoyProfile(
        "GREEN-TARGET-BUOY",
        TARGET_BUOY_DIAMETER,
        TARGET_BUOY_HEIGHT,
        Color.GREEN,
        BuoyRole.TARGET,
    ),
    "4": BuoyProfile(
        "BLACK-TARGET-BUOY",
        TARGET_BUOY_DIAMETER,
        TARGET_BUOY_HEIGHT,
        Color.BLACK,
        BuoyRole.TARGET,
    ),
}
