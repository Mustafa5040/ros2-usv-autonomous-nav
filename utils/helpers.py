from utils.objects import *
from math import hypot,sin,cos, pi
from utils.parameters import TASK
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
import subprocess
from pathlib import Path
import logging
import subprocess
def get_unit_vector(point1, point2):
        distance = hypot(point1[0] - point2[0], point1[1] - point2[1])
        dx = point2[0] - point1[0]  # X component of red to green vector
        dy = point2[1] - point1[1]  # Y component of red to green vector

        ux = dx / max(
            0.01, distance
        )  # X component of the unit vector from red to green
        uy = dy / max(
            0.01, distance
        )  # Y component of the unit vector from red to green
        return (ux, uy)

def linear_interpolate_points(start, end, density=0.05):
        points = []
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = hypot(dx, dy)
        steps = max(1, int(length / density))
        for i in range(steps + 1):
            t = i / steps
            points.append((start[0] + t * dx, start[1] + t * dy))
        return points
  #vertical walls
def generate_virtual_walls(buoy_pair: tuple, vehicle_xy):
    l_buoy, r_buoy = buoy_pair
    walls = []
    
    forward_x, forward_y = get_gate_approach_vector(
    (l_buoy.global_x, l_buoy.global_y),
    (r_buoy.global_x, r_buoy.global_y),
    vehicle_xy)

    left_start = l_buoy.global_x - forward_x * TASK.wall_back_ext, l_buoy.global_y - forward_y * TASK.wall_back_ext
    left_end   = l_buoy.global_x + forward_x * TASK.wall_fwd_ext, l_buoy.global_y + forward_y * TASK.wall_fwd_ext
    
    right_start = r_buoy.global_x - forward_x * TASK.wall_back_ext, r_buoy.global_y - forward_y * TASK.wall_back_ext
    right_end   = r_buoy.global_x + forward_x * TASK.wall_fwd_ext, r_buoy.global_y + forward_y * TASK.wall_fwd_ext

    walls.extend(linear_interpolate_points(left_start, left_end))
    walls.extend(linear_interpolate_points(right_start, right_end))
    
    return walls
def get_gate_approach_vector(p1, p2, vehicle_position):
        # to do
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]

        # 90 deg Counter-Clockwise
        perp_x = -dy
        perp_y = dx

        # length of the vector
        mag = hypot(perp_x, perp_y)
        if mag < 0.01:
            return (0.0, 0.0)

        ux = perp_x / mag
        uy = perp_y / mag

        vehicle_x, vehicle_y = vehicle_position
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2

        vehicle_to_gate_x = mid_x - vehicle_x
        vehicle_to_gate_y = mid_y - vehicle_y

        dot_product = (ux * vehicle_to_gate_x) + (uy * vehicle_to_gate_y)

        if dot_product < 0:
            return (-ux, -uy)

        return (ux, uy)

def yaw_to_quaternion(yaw):
        qx = 0
        qy = 0
        qz = sin(yaw / 2)
        qw = cos(yaw / 2)
        return qx, qy, qz, qw
def normalize_angle(angle):
    while angle > pi:
        angle -= 2.0 * pi
    while angle < -pi:
        angle += 2.0 * pi
    return angle
def encode_to_mp4(input_mjpg_path: str, output_mp4_path: str, fps: int = 24) -> bool:
    input_file = Path(input_mjpg_path)
    output_file = Path(output_mp4_path)

    if not input_file.exists():
        print(f"[FFMPEG] Input file not found: {input_mjpg_path}")
        return False

    # Jetson donanımsal H.264 kodlayıcısı (h264_nvmpi veya h264_v4l2m2m)
    # Genellikle Jetson platformlarında h264_v4l2m2m en kararlı çalışan donanım encoder'ıdır.
    cmd = [
        "ffmpeg",
        "-y",
        "-f", "mjpeg",
        "-r", str(fps),
        "-i", str(input_file),
        "-c:v", "libx264",
        "-preset", "ultrafast",  # İşlemciyi yormayan en hızlı preset
        "-crf", "28",            # Dosya boyutunu küçük, hızı yüksek tutan kalite ayarı
        "-pix_fmt", "yuv420p",
        str(output_file)
    ]
    
    try:
        result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
        print(f"[FFMPEG] Donanımsal dönüştürme başarıyla tamamlandı: {output_mp4_path}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"[FFMPEG] Donanımsal dönüştürme başarısız, CPU'ya (libx264) dönülüyor...\n{e.stderr}")
        
        # Eğer donanım desteklemezse güvenli yedek olarak ultrafast CPU moduna düşebilir
        fallback_cmd = [
            "ffmpeg", "-y", "-f", "mjpeg", "-r", str(fps), "-i", str(input_file),
            "-c:v", "libx264", "-preset", "ultrafast", "-crf", "28", "-pix_fmt", "yuv420p",
            str(output_file)
        ]
        try:
            subprocess.run(fallback_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            return True
        except Exception as ex:
            print(f"[FFMPEG] Fallback failed: {ex}")
            return False
    except FileNotFoundError:
        print("[FFMPEG] ffmpeg not found!")
        return False
