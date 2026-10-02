#!/usr/bin/env python3
import csv
import math
import os
import time
from datetime import datetime, timezone, timedelta
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from pymavlink import mavutil
import threading

# 1. Kayıt Klasörü ve Dosya Yolları Ayarı
save_dir = Path("/root/ida_ws/logs/")
save_dir.mkdir(parents=True, exist_ok=True)
timestamp_str = datetime.now().strftime('%Y_%m_%d__%H.%M.%S')

CSV_PATH = str(save_dir / f"arac_telemetri_{timestamp_str}.csv")
MAP_DIR = save_dir / f"Lokal_Harita_{timestamp_str}"
MAP_DIR.mkdir(parents=True, exist_ok=True)

BAUD = 57600                     
RATE_HZ = 5                              

HEADER = [
    "timestamp_utc", "lat", "lon", "speed",
    "roll", "pitch", "heading",
    "speed_setpoint", "heading_setpoint",
]

# MAVLink Portunu Dinamik Oku
PORT_FILE = "/tmp/mavlink_port.txt"
if os.path.exists(PORT_FILE):
    with open(PORT_FILE, "r") as f:
        p = f.read().strip()
        if p:
            CONNECTION_STRING = p
else:
    CONNECTION_STRING = "/dev/ttyACM0"


class CostmapListenerNode(Node):
    """ROS2 üzerinden local_costmap verilerini dinleyip PNG olarak kaydeden yardımcı düğüm"""
    def __init__(self):
        super().__init__('mavlink_costmap_listener')
        self.latest_costmap = None
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.costmap_cb,
            10
        )
        # 1 Hz aralıklarla costmap'i diske bas
        self.timer = self.create_timer(1.0, self.save_costmap_callback)

    def costmap_cb(self, msg):
        self.latest_costmap = msg

    def save_costmap_callback(self):
        if self.latest_costmap is not None:
            try:
                width = self.latest_costmap.info.width
                height = self.latest_costmap.info.height
                data = np.array(self.latest_costmap.data, dtype=np.int8)

                grid = data.reshape((height, width))
                
                # Nav2 grid kodları: -1 Bilinmeyen (Gri), 0 Boş (Beyaz), 1-100 Engel (Siyah)
                img = np.full((height, width), 127, dtype=np.uint8)
                img[grid == 0] = 255  
                img[grid >= 1] = 0    

                img = cv2.flip(img, 0) 
                file_time = datetime.now(timezone(timedelta(hours=3)))
                filename = os.path.join(MAP_DIR, f"harita_{file_time}.png")
                cv2.imwrite(filename, img)
            except Exception as e:
                self.get_logger().error(f"Costmap kayıt hatası: {e}")


def ros_spin_thread():
    """ROS2 node'unu arka planda (ayrı bir thread'de) döndürür"""
    rclpy.init()
    node = CostmapListenerNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    # Arka planda Costmap kaydı için ROS2 thread'ini başlat
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    print(f"Bağlanılıyor: {CONNECTION_STRING}")
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat alındı (sys={master.target_system}, comp={master.target_component})")

    interval_us = int(1_000_000 / RATE_HZ)
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
        mavutil.mavlink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT,
    ):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, interval_us, 0, 0, 0, 0, 0,
        )

    file_exists = os.path.isfile(CSV_PATH) and os.path.getsize(CSV_PATH) > 0

    csv_file = open(CSV_PATH, "a", newline="", encoding="utf-8-sig")
    writer = csv.writer(csv_file, delimiter=";")
    if not file_exists:
        writer.writerow(HEADER)
        csv_file.flush()

    # Kilitlenmeleri önlemek için başlangıç varsayılan değerleri (0.0)
    latest = {
        "lat": 0.0, "lon": 0.0, "speed": 0.0,
        "roll": 0.0, "pitch": 0.0, "heading": 0.0,
        "heading_setpoint": 0.0,
        "speed_setpoint": 0.0, 
    }

    print(f"Loglama başladı -> {CSV_PATH} ve {MAP_DIR} (Ctrl+C ile durdurun)")
    last_write = 0.0
    min_interval = 1.0 / RATE_HZ

    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is not None:
                msg_type = msg.get_type()

                if msg_type == "GLOBAL_POSITION_INT":
                    latest["lat"] = msg.lat / 1e7
                    latest["lon"] = msg.lon / 1e7
                    latest["speed"] = math.hypot(msg.vx, msg.vy) / 100.0  # cm/s -> m/s

                elif msg_type == "ATTITUDE":
                    latest["roll"] = math.degrees(msg.roll)
                    latest["pitch"] = math.degrees(msg.pitch)
                    latest["heading"] = math.degrees(msg.yaw) % 360

                elif msg_type == "VFR_HUD":
                    latest["heading"] = float(msg.heading)
                    latest["speed"] = float(msg.groundspeed)

                elif msg_type == "NAV_CONTROLLER_OUTPUT":
                    latest["heading_setpoint"] = float(msg.nav_bearing)
                    if hasattr(msg, 'nav_ground_speed'):
                        latest["speed_setpoint"] = float(msg.nav_ground_speed) / 100.0

            now = time.time()
            speed_sp = latest["speed_setpoint"]

            if (now - last_write) >= min_interval:
                dt = datetime.fromtimestamp(now, tz=timezone.utc)
                writer.writerow([
                    dt.isoformat(timespec="milliseconds"),
                    f"{latest['lat']:.7f}", f"{latest['lon']:.7f}", f"{latest['speed']:.3f}",
                    f"{latest['roll']:.3f}", f"{latest['pitch']:.3f}", f"{latest['heading']:.3f}",
                    f"{speed_sp:.3f}", f"{latest['heading_setpoint']:.3f}",
                ])
                csv_file.flush()
                last_write = now

    except KeyboardInterrupt:
        print("Stopping...")

    finally:
        csv_file.close()
        master.close()
        print("Recording completed.")

if __name__ == "__main__":
    main()