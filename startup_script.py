#!/usr/bin/env python3
import os
import time
import subprocess
import sys
import threading
import signal
import glob
from pymavlink import mavutil

USE_MANUAL_COLOR = False
MANUAL_TARGET_COLOR = "GREEN"
GPIO_NO = "440"
SYSFS_DIR = "/sys/class/gpio"
PIN_DIR = f"{SYSFS_DIR}/PQ.05"

def ros_bash(cmd: str):
    """Container bashrc ile aynı ortamı yükler, sonra komutu çalıştırır."""
    return [
        "bash", "-c",
        "source /opt/ros/humble/install/setup.bash 2>/dev/null || "
        "source /opt/ros/humble/setup.bash 2>/dev/null || true; "
        "source /root/pointcloud_to_laserscan_ws/pointcloud_to_laserscan/install/local_setup.bash 2>/dev/null || true; "
        "source /root/nav2_fix_ws/src/install/setup.bash 2>/dev/null || true; "
        "source /root/ardu_msgs/ros2/ardupilot_msgs/install/local_setup.bash 2>/dev/null || true; "
        "source /root/geo_info/geographic_info/install/local_setup.bash 2>/dev/null || true; "
        "source /root/depthai_ws/depthai-ros/install/local_setup.bash 2>/dev/null || true; "
        "source /root/sensor_msgs_py_ws/common_interfaces/install/local_setup.bash 2>/dev/null || true; "
        "export AMENT_PREFIX_PATH_DISABLE_AUTOCOMPLETE=1; "
        "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; "
        "export FASTRTPS_DEFAULT_PROFILES_FILE=/root/ida_ws/communication/fastdds_lowmem.xml; "
        f"{cmd}"
    ]

def detect_mavlink_port():
    acm_ports = sorted(glob.glob('/dev/ttyACM*'))
    port_count = len(acm_ports)
    if port_count == 0:
        print("[UYARI] Hiçbir ACM portu bulunamadı, varsayılan /dev/ttyACM0 deneniyor...", flush=True)
        return "/dev/ttyACM0"
    elif port_count == 1:
        return acm_ports[0]
    else:
        return acm_ports[-2]

MAVLINK_CONSOLE_PORT = detect_mavlink_port()
BAUD_RATE = 57600
COLOR_FILE_PATH = "/tmp/target_color.txt"
PREFIXES = ("RENK:", "COLOR:")
COLORS = {"0": "RED", "1": "GREEN", "2": "BLACK"}
DEDUPE_WINDOW = 1.0
RENK_DEBUG = False
MISSION_LOCK = threading.Event()

def setup_gpio():
    try:
        if not os.path.exists(PIN_DIR):
            with open(f"{SYSFS_DIR}/export", "w") as f:
                f.write(GPIO_NO)
            time.sleep(0.1)
        with open(f"{PIN_DIR}/direction", "w") as f:
            f.write("in")
    except Exception as e:
        print(f"[UYARI] GPIO Ayarlanamadi: {e}", flush=True)

def read_gpio_value():
    try:
        with open(f"{PIN_DIR}/value", "r") as f:
            return f.read().strip()
    except Exception:
        return "0"

def set_manual_color_if_needed():
    if USE_MANUAL_COLOR:
        with open(COLOR_FILE_PATH, "w") as f:
            f.write(MANUAL_TARGET_COLOR)
    else:
        if not os.path.exists(COLOR_FILE_PATH):
            with open(COLOR_FILE_PATH, "w") as f:
                f.write("UNKNOWN")

def renk_yaz(color_name: str) -> None:
    tmp = f"{COLOR_FILE_PATH}.tmp"
    with open(tmp, "w") as f:
        f.write(color_name)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, COLOR_FILE_PATH)

def statustext_oku(msg) -> str:
    ham = msg.text
    if isinstance(ham, (bytes, bytearray)):
        ham = ham.decode("utf-8", errors="replace")
    return str(ham or "").replace("\x00", "").strip()

def renk_kodu_ayikla(text: str):
    up = text.replace("ı", "i").replace("İ", "I").upper().strip()
    if up.startswith("ACK:"):
        return None
    for pfx in PREFIXES:
        pos = up.find(pfx)
        if pos == -1:
            continue
        kalan = up[pos + len(pfx):].strip()
        if not kalan:
            return None
        return kalan.split()[0].strip(",;.")
    return None

def heartbeat_dongusu(conn, dur_event) -> None:
    while not dur_event.is_set():
        try:
            conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
        except Exception:
            pass
        dur_event.wait(1.0)

def mavlink_color_listener():
    print("[İDA] MAVLINK COLOR LISTENER BAŞLATILDI", flush=True)
    son_kod, son_ts = None, 0.0
    while True:
        conn = None
        hb_stop = threading.Event()
        try:
            while MISSION_LOCK.is_set():
                time.sleep(0.2)
            port = detect_mavlink_port()
            conn = mavutil.mavlink_connection(
                port, baud=BAUD_RATE, autoreconnect=True,
                source_system=200, source_component=191
            )
            threading.Thread(target=heartbeat_dongusu, args=(conn, hb_stop), daemon=True).start()
            print(f"[İDA] Dinleniyor: {port}", flush=True)
            while True:
                if MISSION_LOCK.is_set():
                    raise RuntimeError("mission kilidi aktif, port bırakılıyor")
                msg = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=5)
                if msg is None:
                    continue
                text = statustext_oku(msg)
                code = renk_kodu_ayikla(text)
                if code is None:
                    if RENK_DEBUG:
                        print(f"[İDA][dbg] sys={msg.get_srcSystem()} {text!r}", flush=True)
                    continue
                color_name = COLORS.get(code)
                if color_name is None:
                    print(f"[İDA] Tanınmayan renk kodu: {code!r} ({text!r})", flush=True)
                    continue
                simdi = time.monotonic()
                if code == son_kod and (simdi - son_ts) < DEDUPE_WINDOW:
                    continue
                son_kod, son_ts = code, simdi
                print(f"[İDA] RENK GELDİ: {code} → {color_name}", flush=True)
                try:
                    renk_yaz(color_name)
                except OSError as e:
                    print(f"[İDA] Dosya yazılamadı: {e}", flush=True)
                    continue
                try:
                    conn.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO,
                        f"ACK:RENK:{code} {color_name}".encode("utf-8")[:50]
                    )
                except Exception:
                    pass
        except Exception as e:
            print(f"[İDA] Listener yeniden başlatılıyor: {e}", flush=True)
        finally:
            hb_stop.set()
            try:
                if conn:
                    conn.close()
            except Exception:
                pass
            time.sleep(1.0)

def kill_process_group(process):
    if process and process.poll() is None:
        try:
            os.killpg(os.getpgid(process.pid), signal.SIGINT)
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"[SUPERVISOR] ({process.pid}) SIGKILL!", flush=True)
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            except Exception:
                pass
        except Exception:
            pass

def main():
    print("[STARTUP] SUPERVISOR STARTED... READY FOR RC COMMAND!", flush=True)
    setup_gpio()
    set_manual_color_if_needed()
    color_thread = threading.Thread(target=mavlink_color_listener, daemon=True)
    color_thread.start()

    mission_running = False
    stop_signal_start_time = None
    clock_process = None
    vision_process = None
    nav_process = None
    state_machine_process = None
    logger_process = None
    my_env = os.environ.copy()

    try:
        while True:
            pin_state = read_gpio_value()

            if pin_state == "1":
                if stop_signal_start_time is not None:
                    print("[SUPERVISOR] RC LOW glitch iptal.", flush=True)
                    stop_signal_start_time = None

                if not mission_running:
                    print("\n[SUPERVISOR] >>> RC SWITCH HIGH! STARTING MISSION <<<", flush=True)

                    print("[SUPERVISOR] Clock...", flush=True)
                    clock_process = subprocess.Popen(
                        ros_bash("python3 -u /root/ida_ws/sync_clock.py"),
                        stdout=sys.stdout, stderr=sys.stderr, env=my_env, preexec_fn=os.setsid
                    )
                    time.sleep(3)

                    print("[SUPERVISOR] Logger...", flush=True)
                    logger_process = subprocess.Popen(
                        ros_bash("python3 -u /root/ida_ws/data_logger.py"),
                        stdout=sys.stdout, stderr=sys.stderr, env=my_env, preexec_fn=os.setsid
                    )

                
                    print("[SUPERVISOR] Vision (25s)...", flush=True)
                    vision_process = subprocess.Popen(
                        ros_bash("ros2 launch /root/ida_ws/launch/vision.launch.py"),
                        stdout=sys.stdout, stderr=sys.stderr, env=my_env, preexec_fn=os.setsid
                    )
                    time.sleep(25)

                    print("[SUPERVISOR] Navigation (25s)...", flush=True)
                    nav_process = subprocess.Popen(
                        ros_bash("ros2 launch /root/ida_ws/launch/navigation.launch.py"),
                        stdout=sys.stdout, stderr=sys.stderr, env=my_env, preexec_fn=os.setsid
                    )
                    time.sleep(25)

                    print("[SUPERVISOR] State Machine (10s)...", flush=True)
                    state_machine_process = subprocess.Popen(
                        ros_bash("python3 -u /root/ida_ws/state_machine.py"),
                        stdout=sys.stdout, stderr=sys.stderr, env=my_env, preexec_fn=os.setsid
                    )
                    time.sleep(10)

                    mission_running = True
                    print("[SUPERVISOR] >>> TÜM SİSTEMLER BAŞLATILDI <<<", flush=True)

            elif pin_state == "0" and mission_running:
                if stop_signal_start_time is None:
                    stop_signal_start_time = time.time()
                    print("[SUPERVISOR] RC 0 — 3 sn bekleniyor...", flush=True)
                elif time.time() - stop_signal_start_time > 3.0:
                    print("\n[SUPERVISOR] >>> STOPPING <<<", flush=True)
                    for p in [state_machine_process, nav_process, vision_process, logger_process, clock_process]:
                        kill_process_group(p)
                    mission_running = False
                    stop_signal_start_time = None
                    print("[SUPERVISOR] ALL STOPPED. READY.", flush=True)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[BİLGİ] Kapatılıyor...", flush=True)
        for p in [state_machine_process, nav_process, vision_process, logger_process, clock_process]:
            kill_process_group(p)

if __name__ == "__main__":
    main()