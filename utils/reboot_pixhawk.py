#!/usr/bin/env python3
import sys
import time
from pymavlink import mavutil

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    baud = 57600
    print(f"[REBOOT] Connecting to MAVLink port: {port}")
    
    try:
        master = mavutil.mavlink_connection(port, baud=baud)
        master.wait_heartbeat(timeout=5)
        print(f"[REBOOT] Heartbeat received (SYS ID: {master.target_system}). Sening Reboot to pixhawk...")
        
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)
        master.close()
        print("[REBOOT] Sent Reset. Port Closed...")
        
    except Exception as e:
        print(f"[REBOOT] Reset error: {e}")

if __name__ == "__main__":
    main()