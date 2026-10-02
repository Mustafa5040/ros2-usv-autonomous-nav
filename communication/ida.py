
from pymavlink import mavutil
import time

#PORT = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_44002B001251333233343437-if00"
#PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
PORT = "COM17"

count = 0

BAUD = 57600

RENKLER = {"0": "KIRMIZI", "1": "YEŞİL", "2": "MAVİ"}

def main():
    conn = mavutil.mavlink_connection(
        PORT,
        baud=BAUD,
        autoreconnect=True,
        source_system=255,
        force_mavlink1=False
    )
    print("YKİ heartbeat bekleniyor...")
    conn.wait_heartbeat()
    print(f"Dinleniyor: {PORT}")

    conn.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"RENK BEKLENİYOR".encode("utf-8")
            )

    while True:
        msg = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=5)
        if msg:
            metin = msg.text.strip()
            if metin.startswith("RENK:"):
                kod = metin.split(":")[1]
                print(f"[İDA] Renk alındı: {kod} → {RENKLER.get(kod, 'BILINMIYOR')} → {count}")



if __name__ == "__main__":
    main()