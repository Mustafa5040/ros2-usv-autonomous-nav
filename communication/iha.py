from pymavlink import mavutil

# PORT = "/dev/serial/by-id/usb-ArduPilot_fmuv3_330023000B51333530343231-if00"
#PORT = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_2E0017000151313137333837-if00"
# PORT = "/dev/serial/by-id/usb-CubePilot_CubeOrange+_44002B001251333233343437-if00"

PORT = "COM11"

BAUD = 57600


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
    print(f"Bağlandı: {PORT}")

    while True:
        girdi = input("Renk gönder (0=Kırmızı, 1=Yeşil, 2=Mavi, q=Çıkış): ").strip()
        if girdi == "q":
            break
        if girdi in ("0", "1", "2"):
            conn.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"RENK:{girdi}".encode("utf-8")
            )
            print(f"Gönderildi: RENK:{girdi}")
        else:
            print("Geçersiz giriş.")


if __name__ == "__main__":
    main()