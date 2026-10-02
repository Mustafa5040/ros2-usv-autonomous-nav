from pymavlink import mavutil
import time

PORTİHA = "COM7"

PORTİDA = "COM5"

BAUD = 57600

RENKLER = {"0": "KIRMIZI", "1": "YEŞİL", "2": "MAVİ"}

def main():

    connIHA = mavutil.mavlink_connection(PORTİHA, baud=BAUD)
    connIDA = mavutil.mavlink_connection(PORTİDA, baud=BAUD)

    print("Portlar açıldı...")

    print("İHA heartbeat bekleniyor...")
    connIHA.wait_heartbeat()
    print("İHA bağlı")

    print("İDA heartbeat bekleniyor...")
    connIDA.wait_heartbeat()
    print("İDA bağlı")

    connIHA.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"RENK BEKLENİYOR".encode("utf-8")
            )

    while True:
        msg = connIHA.recv_match(type="STATUSTEXT", blocking=True)
        if msg:
            metin = msg.text.strip()
            if metin.startswith("RENK:"):
                kod = metin.split(":")[1]
                print(f"[YKİ] Renk alındı: {kod} → {RENKLER.get(kod, 'Bilinmiyor')}")
            
                connIDA.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                f"RENK:{kod}".encode("utf-8")
                )
                print(f"[YKİ] Gönderildi: RENK:{kod} → {RENKLER.get(kod, 'Bilinmiyor')}")


if __name__ == "__main__":
    main()