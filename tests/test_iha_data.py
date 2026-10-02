#!/usr/bin/env python3
import os
import time
from pymavlink import mavutil

# ida_start.sh'ın oluşturduğu port dosyasından MAVLink portunu dinamik al
PORT_FILE = "/tmp/mavlink_port.txt"
if os.path.exists(PORT_FILE):
    with open(PORT_FILE, "r") as f:
        p = f.read().strip()
        if p:
            CONNECTION_STRING = p
else:
    CONNECTION_STRING = "/dev/ttyACM0"

BAUD = 57600

print(f"[IHA TEST] Port açılıyor: {CONNECTION_STRING} (Baud: {BAUD})")

try:
    master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD, autoreconnect=True)
    print("[IHA TEST] Bağlantı nesnesi oluşturuldu. Heartbeat bekleniyor...")
    
    master.wait_heartbeat()
    print(f"[IHA TEST] ✓ Heartbeat alındı! System ID: {master.target_system}, Component ID: {master.target_component}")
    print("[IHA TEST] Veriler dinleniyor... Çıkmak için CTRL+C tuşlarına basın.\n")

    while True:
        # Bloklayarak mesajları dinle (1 saniye zaman aşımı)
        msg = master.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            print("[IHA TEST] Veri akışı bekleniyor (Timeout)... Gelen paket yok.")
            continue

        msg_type = msg.get_type()

        # 1. Şartnamede geçen İHA renk veya durum mesajları (STATUSTEXT)
        if msg_type == "STATUSTEXT":
            text = msg.text.strip()
            print(f"[STATUSTEXT ALINDI] -> {text}")
            if text.startswith("RENK:"):
                print(f"  *** HEDEF RENK KODU TESPİT EDİLDİ: {text.split(':')[1]} ***")

        # 2. GPS / Konum Verisi (İHA veya İDA hareket halindeyken)
        elif msg_type == "GLOBAL_POSITION_INT":
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f"[GPS] Lat: {lat:.7f}, Lon: {lon:.7f}, İrtifa: {alt:.2f}m")

        # 3. Batarya / Güç Durumu
        elif msg_type == "SYS_STATUS":
            voltage = msg.voltage_battery / 1000.0
            current = msg.current_battery / 100.0
            remaining = msg.battery_remaining
            print(f"[BATARYA] Voltaj: {voltage:.2f}V, Akım: {current:.2f}A, Kalan: %{remaining}")

except KeyboardInterrupt:
    print("\n[IHA TEST] Test kullanıcı tarafından sonlandırıldı.")
except Exception as e:
    print(f"\n[IHA TEST HATA] Bir hata oluştu: {e}")