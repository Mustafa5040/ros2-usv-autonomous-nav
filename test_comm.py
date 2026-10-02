#!/usr/bin/env python3
import os
import time
import threading
import glob
from pymavlink import mavutil

# --- DİNAMİK ACM PORT TESPİTİ ---
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

RENK_DEBUG = True  # Test aşamasında her şeyi görmek için True yaptık
MISSION_LOCK = threading.Event()

def renk_yaz(color_name: str) -> None:
    """Atomik yazma: önce geçici dosya, sonra os.replace()."""
    tmp = f"{COLOR_FILE_PATH}.tmp"
    with open(tmp, "w") as f:
        f.write(color_name)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, COLOR_FILE_PATH)
    print(f"[TEST] Dosyaya yazıldı -> {COLOR_FILE_PATH}: {color_name}", flush=True)

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

def main():
    print(f"[TEST] Renk dinleyicisi başlatılıyor... Port: {MAVLINK_CONSOLE_PORT}", flush=True)
    son_kod, son_ts = None, 0.0

    while True:
        conn = None
        hb_stop = threading.Event()
        try:
            conn = mavutil.mavlink_connection(
                MAVLINK_CONSOLE_PORT, baud=BAUD_RATE, autoreconnect=True, source_system=200, source_component=191
            )
            threading.Thread(target=heartbeat_dongusu, args=(conn, hb_stop), daemon=True).start()
            print(f"[TEST] Bağlantı kuruldu, STATUSTEXT bekleniyor...", flush=True)

            while True:
                msg = conn.recv_match(type="STATUSTEXT", blocking=True, timeout=5)
                if msg is None:
                    continue

                text = statustext_oku(msg)
                code = renk_kodu_ayikla(text)
                
                if code is None:
                    if RENK_DEBUG:
                        print(f"[TEST][dbg] sys={msg.get_srcSystem()} Gelen Mesaj: {text!r}", flush=True)
                    continue

                color_name = COLORS.get(code)
                if color_name is None:
                    print(f"[TEST] Tanınmayan renk kodu: {code!r} ({text!r})", flush=True)
                    continue

                simdi = time.monotonic()
                if code == son_kod and (simdi - son_ts) < DEDUPE_WINDOW:
                    print(f"[TEST] Tekrarlayan paket engellendi (Dedupe): {code}", flush=True)
                    continue
                son_kod, son_ts = code, simdi

                print(f"[TEST] ★ RENK YAKALANDI: {code} → {color_name} (sys={msg.get_srcSystem()})", flush=True)
                
                try:
                    renk_yaz(color_name)
                except OSError as e:
                    print(f"[TEST] Dosya yazılamadı: {e}", flush=True)
                    continue

                # Onay (ACK) gönderimi
                try:
                    conn.mav.statustext_send(
                        mavutil.mavlink.MAV_SEVERITY_INFO,
                        f"ACK:RENK:{code} {color_name}".encode("utf-8")[:50])
                    print(f"[TEST] ACK mesajı YKİ'ye gönderildi.", flush=True)
                except Exception as e:
                    print(f"[TEST] ACK gönderilemedi: {e}", flush=True)

        except Exception as e:
            print(f"[TEST] Hata / Yeniden bağlanılıyor: {e}", flush=True)
        finally:
            hb_stop.set()
            try:
                if conn:
                    conn.close()
            except Exception:
                pass
            time.sleep(1.0)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[TEST] Test kullanıcı tarafından durduruldu.")