# main.py — ESP32-C6 MicroPython BLE Peripheral (Notify/Write)
# Service: 0xFFF0,  Char TX: 0xFFF1 (Notify),  Char RX: 0xFFF2 (Write)
# 功能：hello 握手→回 ctx 與 fw，get fw_info，OTA 三段式，reboot；所有訊息以 '\n' 結尾

import ujson, bluetooth, ubinascii, urandom, time, os
from micropython import const

try:
    import uhashlib as hashlib
except:
    import hashlib

# ====== 版本資訊（由 manifest_gen_portable.py 可選擇覆寫） ======
FW_VER  = "v2025.08.30-ota-final"
FW_SIZE = 0
FW_SHA  = ""

TARGET_PATH = "/main.py"          # OTA 最終目的檔名（可改）
TMP_PATH    = "/ota.tmp"          # 中繼檔

_UUID16 = bluetooth.UUID
_SERVICE_UUID = _UUID16(0xFFF0)
_TX_UUID      = _UUID16(0xFFF1)  # Notify
_RX_UUID      = _UUID16(0xFFF2)  # Write

ble = bluetooth.BLE()
ble.active(True)

tx_char = (_TX_UUID, bluetooth.FLAG_NOTIFY | bluetooth.FLAG_READ,)
rx_char = (_RX_UUID, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE,)
svc = (_SERVICE_UUID, (tx_char, rx_char))
handle_tx = None
handle_rx = None

conn_handle = None
SESSION_CTX = None
_rx_buf = b""

# OTA 狀態
ota_on = False
ota_expect_size = 0
ota_received = 0
ota_expect_sha = ""
ota_seq = 0
ota_sha = None
ota_fp = None  # file object

def rnd_ctx(n=6):
    return ubinascii.hexlify(urandom.urandom(n)).decode()

def fw_dict():
    return {"ver": FW_VER, "size": FW_SIZE, "sha": FW_SHA}

def json_line(obj):
    try:
        s = ujson.dumps(obj) + "\n"
        if conn_handle is None or handle_tx is None: return
        data = s.encode(); mtu = 180
        for i in range(0, len(data), mtu):
            ble.gatts_notify(conn_handle, handle_tx, data[i:i+mtu])
            time.sleep_ms(3)
    except Exception:
        pass

def send_err(code, **kw):
    o = {"err": code}; o.update(kw); json_line(o)

def safe_remove(path):
    try: os.remove(path)
    except: pass

def start_ota(name, size, sha):
    global ota_on, ota_expect_size, ota_received, ota_expect_sha, ota_seq, ota_sha, ota_fp
    if ota_fp:
        try: ota_fp.close()
        except: pass
        ota_fp = None
    safe_remove(TMP_PATH)

    ota_on = True
    ota_expect_size = int(size or 0)
    ota_received = 0
    ota_expect_sha = sha or ""
    ota_seq = 0
    ota_sha = hashlib.sha256()
    ota_fp = open(TMP_PATH, "wb")

def ota_append(seq, b64):
    global ota_received, ota_seq, ota_fp, ota_sha
    if not ota_on or ota_fp is None:
        return "notbegin"
    if seq != ota_seq:
        return "badseq"
    try:
        raw = ubinascii.a2b_base64(b64)
    except Exception:
        return "b64"
    try:
        ota_fp.write(raw)
        ota_fp.flush()
        ota_received += len(raw)
        ota_seq += 1
        ota_sha.update(raw)
        return None
    except Exception:
        return "write"

def finish_ota(sha_final):
    global ota_on, ota_fp, FW_SIZE, FW_SHA
    if not ota_on or ota_fp is None:
        return "notbegin"
    try:
        ota_fp.close()
        ota_fp = None
        calc = ota_sha.digest()
        calc_hex = ubinascii.hexlify(calc).decode()
        if calc_hex != (sha_final or ota_expect_sha):
            safe_remove(TMP_PATH)
            return "sha"
        if ota_expect_size and ota_received != ota_expect_size:
            safe_remove(TMP_PATH)
            return "size"
        # 覆蓋目標
        safe_remove(TARGET_PATH + ".bak")
        try:
            os.rename(TARGET_PATH, TARGET_PATH + ".bak")
        except:
            pass
        os.rename(TMP_PATH, TARGET_PATH)
        # 更新即時回報（下次 hello 會帶最新 size/sha）
        FW_SIZE = ota_received
        FW_SHA  = calc_hex
        ota_on = False
        return None
    except Exception:
        safe_remove(TMP_PATH)
        return "finalize"

def on_write(h):
    global _rx_buf, SESSION_CTX
    try:
        v = ble.gatts_read(h)
        if not v: return
        _rx_buf += v
        while True:
            idx = _rx_buf.find(b"\n")
            if idx < 0: break
            line = _rx_buf[:idx].strip()
            _rx_buf = _rx_buf[idx+1:]
            if not line: continue
            try:
                j = ujson.loads(line)
            except Exception:
                send_err("json"); continue

            op = j.get("op")
            if op == "hello":
                SESSION_CTX = rnd_ctx()
                json_line({"ok":"hello","ctx":SESSION_CTX,"fw":fw_dict()})
                continue

            if j.get("ctx") != SESSION_CTX:
                send_err("noctx"); continue

            if op == "get" and j.get("what") == "fw_info":
                json_line({"ok":"fw_info","fw":fw_dict()})
                continue

            if op == "reboot":
                json_line({"ack":"reboot"})
                try:
                    import machine
                    time.sleep_ms(300)
                    machine.reset()
                except:
                    pass
                continue

            # ===== OTA 三段式 =====
            if op == "ota_begin":
                name = j.get("name") or "main.py"
                size = j.get("size") or 0
                sha  = j.get("sha")  or ""
                try:
                    start_ota(name, size, sha)
                    json_line({"ack":"ota_begin","name":name,"size":size})
                except Exception:
                    send_err("otabegin")
                continue

            if op == "ota_chunk":
                seq = int(j.get("seq") or 0)
                b64 = j.get("data") or ""
                err = ota_append(seq, b64)
                if err: send_err("ota", stage="chunk", seq=seq, why=err)
                else:   json_line({"ack":"ota_chunk","seq":seq})
                continue

            if op == "ota_end":
                sha_final = j.get("sha") or ""
                err = finish_ota(sha_final)
                if err: send_err("ota", stage="end", why=err)
                else:
                    json_line({"ok":"ota_end","sha":sha_final})
                    json_line({"ack":"reboot"})
                    try:
                        import machine
                        time.sleep_ms(400)
                        machine.reset()
                    except:
                        pass
                continue

            send_err("badop")
    except Exception:
        send_err("rx")

def irq(event, data):
    global conn_handle
    if event == bluetooth.IRQ_CENTRAL_CONNECT:
        conn_handle, _, _ = data
    elif event == bluetooth.IRQ_CENTRAL_DISCONNECT:
        conn_handle = None
        advertise()
    elif event == bluetooth.IRQ_GATTS_WRITE:
        _ch, value_handle = data
        if value_handle == handle_rx:
            on_write(value_handle)

def advertise():
    name = "C6-LED"
    adv = bytearray(b"\x02\x01\x06") + bytes((len(name)+1, 0x09)) + name.encode()
    adv += bytes((3, 0x03, 0xF0, 0xFF))
    ble.gap_advertise(None)
    ble.gap_advertise(100, adv_data=adv)

def setup_gatt():
    global handle_tx, handle_rx
    ((h_tx, h_rx),) = ble.gatts_register_services((svc,))
    handle_tx, handle_rx = h_tx, h_rx
    ble.gatts_write(handle_tx, b"")
    ble.gatts_write(handle_rx, b"")

def main():
    setup_gatt()
    ble.irq(irq)
    advertise()
    while True:
        time.sleep_ms(200)

if __name__ == "__main__":
    main()

