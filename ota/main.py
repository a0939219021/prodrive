# main.py - v2025.08.27-r4-iosfix5-minfix
# Minimal fixes:
# - Guard BLE IRQ operations (gatts_read/gap_advertise) to avoid OSError in IRQ
# - Defer advertising after disconnect by 50ms (Timer ONE_SHOT)
# - tx_json(): use valid conn_handle + <=20B chunking + exception guard
#
# NOTE: 仍保留 r4-iosfix5 的原始行為（例如 verify 失敗時可能不回 JSON）

import bluetooth, ujson, os, ubinascii, uhashlib, machine, time
from machine import Timer

ble = bluetooth.BLE(); ble.active(True)

UUID_SVC = bluetooth.UUID("0000fff0-0000-1000-8000-00805f9b34fb")
UUID_RX  = bluetooth.UUID("0000fff1-0000-1000-8000-00805f9b34fb")  # Write / WriteNR
UUID_TX  = bluetooth.UUID("0000fff2-0000-1000-8000-00805f9b34fb")  # Notify

CHAR_RX = (UUID_RX, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE)
CHAR_TX = (UUID_TX, bluetooth.FLAG_NOTIFY)
SERVICE = (UUID_SVC, (CHAR_RX, CHAR_TX))

_handle_rx = None
_handle_tx = None
_conn = None  # 當前連線 handle

fw_tmp = "/update.tmp"
fw_expect_size = 0
fw_expect_sha  = ""
fw_receiving   = False
fw_received    = 0

# 用來延遲重新廣播，避免在 IRQ 當下調用 gap_advertise 觸發 OSError
_adv_timer = Timer(0)

def sha256_file(path, chunk=4096):
    h = uhashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b: break
            h.update(b)
    return ubinascii.hexlify(h.digest()).decode()

def _notify_bytes(b):
    # 安全分段（<=20 bytes）+ 連線檢查 + 例外防護
    global _conn, _handle_tx
    if not _conn or not _handle_tx:
        return
    for i in range(0, len(b), 20):
        chunk = b[i:i+20]
        try:
            ble.gatts_notify(_conn, _handle_tx, chunk)
        except OSError:
            break
        if len(b) > 20:
            time.sleep_ms(2)

def tx_json(obj):
    try:
        _notify_bytes(ujson.dumps(obj).encode())
    except Exception:
        pass

def on_write(v):
    global fw_expect_size, fw_expect_sha, fw_receiving, fw_received
    # 嘗試當作文字控制指令（BEGIN/END）
    is_text = False
    try:
        s = v.decode()
        is_text = True
    except:
        pass

    if is_text and s.startswith("FW:BEGIN"):
        # 例如：FW:BEGIN main.py 15453 86cf7e8e...
        try:
            parts = s.strip().split()
            _, name, size, sha = parts[:4]
            fw_expect_size = int(size)
            fw_expect_sha  = sha
            fw_receiving   = True
            fw_received    = 0
            try:
                os.remove(fw_tmp)
            except:
                pass
            tx_json({"t":"fw","stage":"begin","ok":True,"size":fw_expect_size,"sha":fw_expect_sha})
        except:
            tx_json({"t":"fw","stage":"begin","ok":False,"err":"bad_begin"})
        return

    if is_text and s.startswith("FW:END"):
        if not fw_receiving:
            return
        # 驗證
        got_size = 0
        try:
            got_size = os.stat(fw_tmp)[6]
        except:
            pass
        calc = ""
        ok = False
        try:
            calc = sha256_file(fw_tmp)
            ok = (got_size == fw_expect_size) and (calc == fw_expect_sha)
        except:
            ok = False

        if ok:
            try:
                try: os.remove("main.py.old")
                except: pass
                try: os.rename("main.py","main.py.old")
                except: pass
                os.rename(fw_tmp,"main.py")
            except:
                tx_json({"t":"fw","stage":"done","ok":False,"err":"apply_failed"})
            else:
                tx_json({"t":"fw","stage":"verify","ok":True,"size":got_size,"sha":calc})
                tx_json({"t":"fw","stage":"done","ok":True})
                time.sleep_ms(300)
                machine.reset()
        else:
            # 保留 r4-iosfix5 的原始（未回報詳細錯誤）的行為
            pass

        fw_receiving = False
        return

    # 二進位分塊
    if fw_receiving and isinstance(v, (bytes, bytearray)):
        try:
            with open(fw_tmp, "ab") as f:
                f.write(v)
            fw_received += len(v)
            if (fw_received & 0x3FF) == 0:
                tx_json({"t":"fw","stage":"recv","n":fw_received})
        except:
            fw_receiving = False
            tx_json({"t":"fw","stage":"recv","ok":False,"err":"write_failed"})

def _adv_payload():
    # Flags + 16-bit Service UUID + Name
    return (
        b'\x02\x01\x06' +
        b'\x03\x03\xf0\xff' +
        b'\x07\x09C6-LED'
    )

def _deferred_advertise(_t):
    try:
        ble.gap_advertise(100, adv_data=_adv_payload())
    except OSError:
        # 如果仍失敗就算了，下次再觸發
        pass

def _irq(event, data):
    # 1: connect, 2: disconnect, 3: write
    global _conn
    try:
        if event == 3:  # _IRQ_GATTS_WRITE
            conn_handle, attr_handle = data
            if attr_handle == _handle_rx:
                try:
                    v = ble.gatts_read(_handle_rx)
                except OSError:
                    return
                on_write(v)

        elif event == 1:  # _IRQ_CENTRAL_CONNECT
            conn_handle, addr_type, addr = data
            _conn = conn_handle
            # 這裡不立刻 notify，避免對端尚未啟用通知造成 OSError

        elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
            conn_handle, addr_type, addr = data
            if _conn == conn_handle:
                _conn = None
            # 延遲 50ms 再廣播，避免在 IRQ 當下觸發 OSError
            _adv_timer.init(mode=Timer.ONE_SHOT, period=50, callback=_deferred_advertise)

    except Exception:
        # 防止任何未處理例外把 IRQ 拉掛
        pass

def setup():
    global _handle_rx, _handle_tx
    (( _handle_rx, _handle_tx ),) = ble.gatts_register_services((SERVICE,))
    ble.irq(_irq)
    # 初始廣播
    try:
        ble.gap_advertise(100, adv_data=_adv_payload())
    except OSError:
        pass

setup()
# 初始訊息（若尚未連線/未啟用通知，這條不一定送得出去）
tx_json({"t":"fw","stage":"idle"})
