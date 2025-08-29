# main.py - v2025.08.27-r4-iosfix5 (as-is)
# Known issues:
# - gatts_notify(0, ...) 連線代號寫死，斷線/未啟用通知時可能丟 OSError(-128)
# - 驗證失敗時未必會送出明確 JSON，前端可能卡在 "verifying"
# - 未做 notify 分段（依 MTU <= 20/182），長訊息可能被丟棄

import bluetooth, ujson, os, ubinascii, uhashlib, machine, time

ble = bluetooth.BLE(); ble.active(True)

UUID_SVC = bluetooth.UUID("0000fff0-0000-1000-8000-00805f9b34fb")
UUID_RX  = bluetooth.UUID("0000fff1-0000-1000-8000-00805f9b34fb")  # Write / WriteNR
UUID_TX  = bluetooth.UUID("0000fff2-0000-1000-8000-00805f9b34fb")  # Notify

CHAR_RX = (UUID_RX, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE)
CHAR_TX = (UUID_TX, bluetooth.FLAG_NOTIFY)
SERVICE = (UUID_SVC, (CHAR_RX, CHAR_TX))

_handle_rx = None
_handle_tx = None

fw_tmp = "/update.tmp"
fw_expect_size = 0
fw_expect_sha  = ""
fw_receiving   = False
fw_received    = 0

def sha256_file(path, chunk=4096):
    h = uhashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b: break
            h.update(b)
    return ubinascii.hexlify(h.digest()).decode()

def tx_json(obj):
    # NOTE: 未分段，可能超過 MTU 造成例外，中斷流程
    try:
        ble.gatts_notify(0, _handle_tx, ujson.dumps(obj))
    except Exception as e:
        # r4-iosfix5 原樣：忽略例外
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

        # r4-iosfix5：只有成功時才送 verify OK；失敗時有機會不送明確結果（前端會卡住）
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
            # 原樣：可能只紀錄，不一定回報 JSON（這就是桌面卡 "verifying" 的主因之一）
            # tx_json({"t":"fw","stage":"verify","ok":False,"got_size":got_size,"calc":calc,"expect_size":fw_expect_size,"expect":fw_expect_sha})
            pass

        fw_receiving = False
        return

    # 二進位分塊
    if fw_receiving and isinstance(v, (bytes, bytearray)):
        try:
            with open(fw_tmp, "ab") as f:
                f.write(v)
            fw_received += len(v)
            # 報進度（可能過長未分段）
            if (fw_received & 0x3FF) == 0:
                tx_json({"t":"fw","stage":"recv","n":fw_received})
        except:
            fw_receiving = False
            tx_json({"t":"fw","stage":"recv","ok":False,"err":"write_failed"})

def _irq(event, data):
    if event == 3:  # _IRQ_GATTS_WRITE
        conn_handle, attr_handle = data
        if attr_handle == _handle_rx:
            v = ble.gatts_read(_handle_rx)
            on_write(v)
    elif event == 1:  # _IRQ_CENTRAL_CONNECT
        # 原樣：不保存 conn_handle，notify 一律用 0
        pass
    elif event == 2:  # _IRQ_CENTRAL_DISCONNECT
        # 重新廣播
        ble.gap_advertise(100, adv_data=_adv())

def _adv():
    # Flags + 16-bit UUID + Name (C6-LED)
    return (
        b'\x02\x01\x06' +
        b'\x03\x03\xf0\xff' +
        b'\x07\x09C6-LED'
    )

def setup():
    global _handle_rx, _handle_tx
    (( _handle_rx, _handle_tx ),) = ble.gatts_register_services((SERVICE,))
    ble.irq(_irq)
    ble.gap_advertise(100, adv_data=_adv())

setup()
# 初始訊息（可能因未連線、未啟用通知而被丟）
tx_json({"t":"fw","stage":"idle"})
