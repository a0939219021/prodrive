# main.py — ESP32-C6 BLE + WS2812 + OTA (MicroPython, iOS-friendly, with INFO READ char)
# Version: v2025.08.27-r4-iosfix5-quiet (INFO char added; handle parsing tolerant; adv interval fixed)
#
import sys, time, machine, ujson
import uos as os
import bluetooth
from machine import Pin
try:
    import neopixel
except:
    neopixel = None

FW_VERSION = "v2025.08.27-r4-iosfix5-quiet"
FW_BUILD   = "r4-iosfix5-quiet"

# ===== Project constants =====
DEV_NAME = "C6-LED"
WS_PIN = 8
WS_COUNT = 1
OTA_DIR = "/ota"
CFG_FILE = "/cfg.json"

def ensure_dir(path: str):
    if not path or path == "/":
        return
    try:
        os.stat(path)
        return
    except OSError:
        i = path.rfind("/")
        if i > 0:
            ensure_dir(path[:i])
        try:
            os.mkdir(path)
        except OSError:
            try:
                os.stat(path)
            except OSError:
                raise

def load_cfg():
    try:
        with open(CFG_FILE, "r") as f:
            return ujson.loads(f.read())
    except:
        return {"mode": 0, "brightness": 128, "speed": 8}

def save_cfg(cfg):
    try:
        with open(CFG_FILE, "w") as f:
            f.write(ujson.dumps(cfg))
    except Exception as e:
        print("CFG save failed:", e)

CFG = load_cfg()

class StatusLED:
    def __init__(self, pin=WS_PIN, n=WS_COUNT):
        self.ok = False
        if neopixel:
            try:
                self.np = neopixel.NeoPixel(Pin(pin, Pin.OUT), n)
                self.ok = True
            except Exception as e:
                print("neopixel init fail:", e)

    def set_rgb(self, r, g, b, show=True):
        if not self.ok: return
        for i in range(WS_COUNT):
            self.np[i] = (r, g, b)
        if show:
            self.np.write()

    def off(self):
        self.set_rgb(0,0,0)

LED = StatusLED()
LED.set_rgb(0, 8, 0)

ensure_dir(OTA_DIR)

UUID_SVC = bluetooth.UUID(0xFFF0)
UUID_TX  = bluetooth.UUID(0xFFF1)  # Notify
UUID_RX  = bluetooth.UUID(0xFFF2)  # Write/WriteNR
UUID_INF = bluetooth.UUID(0xFFF3)  # Read (device info/version)

FLAG_NOTIFY = bluetooth.FLAG_NOTIFY
FLAG_WRITE  = bluetooth.FLAG_WRITE
FLAG_WNR    = bluetooth.FLAG_WRITE_NO_RESPONSE
FLAG_READ   = bluetooth.FLAG_READ

class BLEUartLike:
    def __init__(self, name=DEV_NAME):
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)

        TX  = (UUID_TX,  FLAG_NOTIFY)
        RX  = (UUID_RX,  FLAG_WRITE | FLAG_WNR)
        INF = (UUID_INF, FLAG_READ)  # safe to read

        SVC = (UUID_SVC, (TX, RX, INF))

        handles = self._ble.gatts_register_services((SVC,))
        # Tolerate shapes: ((tx, rx, inf),)  or  (tx, rx, inf)
        self.tx_h = self.rx_h = self.inf_h = None
        if isinstance(handles, tuple):
            if len(handles) == 1 and isinstance(handles[0], (tuple, list)):
                vals = handles[0]
                if len(vals) >= 3:
                    self.tx_h, self.rx_h, self.inf_h = vals[0], vals[1], vals[2]
            elif len(handles) >= 3 and isinstance(handles[0], int):
                self.tx_h, self.rx_h, self.inf_h = handles[0], handles[1], handles[2]
        if self.tx_h is None or self.rx_h is None or self.inf_h is None:
            try:
                self.tx_h, self.rx_h, self.inf_h = handles[0]
            except:
                raise RuntimeError("Unexpected handle shape: %r" % (handles,))

        # Prepare INFO payload
        info = ujson.dumps({"ver": FW_VERSION, "build": FW_BUILD})
        try:
            self._ble.gatts_write(self.inf_h, info)
        except Exception as e:
            print("INFO write failed:", e)

        self._connections = set()
        self._ota = None
        # Advertise every 100 ms (MicroPython takes microseconds)
        self._ble.gap_advertise(100000, adv_data=self._payload(name))

    def _payload(self, name):
        name_bytes = name.encode()
        return bytearray(b"\x02\x01\x06") + bytes((len(name_bytes)+1, 0x09)) + name_bytes

    def _irq(self, event, data):
        if event == 1:  # central connect
            conn_handle, addr_type, addr = data
            self._connections.add(conn_handle)
            LED.set_rgb(0, 0, 8)
        elif event == 2:  # central disconnect
            conn_handle, addr_type, addr = data
            self._connections.discard(conn_handle)
            self._ota = None
            LED.set_rgb(0, 8, 0)
            self._ble.gap_advertise(100000, adv_data=self._payload(DEV_NAME))
        elif event == 3:  # write
            conn_handle, value_handle = data
            if value_handle == self.rx_h:
                try:
                    raw = self._ble.gatts_read(self.rx_h)
                    self._handle_rx(raw, conn_handle)
                except Exception as e:
                    print("RX error:", e)

    def notify(self, payload: bytes):
        for ch in list(self._connections):
            try:
                self._ble.gatts_notify(ch, self.tx_h, payload)
            except Exception:
                pass

    # ===== OTA protocol =====
    # JSON control frames:
    #  {"op":"begin", "name":"main.py", "size":1234, "sha256":"...hex..."}
    #  {"op":"end"} / {"op":"commit"} / {"op":"reboot"}
    # Binary frames: raw chunk bytes (append to temp file)
    def _handle_rx(self, buf, ch):
        if len(buf) and buf[:1] in (b'{',):
            try:
                msg = ujson.loads(buf)
            except:
                self.notify(b'{"err":"json"}')
                return
            op = msg.get("op")
            if op == "begin":
                name = msg.get("name") or "incoming.bin"
                size = int(msg.get("size") or 0)
                sha  = msg.get("sha256") or ""
                tmpf = OTA_DIR + "/incoming.tmp"
                self._ota = {"name": name, "size": size, "sha": sha, "w": 0, "tmp": tmpf}
                try:
                    try:
                        os.remove(tmpf)
                    except:
                        pass
                    with open(tmpf, "wb") as f:
                        pass
                    LED.set_rgb(8, 4, 0)
                    self.notify(b'{"ack":"begin"}')
                except Exception:
                    self.notify(b'{"err":"fs"}')
            elif op == "end":
                if not self._ota:
                    self.notify(b'{"err":"noctx"}')
                    return
                ok = (self._ota["w"] == self._ota["size"])
                self.notify(ujson.dumps({"ack":"end","ok":bool(ok),"w":self._ota["w"]}).encode())
                LED.set_rgb(0, 8, 0 if ok else 8)
            elif op == "commit":
                if not self._ota:
                    self.notify(b'{"err":"noctx"}')
                    return
                try:
                    final = OTA_DIR + "/" + self._ota["name"]
                    try:
                        os.remove(final)
                    except:
                        pass
                    os.rename(self._ota["tmp"], final)
                    with open(OTA_DIR + "/pending.json", "w") as f:
                        f.write(ujson.dumps({"install": final, "target": self._ota["name"]}))
                    self.notify(b'{"ack":"commit"}')
                    LED.set_rgb(0, 0, 8)
                except Exception:
                    self.notify(b'{"err":"commit"}')
            elif op == "reboot":
                self.notify(b'{"ack":"reboot"}')
                time.sleep_ms(200)
                machine.reset()
            elif op == "set":
                changed = False
                for k in ("mode","brightness","speed"):
                    if k in msg:
                        try:
                            CFG[k] = int(msg[k])
                            changed = True
                        except:
                            pass
                if changed:
                    save_cfg(CFG)
                self.notify(ujson.dumps({"ack":"set","cfg":CFG}).encode())
            else:
                self.notify(b'{"err":"op"}')
        else:
            if not self._ota:
                self.notify(b'{"err":"noctx"}')
                return
            try:
                with open(self._ota["tmp"], "ab") as f:
                    f.write(buf)
                self._ota["w"] += len(buf)
                if (self._ota["w"] % 4096) < len(buf):
                    self.notify(ujson.dumps({"pg": self._ota["w"]}).encode())
            except Exception:
                self.notify(b'{"err":"write"}')

BLE = BLEUartLike()

def _apply_pending():
    p = OTA_DIR + "/pending.json"
    try:
        with open(p, "r") as f:
            info = ujson.loads(f.read())
        src = info.get("install")
        tgt = info.get("target")
        if src and tgt:
            dest = "/" + tgt
            bak = dest + ".bak"
            try:
                os.stat(dest)
                try:
                    os.remove(bak)
                except:
                    pass
                os.rename(dest, bak)
            except OSError:
                pass
            try:
                try:
                    os.remove(dest)
                except:
                    pass
                os.rename(src, dest)
                try:
                    os.remove(p)
                except:
                    pass
                print("OTA installed:", dest)
                LED.set_rgb(0, 8, 0)
            except Exception as e:
                print("OTA install failed:", e)
                LED.set_rgb(8, 0, 0)
    except Exception:
        pass

_apply_pending()

def heartbeat():
    while True:
        try:
            if LED.ok:
                b = (CFG.get("brightness",128) & 0xFF) // 8
                if b < 1: b = 1
                LED.set_rgb(0, b, 0)
            time.sleep_ms(500)
        except Exception:
            time.sleep_ms(500)

try:
    heartbeat()
except KeyboardInterrupt:
    pass

