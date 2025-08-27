# c6_can_listen_diag.py  — v2025.08.27-r4
# 修正：FW:END 的驗證移出 BLE IRQ，在主循環完成；加入 FW:STAT；UI 可確實收到 Verify OK
# 其他功能承接 r3（MAP/RPM/MAF/Thr、LED、BIT/CLK、ID 篩選、超時、輪詢等）

import time, ujson, os
from machine import Pin, SPI
import ubluetooth as bt
import neopixel
try:
    import uhashlib as hashlib
except:
    import hashlib

VERSION = "v2025.08.27-r4"

# ========= 硬體腳位 =========
SCK, MOSI, MISO, CS, INT = 14, 15, 6, 7, 23
WS_PIN, WS_NUM = 8, 1
CAN_STB_PIN = None

# ========= BLE UUIDs（128-bit）=========
SVC = "12345678-1234-5678-1234-56789abcdef0"
TX  = "12345678-1234-5678-1234-56789abcdef1"  # notify
RX  = "12345678-1234-5678-1234-56789abcdef2"  # text commands
FILE= "12345678-1234-5678-1234-56789abcdef3"  # binary file chunks (write/wnr)

# ========= OBD =========
REQ = 0x7DF
PIDS = [0x0C, 0x10, 0x11, 0x0B]  # RPM, MAF, Throttle, MAP
RESP_MIN_ID, RESP_MAX_ID = 0x7E8, 0x7EF
def mk_req(pid): return bytes([0x02, 0x01, pid, 0, 0, 0, 0, 0])

def parse(pid, data):
    if len(data) < 4 or data[1] != 0x41 or data[2] != pid: return None
    if pid == 0x0C:
        if len(data) < 5: return None
        A,B=data[3],data[4]; return ("rpm", int((256*A+B)/4))
    if pid == 0x10:
        if len(data) < 5: return None
        A,B=data[3],data[4]; return ("maf", (256*A+B)/100.0)
    if pid == 0x11:
        A=data[3]; return ("thr", round(A*100.0/255.0,1))
    if pid == 0x0B:
        A=data[3]; return ("map", int(A))
    return None

# ========= WS2812 =========
np = neopixel.NeoPixel(Pin(WS_PIN), WS_NUM)
def pix(c): np[0]=c; np.write()
OFF=(0,0,0); BLUE=(0,0,60); GREEN=(0,60,0); RED=(60,0,0); WHITE=(60,60,60)
def flash(color, ms=60): np[0]=color; np.write(); time.sleep_ms(ms); np[0]=OFF; np.write()

# ========= BLE 事件常數 =========
IRQ_CONN  = getattr(bt, "_IRQ_CENTRAL_CONNECT", 1)
IRQ_DISC  = getattr(bt, "_IRQ_CENTRAL_DISCONNECT", 2)
IRQ_WRITE = getattr(bt, "_IRQ_GATTS_WRITE", 3)
FLAG_WRITE_NR = getattr(bt, "FLAG_WRITE_NO_RESPONSE", 0x04)

def uuid128_to_le(uuid_str):
    hx = uuid_str.replace("-", "")
    arr = bytearray(16)
    for i in range(16):
        arr[i] = int(hx[2*i:2*i+2], 16)
    for i in range(8):
        t = arr[i]; arr[i] = arr[15 - i]; arr[15 - i] = t
    return bytes(arr)

# ========= MCP2515 =========
class MCP2515:
    RESET=0xC0; READ=0x03; WRITE=0x02; RTS=0x80; BIT=0x05
    CANCTRL=0x0F; CANSTAT=0x0E; CANINTE=0x2B; CANINTF=0x2C
    RXB0CTRL=0x60; RXB1CTRL=0x70; RXB0SIDH=0x61; RXB0SIDL=0x62; RXB0DLC=0x65; RXB0D0=0x66
    TXB0CTRL=0x30; TXB0SIDH=0x31; TXB0SIDL=0x32; TXB0DLC=0x35; TXB0D0=0x36
    CNF1=0x2A; CNF2=0x29; CNF3=0x28; EFLG=0x2D
    MODE_NORMAL=0x00; MODE_SLEEP=0x20; MODE_LOOP=0x40; MODE_LISTEN=0x60; MODE_CONFIG=0x80
    def __init__(self, spi, cs, irq):
        self.spi=spi; self.cs=Pin(cs,Pin.OUT,value=1); self.irq=Pin(irq,Pin.IN,Pin.PULL_UP)
        self.reset(); time.sleep_ms(5)
    def _cs(self,b): self.cs.value(0 if b else 1)
    def reset(self): self._cs(True); self.spi.write(bytes([self.RESET])); self._cs(False)
    def rd(self,addr): self._cs(True); self.spi.write(bytes([self.READ,addr])); v=self.spi.read(1)[0]; self._cs(False); return v
    def wr(self,addr,val): self._cs(True); self.spi.write(bytes([self.WRITE,addr,val])); self._cs(False)
    def bm(self,addr,mask,data): self._cs(True); self.spi.write(bytes([self.BIT,addr,mask,data])); self._cs(False)
    def mode(self,m,tries=50):
        self.bm(self.CANCTRL,0xE0,m)
        for _ in range(tries):
            if (self.rd(self.CANSTAT)&0xE0)==m: return True
            time.sleep_ms(2)
        return False
    def cfg(self, bitrate_k=250, clk_mhz=16):
        self.mode(self.MODE_CONFIG)
        if   bitrate_k==500 and clk_mhz==8:    cnf1,cnf2,cnf3=(0x00,0x90,0x02)
        elif bitrate_k==250 and clk_mhz==8:    cnf1,cnf2,cnf3=(0x01,0x90,0x02)
        elif bitrate_k==500 and clk_mhz==16:   cnf1,cnf2,cnf3=(0x00,0x9E,0x03)
        elif bitrate_k==250 and clk_mhz==16:   cnf1,cnf2,cnf3=(0x01,0xB1,0x05)
        else:                                  cnf1,cnf2,cnf3=(0x01,0xB1,0x05)
        self.wr(self.CNF1,cnf1); self.wr(self.CNF2,cnf2); self.wr(self.CNF3,cnf3)
        self.wr(self.RXB0CTRL,0x60); self.wr(self.RXB1CTRL,0x60)
        self.wr(self.CANINTE,0x03)
        return True
    def start_normal(self): return self.mode(self.MODE_NORMAL)
    def start_listen(self): return self.mode(self.MODE_LISTEN)
    def send(self,canid,data,timeout_ms=250):
        sidh=(canid>>3)&0xFF; sidl=(canid&0x07)<<5
        self.wr(self.TXB0SIDH,sidh); self.wr(self.TXB0SIDL,sidl)
        self.wr(self.TXB0DLC,len(data)&0x0F)
        self._cs(True); self.spi.write(bytes([self.WRITE,self.TXB0D0])+bytes(data)); self._cs(False)
        self._cs(True); self.spi.write(bytes([self.RTS|0x01])); self._cs(False)
        t0=time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(),t0)<timeout_ms:
            if self.rd(self.CANINTF)&0x04:
                self.bm(self.CANINTF,0x04,0x00); return True
        return False
    def recv(self):
        if self.rd(self.CANINTF)&0x01:
            sidh=self.rd(self.RXB0SIDH); sidl=self.rd(self.RXB0SIDL)
            canid=(sidh<<3)|(sidl>>5)
            dlc=self.rd(self.RXB0DLC)&0x0F
            self._cs(True); self.spi.write(bytes([self.READ,self.RXB0D0])); data=self.spi.read(dlc); self._cs(False)
            self.bm(self.CANINTF,0x01,0x00)
            return (canid, data)
        return None

# ========= BLE =========
class BLEWrap:
    def __init__(self, name="C6-LED"):
        self._name=name
        self.ble=bt.BLE(); self.ble.active(True)
        try:
            self.ble.config(gap_name=name); self.ble.config(mtu=247)
        except: pass
        self.ble.irq(self._irq)
        svc=bt.UUID(SVC); tx=bt.UUID(TX); rx=bt.UUID(RX); fch=bt.UUID(FILE)
        self._tx=(tx, bt.FLAG_NOTIFY|bt.FLAG_READ)
        self._rx=(rx, bt.FLAG_WRITE)
        self._file=(fch, bt.FLAG_WRITE|FLAG_WRITE_NR)
        ((self.hTX,self.hRX,self.hFILE),)=self.ble.gatts_register_services([(svc,(self._tx,self._rx,self._file))])
        try:
            self.ble.gatts_set_buffer(self.hTX, 200, False)
            self.ble.gatts_set_buffer(self.hRX, 200, True)
            self.ble.gatts_set_buffer(self.hFILE, 512, True)
        except: pass
        adv=self._adv_payload(name=self._name, svcs=[SVC])
        self.ble.gap_advertise(200_000, adv_data=adv, resp_data=None)
        self.conns=set()

        # 外掛回呼
        self.on_connect=None
        self.on_disconnect=None
        self.on_cmd=None

        # OTA 狀態
        self.fw_active=False
        self.fw_size=0
        self.fw_rx=0
        self.fw_sha_hex=""
        self.fw_sha=None
        self.fw_tmp="fw.tmp"
        self.fw_target="c6_can_listen_diag.py"
        self._fwf=None
        # 將 FW:END 驗證移交主循環
        self.fw_end_pending=False
        self.fw_end_deadline=0

        # 指令佇列（避免在 IRQ 內做重活）
        self.cmd_queue=[]

    def _adv_payload(self, name=None, svcs=None):
        p=bytearray()
        def a(t,v): p.extend(bytes((len(v)+1,t))+v)
        a(0x01, b"\x06")
        if svcs:
            buf=bytearray()
            for u in svcs: buf.extend(uuid128_to_le(u))
            if buf: a(0x07, bytes(buf))
        if name: a(0x09, name.encode())
        return bytes(p)

    def _irq(self, ev, data):
        if ev==IRQ_CONN:
            self.conns.add(data[0])
            if self.on_connect:
                try: self.on_connect()
                except: pass
        elif ev==IRQ_DISC:
            self.conns.discard(data[0])
            self.ble.gap_advertise(200_000, adv_data=self._adv_payload(name=self._name, svcs=[SVC]), resp_data=None)
            if self.fw_active: self._fw_abort("disconnect")
            if self.on_disconnect:
                try: self.on_disconnect()
                except: pass
        elif ev==IRQ_WRITE:
            ch = data[1]
            if ch == self.hRX:
                # 只把指令塞到佇列，主循環再處理
                try:
                    cmd=self.ble.gatts_read(self.hRX).decode().strip()
                    self.cmd_queue.append(cmd)
                except: pass
            elif ch == self.hFILE and self.fw_active:
                # 檔案 chunk 可直接寫（小量、快速）
                try:
                    chunk=self.ble.gatts_read(self.hFILE)
                    if chunk: self._fw_write(chunk)
                except: pass

    # ===== OTA =====
    def _fw_notify(self, ev, **kw):
        kw.update({"t":"fw","ev":ev})
        self.notify_json(kw)

    def _exists(self, path):
        try:
            os.stat(path); return True
        except: return False

    def _fw_begin(self, target, size, sha_hex):
        try:
            if self._fwf: self._fwf.close()
        except: pass
        try:
            if self.fw_tmp and self._exists(self.fw_tmp): os.remove(self.fw_tmp)
        except: pass
        try:
            self._fwf = open(self.fw_tmp, "wb")
        except:
            self._fw_notify("begin", ok=False, err="open"); return False
        self.fw_active=True
        self.fw_target=target or self.fw_target
        self.fw_size=int(size); self.fw_rx=0
        self.fw_sha_hex=(sha_hex or "").lower()
        self.fw_sha=hashlib.sha256()
        self.fw_end_pending=False; self.fw_end_deadline=0
        self._fw_notify("begin", ok=True, target=self.fw_target, size=self.fw_size)
        return True

    def _fw_write(self, chunk:bytes):
        try:
            self._fwf.write(chunk)
            self.fw_sha.update(chunk)
            self.fw_rx += len(chunk)
        except:
            self._fw_notify("write", ok=False, err="write")
            self._fw_abort("io")
            return
        if (self.fw_rx % 2048) < len(chunk):
            self._fw_notify("progress", rx=self.fw_rx, size=self.fw_size)

    def _fw_end(self):
        if not self.fw_active:
            self._fw_notify("end", ok=False, err="no_session"); return
        try:
            self._fwf.flush(); self._fwf.close()
        except: pass
        calc = self.fw_sha.digest()
        hexs = "".join("{:02x}".format(b) for b in calc)
        if (self.fw_rx != self.fw_size) or (hexs != self.fw_sha_hex):
            self._fw_notify("end", ok=False, rx=self.fw_rx, size=self.fw_size, sha=hexs)
            self._fw_abort("verify")
            return
        try:
            if self._exists(self.fw_target):
                try:
                    if self._exists(self.fw_target+".bak"): os.remove(self.fw_target+".bak")
                except: pass
                os.rename(self.fw_target, self.fw_target+".bak")
        except: pass
        try:
            os.rename(self.fw_tmp, self.fw_target)
        except:
            self._fw_notify("end", ok=False, err="rename"); self._fw_abort("rename"); return
        self._fw_notify("end", ok=True, target=self.fw_target, sha=self.fw_sha_hex)

    def _fw_abort(self, reason="abort"):
        try:
            if self._fwf: self._fwf.close()
        except: pass
        try:
            if self._exists(self.fw_tmp): os.remove(self.fw_tmp)
        except: pass
        self.fw_active=False; self.fw_size=0; self.fw_rx=0
        self.fw_sha_hex=""; self.fw_sha=None; self._fwf=None
        self.fw_end_pending=False; self.fw_end_deadline=0
        self._fw_notify("abort", reason=reason)

    def notify_json(self, obj, chunk=180):
        if not self.conns: return
        s=ujson.dumps(obj).encode()
        for ch in list(self.conns):
            try:
                for i in range(0,len(s),chunk):
                    self.ble.gatts_notify(ch, self.hTX, s[i:i+chunk])
            except: pass

# ========= 主程式 =========
def main():
    print("[FW]", VERSION)
    pix(OFF)
    if CAN_STB_PIN is not None:
        try: Pin(CAN_STB_PIN, Pin.OUT, value=0)
        except: pass

    # SPI（試 2→1）
    spi=None
    for bus in (2,1):
        try:
            spi=SPI(bus, baudrate=8_000_000, polarity=0, phase=0,
                    sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO)); break
        except: pass
    if not spi:
        print("[SPI] init failed")
        while True: pix(RED); time.sleep_ms(200); pix(OFF); time.sleep_ms(200)

    # BLE
    ble=BLEWrap("C6-LED")

    # CAN 預設
    state={"bit":250,"clk":16,"mode":"LISTEN","dump":True,"poll":False}
    mcp=MCP2515(spi, CS, INT)
    mcp.cfg(state["bit"], state["clk"]); mcp.start_listen()
    print("[CAN] LISTEN @%dk, clk %dMHz" % (state["bit"], state["clk"]))

    # 斷線 → LISTEN + PID:OFF
    def _on_ble_disconnect():
        changed=False
        if state["poll"]: state["poll"]=False; changed=True; print("[BLE] disconnect -> PID:OFF")
        if state["mode"]!="LISTEN": state["mode"]="LISTEN"; mcp.start_listen(); changed=True; print("[BLE] disconnect -> MODE:LISTEN")
        if not changed: print("[BLE] disconnect (no state change)")
    ble.on_disconnect=_on_ble_disconnect

    # 連線時回報版本
    def _on_ble_connect():
        ble.notify_json({"t":"info","fw":VERSION})
    ble.on_connect=_on_ble_connect

    import machine
    def handle_cmd(s):
        up=s.strip().upper()
        # CAN 控制
        if up.startswith("BIT:"):
            try:
                v=int(up.split(":")[1])
                if v in (250,500):
                    state["bit"]=v; mcp.cfg(state["bit"], state["clk"])
                    ok=mcp.start_listen() if state["mode"]=="LISTEN" else mcp.start_normal()
                    print("[CAN] BIT =",v,"ok=",ok)
            except: pass; return
        if up.startswith("CLK:"):
            try:
                v=int(up.split(":")[1])
                if v in (8,16):
                    state["clk"]=v; mcp.cfg(state["bit"], state["clk"])
                    ok=mcp.start_listen() if state["mode"]=="LISTEN" else mcp.start_normal()
                    print("[CAN] CLK =",v,"ok=",ok)
            except: pass; return
        if up=="MODE:LISTEN": state["mode"]="LISTEN"; mcp.start_listen(); print("[CAN] mode LISTEN"); return
        if up=="MODE:NORMAL": state["mode"]="NORMAL"; mcp.start_normal(); print("[CAN] mode NORMAL"); return
        if up=="DUMP:ON": state["dump"]=True; print("[CAN] dump ON"); return
        if up=="DUMP:OFF": state["dump"]=False; print("[CAN] dump OFF"); return
        if up=="PID:ON": state["poll"]=True; print("[OBD] poll ON"); return
        if up=="PID:OFF": state["poll"]=False; print("[OBD] poll OFF"); return
        # OTA 控制
        if up.startswith("FW:BEGIN"):
            try:
                parts=s.strip().split()
                target = parts[1] if len(parts)>1 else "c6_can_listen_diag.py"
                size   = int(parts[2]) if len(parts)>2 else 0
                shahex = parts[3] if len(parts)>3 else ""
                ble._fw_begin(target, size, shahex)
            except: ble._fw_notify("begin", ok=False, err="bad_args")
            return
        if up=="FW:END":
            # 不在 IRQ 內做驗證；交給主循環
            ble.fw_end_pending=True
            ble.fw_end_deadline=time.ticks_add(time.ticks_ms(), 5000)
            ble._fw_notify("verify", rx=ble.fw_rx, size=ble.fw_size)
            return
        if up=="FW:ABORT": ble._fw_abort("user"); return
        if up=="FW:APPLY":
            ble._fw_notify("apply", ok=True, target=ble.fw_target)
            time.sleep_ms(200); machine.reset(); return
        if up=="FW:STAT":
            ble._fw_notify("stat", active=ble.fw_active, rx=ble.fw_rx, size=ble.fw_size, pending=ble.fw_end_pending)
            return
        # 其他：回韌體版本
        ble.notify_json({"t":"info","fw":VERSION})

    ble.on_cmd=handle_cmd

    vals={"rpm":None,"maf":None,"thr":None,"map":None}
    i=0; tQ=0
    t_led=time.ticks_ms()

    while True:
        now=time.ticks_ms()

        # 先清空指令佇列（包含 FW 指令）
        if ble.cmd_queue:
            # 每回合處理最多 4 個，避免阻塞
            for _ in range(min(4, len(ble.cmd_queue))):
                cmd = ble.cmd_queue.pop(0)
                try: handle_cmd(cmd)
                except: pass

        # 如有等待 FW:END，達到大小或逾時就驗證
        if ble.fw_end_pending:
            if (ble.fw_rx >= ble.fw_size) or (time.ticks_diff(ble.fw_end_deadline, now) <= 0):
                ble.fw_end_pending=False
                try: ble._fw_end()
                except: ble._fw_abort("end_exc")

        # LED：連線快綠、廣播慢藍
        period=100 if ble.conns else 600
        if time.ticks_diff(now,t_led)>=period:
            pix(GREEN if ble.conns else BLUE); time.sleep_ms(30); pix(OFF); t_led=now

        # Listen：Dump + OBD 解析（只收 0x7E8~0x7EF）
        rx=mcp.recv()
        if rx:
            canid,data=rx
            if state["dump"]:
                ble.notify_json({"t":"raw","id":canid,"d":data.hex()})
            if RESP_MIN_ID <= canid <= RESP_MAX_ID:
                for pid in PIDS:
                    pr=parse(pid,data)
                    if pr:
                        k,v=pr; vals[k]=v
                        msg={"t":"obd"}; msg.update(vals)
                        ble.notify_json(msg)
                        if ble.conns: flash(WHITE,20)

        # NORMAL + PID:ON：輪詢
        if state["mode"]=="NORMAL" and state["poll"] and time.ticks_diff(now,tQ)>=180:
            pid=PIDS[i]; i=(i+1)%len(PIDS)
            ok=mcp.send(REQ, mk_req(pid), timeout_ms=250)
            if not ok: ble.notify_json({"t":"err","tx":"timeout","pid":pid})
            tQ=now

        time.sleep_ms(2)

if __name__=="__main__":
    main()
