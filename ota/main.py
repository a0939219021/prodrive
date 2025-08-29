# main.py  (v2025.08.27-r4-iosfix5-quiet)
# ESP32-C6-Pico + Pico-CAN-B — BLE + OBD2 over MCP2515 + OTA
# 變更：iOS 安全檔流 (FILE UUID ...def4)、OTA 期間靜音 (quiet) 以避免 GATT 壅塞、
#      改善 FW:END 後驗證通知、減少噪訊、避免卡在 verifying。

import time, ujson, os
import uhashlib as hashlib
from machine import Pin, SPI, reset
import ubluetooth as bt
import neopixel

# ========= 硬體腳位 =========
SCK, MOSI, MISO, CS, INT = 14, 15, 6, 7, 23
WS_PIN, WS_NUM = 8, 1
CAN_STB_PIN = None

# ========= WS2812 =========
np = neopixel.NeoPixel(Pin(WS_PIN), WS_NUM)
def pix(c):
    try:
        np[0] = c
        np.write()
    except:
        pass
OFF=(0,0,0); BLUE=(0,0,60); GREEN=(0,60,0); RED=(60,0,0); YEL=(40,30,0)

# ========= 版本字串 =========
FW_VERSION = "v2025.08.27-r4-iosfix5-quiet"

# ========= BLE UUIDs（128-bit）=========
SVC = "12345678-1234-5678-1234-56789abcdef0"
TX  = "12345678-1234-5678-1234-56789abcdef1"  # notify/read
RX  = "12345678-1234-5678-1234-56789abcdef2"  # write (控制指令)
FCH = "12345678-1234-5678-1234-56789abcdef4"  # write / write_no_response (韌體資料流)

# ========= OBD =========
REQ = 0x7DF
PIDS = [0x0C, 0x10, 0x11, 0x0B]  # RPM, MAF, Throttle, MAP
def mk_req(pid): return bytes([0x02, 0x01, pid, 0, 0, 0, 0, 0])

def parse(pid, data):
    if len(data) < 4 or data[1] != 0x41 or data[2] != pid: return None
    if pid == 0x0C and len(data)>=5:
        A,B=data[3],data[4]; return ("rpm", int((256*A+B)//4))
    if pid == 0x10 and len(data)>=5:
        A,B=data[3],data[4]; return ("maf", (256*A+B)/100.0)
    if pid == 0x11 and len(data)>=4:
        A=data[3]; return ("thr", round(A*100.0/255.0,1))
    if pid == 0x0B and len(data)>=4:
        A=data[3]; return ("map", int(A))
    return None

# ========= MCP2515 =========
class MCP2515:
    RESET=0xC0; READ=0x03; WRITE=0x02; BIT=0x05; RTS=0x80
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
        elif bitrate_k==500 and clk_mhz==16:   cnf1,cnf2,cnf3=(0x00,0xD0,0x82)
        elif bitrate_k==250 and clk_mhz==16:   cnf1,cnf2,cnf3=(0x01,0xD0,0x82)
        else:                                  cnf1,cnf2,cnf3=(0x01,0xD0,0x82)
        self.wr(self.CNF1,cnf1); self.wr(self.CNF2,cnf2); self.wr(self.CNF3,cnf3)
        self.wr(self.RXB0CTRL,0x60); self.wr(self.RXB1CTRL,0x60)
        self.wr(self.CANINTE,0x03)
        return True
    def start_normal(self): return self.mode(self.MODE_NORMAL)
    def start_listen(self): return self.mode(self.MODE_LISTEN)
    def send(self,canid,data,timeout_ms=60):
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

# ========= OTA（檔案接收 + 驗證 + 置換）=========
class OTA:
    def __init__(self, ble_notify, set_quiet_cb=None):
        self.active=False; self.target=None; self.tmp=None
        self.size=0; self.rx=0; self.sha=None; self.f=None
        self._notify=ble_notify; self._tick=time.ticks_ms()
        self._set_quiet = set_quiet_cb
    def _n(self, payload):
        try:
            d={"t":"fw"}
            if isinstance(payload, dict):
                for k in payload: d[k]=payload[k]
            self._notify(d)
        except:
            pass
    def begin(self, target, size, sha_hex):
        self.abort("new begin")
        self.target=target; self.tmp=target+".part"; self.size=int(size or 0)
        self.rx=0; self.sha=(sha_hex or "").lower()
        try:
            try: os.remove(self.tmp)
            except: pass
            self.f=open(self.tmp, "wb")
            self.active=True
            if self._set_quiet:
                try: self._set_quiet(True)
                except: pass
            pix(YEL)
            self._n({"ev":"begin","ok":True,"target":self.target,"size":self.size})
            return True
        except Exception as e:
            self._n({"ev":"begin","ok":False,"err":str(e)}); self.active=False; self.f=None
            return False
    def on_chunk(self, data):
        if not self.active or not self.f: return
        try:
            self.f.write(data); self.rx += len(data)
            now=time.ticks_ms()
            if time.ticks_diff(now, self._tick)>=250:
                self._tick=now; self._n({"ev":"progress","rx":self.rx,"size":self.size})
        except Exception as e:
            self._n({"ev":"abort","reason":"write_err:"+str(e)}); self.abort("write_err")
    def stat(self):
        self._n({"ev":"stat","active":bool(self.active),"rx":self.rx,"size":self.size})
    def _sha256_file(self, path):
        h=hashlib.sha256()
        with open(path,"rb") as f:
            while True:
                b=f.read(2048)
                if not b: break
                h.update(b)
        return "".join("%02x"%b for b in h.digest())
    def end(self):
        if not self.active:
            self._n({"ev":"end","ok":False,"rx":self.rx,"size":self.size}); return False
        try:
            if self.f:
                try: self.f.flush()
                except: pass
                self.f.close(); self.f=None
            if self.size>0 and self.rx!=self.size:
                self._n({"ev":"end","ok":False,"rx":self.rx,"size":self.size}); self.abort("size_mismatch"); return False
            self._n({"ev":"verify","rx":self.rx,"size":self.size})
            sha = self._sha256_file(self.tmp)
            ok = (sha == (self.sha or ""))
            if not ok:
                self._n({"ev":"end","ok":False,"rx":self.rx,"size":self.size,"sha":sha})
                self.abort("sha_mismatch")
                pix(RED)
                return False
            try: os.remove(self.target)
            except: pass
            os.rename(self.tmp, self.target)
            self.active=False
            self._n({"ev":"end","ok":True,"rx":self.rx,"size":self.size,"sha":sha})
            pix(GREEN)
            if self._set_quiet:
                try: self._set_quiet(False)
                except: pass
            return True
        except Exception as e:
            self._n({"ev":"abort","reason":"end_err:"+str(e)}); self.abort("end_err"); return False
    def apply(self):
        self._n({"ev":"apply"})
        time.sleep_ms(200)
        reset()
    def abort(self, reason=""):
        if self.f:
            try: self.f.close()
            except: pass
            self.f=None
        if self.tmp:
            try: os.remove(self.tmp)
            except: pass
        self.active=False; self.rx=0; self.size=0
        if self._set_quiet:
            try: self._set_quiet(False)
            except: pass
        if reason:
            try: self._n({"ev":"abort","reason":reason})
            except: pass
        pix(OFF)

# ========= BLE =========
IRQ_CONN  = getattr(bt, "_IRQ_CENTRAL_CONNECT", 1)
IRQ_DISC  = getattr(bt, "_IRQ_CENTRAL_DISCONNECT", 2)
IRQ_WRITE = getattr(bt, "_IRQ_GATTS_WRITE", 3)

def uuid128_to_le(uuid_str):
    hx = uuid_str.replace("-", "")
    arr = bytearray(16)
    for i in range(16):
        arr[i] = int(hx[2*i:2*i+2], 16)
    for i in range(8):
        t = arr[i]; arr[i] = arr[15 - i]; arr[15 - i] = t
    return bytes(arr)

class BLEWrap:
    def __init__(self, name="C6-LED"):
        self._name=name
        self.ble=bt.BLE(); self.ble.active(True)
        try:
            self.ble.config(gap_name=self._name)
            self.ble.config(rxbuf=1024)  # 更大的 BLE RX ring buffer
        except: pass
        self.ble.irq(self._irq)

        svc=bt.UUID(SVC); tx=bt.UUID(TX); rx=bt.UUID(RX); fch=bt.UUID(FCH)
        _tx=(tx, bt.FLAG_NOTIFY|bt.FLAG_READ)
        _rx=(rx, bt.FLAG_WRITE)  # 指令：WRITE (有回應)
        _f =(fch, bt.FLAG_WRITE | bt.FLAG_WRITE_NO_RESPONSE)  # 檔案：兩者都允許

        ((self.hTX,self.hRX,self.hFILE),)=self.ble.gatts_register_services([(svc,(_tx,_rx,_f))])

        try:
            self.ble.gatts_set_buffer(self.hTX, 200, False)
            self.ble.gatts_set_buffer(self.hRX, 260, True)
            self.ble.gatts_set_buffer(self.hFILE, 512, True)
        except: pass

        adv=self._adv_payload(name=self._name, svcs=[SVC])
        self.ble.gap_advertise(200_000, adv_data=adv, resp_data=None)
        self.conns=set(); self.on_cmd=None; self.on_file=None

        # 上線告知版本
        self.notify_json({"t":"info","fw":FW_VERSION})

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
            self.conns.add(data[0]); pix(BLUE)
        elif ev==IRQ_DISC:
            self.conns.discard(data[0]); pix(OFF)
            self.ble.gap_advertise(200_000, adv_data=self._adv_payload(name=self._name, svcs=[SVC]), resp_data=None)
        elif ev==IRQ_WRITE:
            ah = data[1]
            if ah == self.hRX and self.on_cmd:
                try:
                    cmd=self.ble.gatts_read(self.hRX).decode().strip()
                    self.on_cmd(cmd)
                except: pass
            elif ah == self.hFILE and self.on_file:
                try:
                    chunk = self.ble.gatts_read(self.hFILE)
                    if chunk: self.on_file(chunk)
                except: pass

    def notify_json(self, obj, chunk=180):
        if not self.conns: return
        try:
            s=ujson.dumps(obj).encode()
        except:
            return
        for ch in list(self.conns):
            try:
                for i in range(0,len(s),chunk):
                    self.ble.gatts_notify(ch,self.hTX,s[i:i+chunk])
            except:
                pass

# ========= 主程式 =========
def main():
    pix(OFF)
    if CAN_STB_PIN is not None:
        try: Pin(CAN_STB_PIN, Pin.OUT, value=0)
        except: pass

    spi=None
    for bus in (2,1):
        try:
            spi=SPI(bus, baudrate=8_000_000, polarity=0, phase=0,
                    sck=Pin(SCK), mosi=Pin(MOSI), miso=Pin(MISO)); break
        except: pass
    if not spi:
        print("[SPI] init failed")
        while True: pix(RED); time.sleep_ms(200); pix(OFF); time.sleep_ms(200)

    ble=BLEWrap("C6-LED")

    # 內部狀態
    state={"bit":250,"clk":16,"mode":"LISTEN","dump":True,"poll":False,"quiet":False}

    # 設定 quiet 的方法（供 OTA 呼叫）
    def set_quiet(on):
        state["quiet"]=bool(on)
        if on:
            state["dump"]=False
            state["poll"]=False
        # 不切換 CAN 模式，避免影響現場測試；只停止送出通知

    # OTA 物件
    ota = OTA(ble.notify_json, set_quiet_cb=set_quiet)

    # CAN
    mcp=MCP2515(spi, CS, INT)
    mcp.cfg(state["bit"], state["clk"]); mcp.start_listen()
    print("[CAN] LISTEN @%dk, clk %dMHz" % (state["bit"], state["clk"]))

    # 指令處理
    def on_cmd(s):
        up=s.strip().upper()
        if up.startswith("BIT:"):
            try:
                v=int(up.split(":")[1])
                if v in (250,500):
                    state["bit"]=v; mcp.cfg(state["bit"], state["clk"])
                    ok=mcp.start_listen() if state["mode"]=="LISTEN" else mcp.start_normal()
                    print("[CAN] BIT =",v,"ok=",ok)
            except: pass
        elif up.startswith("CLK:"):
            try:
                v=int(up.split(":")[1])
                if v in (8,16):
                    state["clk"]=v; mcp.cfg(state["bit"], state["clk"])
                    ok=mcp.start_listen() if state["mode"]=="LISTEN" else mcp.start_normal()
                    print("[CAN] CLK =",v,"ok=",ok)
            except: pass
        elif up=="MODE:LISTEN":
            state["mode"]="LISTEN"; mcp.start_listen(); print("[CAN] mode LISTEN")
        elif up=="MODE:NORMAL":
            state["mode"]="NORMAL"; mcp.start_normal(); print("[CAN] mode NORMAL")
        elif up=="DUMP:ON":
            if not state["quiet"]: state["dump"]=True
        elif up=="DUMP:OFF":
            state["dump"]=False
        elif up=="PID:ON":
            if not state["quiet"]: state["poll"]=True
        elif up=="PID:OFF":
            state["poll"]=False
        elif up=="QUIET:ON" or up=="FW:QUIET ON":
            set_quiet(True); print("[FW] quiet ON")
        elif up=="QUIET:OFF" or up=="FW:QUIET OFF":
            set_quiet(False); print("[FW] quiet OFF")
        # ===== OTA 指令 =====
        elif up.startswith("FW:BEGIN"):
            try:
                parts=s.split()
                tgt=parts[1]; size=int(parts[2]); sha=parts[3]
                ota.begin(tgt, size, sha)
            except Exception as e:
                ble.notify_json({"t":"fw","ev":"begin","ok":False,"err":str(e)})
        elif up=="FW:END":
            ota.end()
        elif up=="FW:STAT":
            ota.stat()
        elif up=="FW:APPLY":
            ota.apply()
        elif up=="FW:ABORT":
            ota.abort("host_abort")
        elif up=="VER?":
            ble.notify_json({"t":"info","fw":FW_VERSION})
        else:
            print("[BLE] unknown cmd:", s)

    def on_file(chunk):
        ota.on_chunk(chunk)

    ble.on_cmd=on_cmd
    ble.on_file=on_file

    # 迴圈
    vals={"rpm":None,"maf":None,"thr":None,"map":None}
    i=0; tQ=0

    while True:
        rx=mcp.recv()
        if rx and (not state["quiet"]):
            canid,data=rx
            if state["dump"]:
                ble.notify_json({"t":"raw","id":canid,"d":data.hex()})
            for pid in PIDS:
                pr=parse(pid,data)
                if pr:
                    k,v=pr; vals[k]=v
                    msg={"t":"obd"}
                    msg["rpm"]=vals["rpm"]; msg["maf"]=vals["maf"]; msg["thr"]=vals["thr"]; msg["map"]=vals["map"]
                    ble.notify_json(msg)
        if (not state["quiet"]) and state["mode"]=="NORMAL" and state["poll"] and time.ticks_ms()-tQ>=120:
            pid=PIDS[i]; i=(i+1)%len(PIDS)
            ok=mcp.send(REQ, mk_req(pid), timeout_ms=60)
            if not ok: ble.notify_json({"t":"err","tx":"timeout","pid":pid})
            tQ=time.ticks_ms()
        time.sleep_ms(2)

if __name__=="__main__":
    main()

