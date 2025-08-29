# main.py - v2025.08.28-r5-iosfix3
# ESP32-C6 + BLE + WS2812 + NVS
import time, bluetooth, struct
from machine import Pin
import neopixel
import esp32_nvs as nvs

# ====== WS2812 ======
LED_PIN = 8
NUM_LEDS = 1
np = neopixel.NeoPixel(Pin(LED_PIN), NUM_LEDS)

# ====== NVS (persistent storage) ======
nvs_set = nvs.NVS("storage")
def load_or_default(key, default):
    try:
        return nvs_set.get_i32(key)
    except:
        nvs_set.set_i32(key, default)
        return default

mode = load_or_default("mode", 0)
brightness = load_or_default("bright", 128)
speed = load_or_default("speed", 8)

# ====== BLE Setup ======
_IRQ_CENTRAL_CONNECT    = 1
_IRQ_CENTRAL_DISCONNECT = 2
_IRQ_GATTS_WRITE        = 3

SERVICE_UUID  = bluetooth.UUID("0000FFF0-0000-1000-8000-00805F9B34FB")
CHAR_MODE     = (bluetooth.UUID("0000FFF1-0000-1000-8000-00805F9B34FB"),
                 bluetooth.FLAG_READ | bluetooth.FLAG_WRITE)
CHAR_BRIGHT   = (bluetooth.UUID("0000FFF2-0000-1000-8000-00805F9B34FB"),
                 bluetooth.FLAG_READ | bluetooth.FLAG_WRITE)
CHAR_SPEED    = (bluetooth.UUID("0000FFF3-0000-1000-8000-00805F9B34FB"),
                 bluetooth.FLAG_READ | bluetooth.FLAG_WRITE)

ble = bluetooth.BLE()
ble.active(True)
handles = None

def ble_irq(event, data):
    global mode, brightness, speed
    if event == _IRQ_CENTRAL_CONNECT:
        print("Central connected")
    elif event == _IRQ_CENTRAL_DISCONNECT:
        print("Central disconnected")
        ble.gap_advertise(100)
    elif event == _IRQ_GATTS_WRITE:
        conn_handle, attr_handle = data
        if attr_handle == handles[0]:
            mode = int.from_bytes(ble.gatts_read(handles[0]), 'little')
            nvs_set.set_i32("mode", mode)
        elif attr_handle == handles[1]:
            brightness = int.from_bytes(ble.gatts_read(handles[1]), 'little')
            nvs_set.set_i32("bright", brightness)
        elif attr_handle == handles[2]:
            speed = int.from_bytes(ble.gatts_read(handles[2]), 'little')
            nvs_set.set_i32("speed", speed)
        nvs_set.commit()

ble.irq(ble_irq)

services = (
    (SERVICE_UUID, (CHAR_MODE, CHAR_BRIGHT, CHAR_SPEED)),
)
handles = ble.gatts_register_services(services)[0]

# Set defaults
ble.gatts_write(handles[0], struct.pack("B", mode))
ble.gatts_write(handles[1], struct.pack("B", brightness))
ble.gatts_write(handles[2], struct.pack("B", speed))

# Advertise
ble.gap_advertise(100, adv_data=bluetooth.advertising_payload(
    name="C6-LED", services=[SERVICE_UUID]
))

# ====== LED loop ======
def apply_led():
    val = (brightness, 0, 0) if mode == 0 else (0, brightness, 0)
    np[0] = val
    np.write()

while True:
    apply_led()
    time.sleep_ms(1000 // max(1, speed))
