#!/usr/bin/env python3
"""
UV Chamber Controller - iGEM Soil Wildfire Simulation
Runs on Raspberry Pi 5. Control via SSH from your laptop:
    ssh <user>@raspberrypi.local
    python3 chamber.py

Hardware:
    GPIO18 (pin 12) --[220 ohm]--> MOSFET gate (logic-level N-ch)
    MOSFET gate --[10k pulldown]--> GND
    MOSFET drain --> UV strip (-)
    MOSFET source --> common GND
    12V (+) --[fuse]--[switch]--> UV strip (+)
    12V (-) --> common GND

    MCP3008 ADC on SPI0:
        VDD/VREF -> 3.3V, AGND/DGND -> GND
        CLK  -> GPIO11 (pin 23)
        MISO -> GPIO9  (pin 21)
        MOSI -> GPIO10 (pin 19)
        CS   -> GPIO8  (pin 24)
        CH0  -> GUVA OUT (sensor VCC -> 3.3V, GND -> common GND)

    Two Pi Camera Module 3 units on CSI 0 and CSI 1.

One-time setup on the Pi:
    sudo apt update
    sudo apt install python3-lgpio python3-spidev python3-picamera2
    sudo raspi-config    # enable SPI, enable Camera
    sudo reboot

Commands once running:
    on / off                  - turn UV on or off (also stops any active exposure)
    read                      - one sensor reading (UV intensity in mW/cm^2)
    expose <sec>              - timed exposure with continuous sensor logging
    log <ms>                  - start continuous sensor logging (0 stops)
    snap [cam]                - capture single photo from cam 0 or 1 (default 0)
    snap both                 - capture from both cameras simultaneously
    run <sec>                 - full experiment cycle:
                                  before-photo, UV on, log sensor, UV off, after-photo
    status                    - show current state
    help                      - this message
    quit                      - turn UV off and exit
"""

import sys
import os
import time
import threading
import atexit
from datetime import datetime

# ---------- Hardware libraries ----------
try:
    import lgpio
except ImportError:
    sys.exit("Missing lgpio. Install with: sudo apt install python3-lgpio")

try:
    import spidev
except ImportError:
    sys.exit("Missing spidev. Install with: sudo apt install python3-spidev")

try:
    from picamera2 import Picamera2
    CAMERAS_AVAILABLE = True
except ImportError:
    print("WARNING: picamera2 not available. Camera commands disabled.")
    print("  Install with: sudo apt install python3-picamera2")
    CAMERAS_AVAILABLE = False

# ---------- Configuration ----------
UV_PIN          = 18           # BCM, header pin 12
ADC_CHANNEL     = 0            # MCP3008 channel for GUVA
ADC_VREF        = 3.3          # MCP3008 VREF voltage
ADC_MAX         = 1023         # 10-bit ADC
GUVA_V_PER_MW   = 0.1          # GUVA-S12SD: ~0.1V per mW/cm^2 UV-A (approximate)
SENSOR_SAMPLES  = 16           # oversampling for noise reduction
DATA_DIR        = os.path.expanduser("~/uv_chamber_data")

os.makedirs(DATA_DIR, exist_ok=True)

# ---------- GPIO setup ----------
_chip = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(_chip, UV_PIN, 0)   # start LOW (off)

# ---------- SPI / ADC setup ----------
spi = spidev.SpiDev()
spi.open(0, 0)              # bus 0, device 0 (CE0)
spi.max_speed_hz = 1_000_000
spi.mode = 0

def read_adc(channel: int) -> int:
    """Read a single MCP3008 channel, returns 0-1023."""
    if not 0 <= channel <= 7:
        raise ValueError("channel must be 0-7")
    # MCP3008 protocol: start bit, single-ended + channel, then 2 bytes back
    resp = spi.xfer2([1, (8 + channel) << 4, 0])
    return ((resp[1] & 0x03) << 8) | resp[2]

def read_uv() -> dict:
    """Oversampled UV reading. Returns adc, volts, mW/cm^2."""
    total = 0
    for _ in range(SENSOR_SAMPLES):
        total += read_adc(ADC_CHANNEL)
    avg = total / SENSOR_SAMPLES
    volts = (avg / ADC_MAX) * ADC_VREF
    mw = volts / GUVA_V_PER_MW
    return {"adc": avg, "volts": volts, "mw_cm2": mw}

# ---------- Camera setup ----------
cameras = []
if CAMERAS_AVAILABLE:
    try:
        n = len(Picamera2.global_camera_info())
        for i in range(n):
            cam = Picamera2(i)
            cam.configure(cam.create_still_configuration())
            cameras.append(cam)
        print(f"Found {len(cameras)} camera(s).")
    except Exception as e:
        print(f"Camera init failed: {e}")
        cameras = []

def snap(cam_index: int, label: str = "") -> str:
    """Capture a still photo from camera N. Returns saved filename."""
    if not cameras:
        print("No cameras available.")
        return ""
    if cam_index >= len(cameras):
        print(f"Camera {cam_index} not found (have {len(cameras)}).")
        return ""

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"_{label}" if label else ""
    fname = os.path.join(DATA_DIR, f"cam{cam_index}_{ts}{tag}.jpg")

    cam = cameras[cam_index]
    cam.start()
    time.sleep(0.5)             # let auto-exposure settle
    cam.capture_file(fname)
    cam.stop()
    print(f"  saved {fname}")
    return fname

def snap_both(label: str = "") -> list:
    """Capture from both cameras as close to simultaneously as possible."""
    return [snap(i, label) for i in range(len(cameras))]

# ---------- UV control ----------
def uv_on():
    lgpio.gpio_write(_chip, UV_PIN, 1)

def uv_off():
    lgpio.gpio_write(_chip, UV_PIN, 0)

def uv_state() -> int:
    return lgpio.gpio_read(_chip, UV_PIN)

# ---------- Logging thread ----------
class SensorLogger(threading.Thread):
    """Background thread that polls the UV sensor at a fixed interval."""
    def __init__(self, interval_ms: int, csv_path: str = None, label: str = ""):
        super().__init__(daemon=True)
        self.interval = interval_ms / 1000.0
        self.csv_path = csv_path
        self.label = label
        self.stop_flag = threading.Event()
        self.readings = []

    def run(self):
        f = open(self.csv_path, "a") if self.csv_path else None
        if f and f.tell() == 0:
            f.write("timestamp,uv_on,adc,volts,mw_cm2,label\n")
        t0 = time.time()
        next_t = t0
        try:
            while not self.stop_flag.is_set():
                r = read_uv()
                ts = datetime.now().isoformat(timespec="milliseconds")
                line = f"{ts},{uv_state()},{r['adc']:.1f},{r['volts']:.4f},{r['mw_cm2']:.4f},{self.label}"
                self.readings.append(r)
                if f:
                    f.write(line + "\n")
                    f.flush()
                else:
                    print(f"  {line}")
                next_t += self.interval
                sleep_for = next_t - time.time()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_t = time.time()
        finally:
            if f:
                f.close()

    def stop(self):
        self.stop_flag.set()

# ---------- Command handlers ----------
state = {
    "logger": None,
    "expose_thread": None,
}

def cmd_on(_args):
    uv_on()
    print("UV ON")

def cmd_off(_args):
    uv_off()
    if state["expose_thread"] and state["expose_thread"].is_alive():
        print("(stopping active exposure)")
    print("UV OFF")

def cmd_read(_args):
    r = read_uv()
    print(f"  ADC={r['adc']:.1f}  V={r['volts']:.3f}  UV={r['mw_cm2']:.3f} mW/cm^2  uv_pin={uv_state()}")

def cmd_log(args):
    if not args:
        print("Usage: log <ms>   (0 to stop)")
        return
    ms = int(args[0])
    if state["logger"]:
        state["logger"].stop()
        state["logger"] = None
        print("Logging stopped.")
    if ms > 0:
        path = os.path.join(DATA_DIR, f"log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        lg = SensorLogger(ms, csv_path=path)
        lg.start()
        state["logger"] = lg
        print(f"Logging to {path} every {ms} ms")

def cmd_expose(args):
    if not args:
        print("Usage: expose <seconds>")
        return
    sec = float(args[0])
    if sec <= 0:
        print("Duration must be positive")
        return

    path = os.path.join(DATA_DIR, f"expose_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    lg = SensorLogger(1000, csv_path=path, label="expose")
    lg.start()
    state["logger"] = lg

    print(f"Exposing for {sec} seconds. Logging to {path}")
    uv_on()
    try:
        end = time.time() + sec
        while time.time() < end:
            remaining = end - time.time()
            print(f"  {int(sec - remaining)}/{int(sec)} s   ", end="\r")
            time.sleep(min(1.0, remaining))
        print()
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        uv_off()
        lg.stop()
        state["logger"] = None
        print("Exposure complete. UV OFF.")

def cmd_snap(args):
    if not cameras:
        print("No cameras available.")
        return
    if args and args[0].lower() == "both":
        snap_both()
        return
    cam_index = int(args[0]) if args else 0
    snap(cam_index)

def cmd_run(args):
    """Full experiment cycle: before-photo -> UV on with logging -> UV off -> after-photo."""
    if not args:
        print("Usage: run <seconds>")
        return
    sec = float(args[0])
    print(f"=== Experiment cycle, {sec} s exposure ===")
    print("Taking BEFORE photos...")
    snap_both(label="before")
    cmd_expose([str(sec)])
    print("Taking AFTER photos...")
    snap_both(label="after")
    print("=== Cycle complete ===")

def cmd_status(_args):
    print(f"  UV pin:        {uv_state()} ({'ON' if uv_state() else 'OFF'})")
    print(f"  Logger:        {'running' if state['logger'] else 'stopped'}")
    print(f"  Cameras:       {len(cameras)} available")
    print(f"  Data dir:      {DATA_DIR}")
    r = read_uv()
    print(f"  Live sensor:   {r['mw_cm2']:.3f} mW/cm^2")

def cmd_help(_args):
    print(__doc__.split("Commands once running:")[1].split("\n\n")[0])

# ---------- Cleanup ----------
def _cleanup():
    try:
        if state.get("logger"):
            state["logger"].stop()
        uv_off()
        lgpio.gpiochip_close(_chip)
        spi.close()
        for cam in cameras:
            try: cam.close()
            except Exception: pass
    except Exception:
        pass

atexit.register(_cleanup)

# ---------- Command shell ----------
COMMANDS = {
    "on":     cmd_on,
    "off":    cmd_off,
    "read":   cmd_read,
    "log":    cmd_log,
    "expose": cmd_expose,
    "snap":   cmd_snap,
    "run":    cmd_run,
    "status": cmd_status,
    "help":   cmd_help,
    "?":      cmd_help,
}

def shell():
    print()
    print("=" * 50)
    print(" UV Chamber Controller - iGEM Soil Project")
    print("=" * 50)
    cmd_help(None)
    while True:
        try:
            line = input("\nchamber> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]
        if cmd in ("quit", "exit", "q"):
            break
        if cmd in COMMANDS:
            try:
                COMMANDS[cmd](args)
            except Exception as e:
                print(f"Error: {e}")
        else:
            print(f"Unknown command: {cmd}  (type 'help')")
    print("Shutting down...")

# ---------- Entry ----------
if __name__ == "__main__":
    shell()
