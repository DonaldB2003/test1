#!/usr/bin/env python3

import time
import math
import threading
import serial
import spidev
import RPi.GPIO as GPIO
import smbus2
import pynmea2
import Adafruit_DHT

# ─── PIN CONFIG ───────────────────────────────────────────────
LORA_RST = 17
LORA_CS  = 8
LORA_DIO0 = 4

LED_RED   = 23
LED_GREEN = 25
LED_BLUE  = 16

DHT_PIN = 24

# ─── CONSTANTS ────────────────────────────────────────────────
SATELLITE_ID = "CUBESAT_01"
TELEMETRY_INTERVAL = 1.0

# ─── GLOBALS ──────────────────────────────────────────────────
bus = smbus2.SMBus(1)
gps_data = {"lat":0.0,"lon":0.0,"alt_gps":0.0,"satellites":0,"fix":False}
gps_lock = threading.Lock()

# ──────────────────────────────────────────────────────────────
# GPIO SETUP
# ──────────────────────────────────────────────────────────────
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(LORA_RST, GPIO.OUT)
    GPIO.setup(LORA_CS, GPIO.OUT)

    GPIO.setup(LED_RED, GPIO.OUT)
    GPIO.setup(LED_GREEN, GPIO.OUT)
    GPIO.setup(LED_BLUE, GPIO.OUT)

    GPIO.output(LED_RED, GPIO.HIGH)
    print("[BOOT] GPIO ready")

# ──────────────────────────────────────────────────────────────
# WORKING LORA (FROM YOUR TEST CODE)
# ──────────────────────────────────────────────────────────────
spi = spidev.SpiDev()

def lora_w(reg, val):
    GPIO.output(LORA_CS, 0)
    spi.xfer2([reg | 0x80, val])
    GPIO.output(LORA_CS, 1)

def lora_r(reg):
    GPIO.output(LORA_CS, 0)
    v = spi.xfer2([reg & 0x7F, 0])
    GPIO.output(LORA_CS, 1)
    return v[1]

def lora_init():
    spi.open(0, 0)
    spi.max_speed_hz = 5000000
    spi.mode = 0

    GPIO.output(LORA_RST, 0)
    time.sleep(0.01)
    GPIO.output(LORA_RST, 1)
    time.sleep(0.1)

    ver = lora_r(0x42)
    print("[LoRa] Version:", hex(ver))
    if ver != 0x12:
        raise RuntimeError("LoRa not detected!")

    lora_w(0x01, 0x80)  # sleep + LoRa

    frf = int(433e6 / (32e6 / 524288))
    lora_w(0x06, (frf >> 16) & 0xFF)
    lora_w(0x07, (frf >> 8) & 0xFF)
    lora_w(0x08, frf & 0xFF)

    lora_w(0x09, 0x8F)
    lora_w(0x1D, 0x72)
    lora_w(0x1E, 0x74)

    lora_w(0x01, 0x81)
    print("[LoRa] Ready")

def lora_send(msg):
    data = msg.encode()

    lora_w(0x01, 0x81)
    lora_w(0x0E, 0x00)
    lora_w(0x10, 0x00)
    lora_w(0x22, len(data))

    GPIO.output(LORA_CS, 0)
    spi.xfer2([0x80] + list(data))
    GPIO.output(LORA_CS, 1)

    lora_w(0x01, 0x83)

    t = time.time()
    while time.time() - t < 5:
        if lora_r(0x12) & 0x08:
            lora_w(0x12, 0xFF)
            lora_w(0x01, 0x81)
            return True
        time.sleep(0.01)

    return False

# ──────────────────────────────────────────────────────────────
# GPS THREAD
# ──────────────────────────────────────────────────────────────
def gps_thread():
    try:
        ser = serial.Serial('/dev/serial0', 9600, timeout=1)
        while True:
            line = ser.readline().decode(errors='ignore')
            if "$GPGGA" in line:
                try:
                    msg = pynmea2.parse(line)
                    with gps_lock:
                        gps_data["lat"] = float(msg.latitude or 0)
                        gps_data["lon"] = float(msg.longitude or 0)
                        gps_data["alt_gps"] = float(msg.altitude or 0)
                        gps_data["satellites"] = int(msg.num_sats or 0)
                        gps_data["fix"] = msg.gps_qual > 0
                except:
                    pass
    except Exception as e:
        print("[GPS] Error:", e)

# ──────────────────────────────────────────────────────────────
# PACKET BUILDER
# ──────────────────────────────────────────────────────────────
def build_packet(seq):
    with gps_lock:
        g = gps_data.copy()

    return (
        f"SAT:{SATELLITE_ID},SEQ:{seq},"
        f"LAT:{g['lat']},LON:{g['lon']},"
        f"ALT:{g['alt_gps']},SATS:{g['satellites']},"
        f"FIX:{int(g['fix'])}"
    )

# ──────────────────────────────────────────────────────────────
# MAIN LOOP
# ──────────────────────────────────────────────────────────────
def main():
    print("CUBESAT STARTING...")

    setup_gpio()
    lora_init()

    threading.Thread(target=gps_thread, daemon=True).start()

    seq = 0

    while True:
        packet = build_packet(seq)
        print("TX:", packet)

        ok = lora_send(packet)

        if ok:
            GPIO.output(LED_BLUE, 1)
            time.sleep(0.1)
            GPIO.output(LED_BLUE, 0)
            print("TX OK")
        else:
            print("TX FAIL")

        seq += 1
        time.sleep(1)

if __name__ == "__main__":
    main()
