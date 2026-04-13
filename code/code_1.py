#!/usr/bin/env python3
"""
CubeSat Flight Software — Minimal
Sensors: BMP280, MPU-6050, DS3231, DHT11, NEO-6M GPS
Radio:   LoRa RA-02 (software SPI)
LEDs:    Red=Power | Green=Running | Blue=TX
"""

import time, math, threading, serial
import RPi.GPIO as GPIO
import smbus2, pynmea2
import board, adafruit_dht

# ─── Pins ─────────────────────────────────────────────────────
NSS  = 8;  RST  = 14; DIO0 = 4
SCK  = 11; MISO = 9;  MOSI = 10
LED_RED = 23; LED_GREEN = 25; LED_BLUE = 16

# ─── Settings ─────────────────────────────────────────────────
SAT_ID      = "CUBESAT_01"
FREQ        = 433
TX_INTERVAL = 1.0
SEA_LEVEL_P = 1013.25

# ─── Globals ──────────────────────────────────────────────────
bus      = smbus2.SMBus(1)
gps      = {"lat":0.0,"lon":0.0,"alt":0.0,"sats":0,"fix":False}
gps_lock = threading.Lock()
running  = True

# ══════════════════════════════════════════════════════════════
# GPIO + LEDs
# ══════════════════════════════════════════════════════════════
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin, mode, state in [
        (NSS,GPIO.OUT,GPIO.HIGH),(RST,GPIO.OUT,GPIO.HIGH),
        (SCK,GPIO.OUT,GPIO.LOW),(MOSI,GPIO.OUT,GPIO.LOW),
        (MISO,GPIO.IN,None),(DIO0,GPIO.IN,None),
        (LED_RED,GPIO.OUT,GPIO.LOW),
        (LED_GREEN,GPIO.OUT,GPIO.LOW),
        (LED_BLUE,GPIO.OUT,GPIO.LOW),
    ]:
        GPIO.setup(pin, mode)
        if state is not None: GPIO.output(pin, state)

def blink(pin, times=1, ms=100):
    def _b():
        for _ in range(times):
            GPIO.output(pin,GPIO.HIGH); time.sleep(ms/1000)
            GPIO.output(pin,GPIO.LOW);  time.sleep(ms/1000)
    threading.Thread(target=_b, daemon=True).start()

def start_leds():
    GPIO.output(LED_RED, GPIO.HIGH)
    def _green():
        while running:
            GPIO.output(LED_GREEN,GPIO.HIGH); time.sleep(0.5)
            GPIO.output(LED_GREEN,GPIO.LOW);  time.sleep(0.5)
    threading.Thread(target=_green, daemon=True).start()

# ══════════════════════════════════════════════════════════════
# SOFTWARE SPI
# ══════════════════════════════════════════════════════════════
def spi_byte(data):
    rx = 0
    for i in range(8):
        GPIO.output(MOSI, bool(data & (0x80>>i)))
        GPIO.output(SCK,  GPIO.HIGH)
        if GPIO.input(MISO): rx |= (0x80>>i)
        GPIO.output(SCK,  GPIO.LOW)
    return rx

def wr(reg, val):
    GPIO.output(NSS,GPIO.LOW)
    spi_byte(reg|0x80); spi_byte(val)
    GPIO.output(NSS,GPIO.HIGH)

def rd(reg):
    GPIO.output(NSS,GPIO.LOW)
    spi_byte(reg&0x7F); v=spi_byte(0)
    GPIO.output(NSS,GPIO.HIGH)
    return v

# ══════════════════════════════════════════════════════════════
# LORA
# ══════════════════════════════════════════════════════════════
def lora_init():
    GPIO.output(RST,GPIO.LOW);  time.sleep(0.01)
    GPIO.output(RST,GPIO.HIGH); time.sleep(0.1)
    v = rd(0x42)
    if v != 0x12: raise RuntimeError(f"LoRa not found: {hex(v)}")
    wr(0x01,0x80)               # sleep + LoRa
    frf = int(FREQ*1e6/(32e6/524288))
    wr(0x06,(frf>>16)&0xFF); wr(0x07,(frf>>8)&0xFF); wr(0x08,frf&0xFF)
    wr(0x0E,0x00); wr(0x0F,0x00)
    wr(0x09,0x8F)               # max power
    wr(0x1D,0x72)               # BW=125k CR=4/5
    wr(0x1E,0x74)               # SF=7 CRC on
    wr(0x26,0x04)               # AGC on
    wr(0x01,0x81)               # standby
    print("[LoRa] Ready")

def lora_send(msg):
    p = msg.encode()
    wr(0x01,0x81); wr(0x10,0x00); wr(0x22,len(p))
    GPIO.output(NSS,GPIO.LOW)
    spi_byte(0x80)
    for b in p: spi_byte(b)
    GPIO.output(NSS,GPIO.HIGH)
    wr(0x01,0x83)               # TX mode
    t = time.time()
    while time.time()-t < 5:
        if rd(0x12)&0x08:
            wr(0x12,0xFF); wr(0x01,0x81)
            return True
        time.sleep(0.01)
    return False

# ══════════════════════════════════════════════════════════════
# DS3231 RTC
# ══════════════════════════════════════════════════════════════
def get_time():
    d = bus.read_i2c_block_data(0x68, 0x00, 3)
    b = lambda v: (v>>4)*10+(v&0x0F)
    return f"{b(d[2]&0x3F):02d}:{b(d[1]):02d}:{b(d[0]&0x7F):02d}"

# ══════════════════════════════════════════════════════════════
# BMP280
# ══════════════════════════════════════════════════════════════
class BMP280:
    def __init__(self):
        c = bus.read_i2c_block_data(0x76, 0x88, 24)
        u = lambda i: (c[i+1]<<8)|c[i]
        s = lambda i: (v:=u(i)) and v-65536 if v>32767 else v
        self.T=[u(0),s(2),s(4)]
        self.P=[u(6)]+[s(i) for i in range(8,24,2)]
        bus.write_byte_data(0x76,0xF4,0x27)
        bus.write_byte_data(0x76,0xF5,0xA0)
        time.sleep(0.1)

    def read(self):
        d  = bus.read_i2c_block_data(0x76,0xF7,6)
        aP = (d[0]<<12)|(d[1]<<4)|(d[2]>>4)
        aT = (d[3]<<12)|(d[4]<<4)|(d[5]>>4)
        v1 = ((aT/16384)-(self.T[0]/1024))*self.T[1]
        v2 = ((aT/131072)-(self.T[0]/8192))**2*self.T[2]
        tf = v1+v2; temp=tf/5120
        v1 = tf/2-64000
        v2 = v1*v1*self.P[6]/32768+v1*self.P[5]*2
        v2 = v2/4+self.P[4]*65536
        v1 = (self.P[3]*v1*v1/524288+self.P[2]*v1)/524288
        v1 = (1+v1/32768)*self.P[1]
        if not v1: return 0,0,0
        p  = ((1048576-aP)-v2/4096)*6250/v1/100
        a  = 44330*(1-(p/SEA_LEVEL_P)**(1/5.255))
        return round(temp,2), round(p,2), round(a,2)

# ══════════════════════════════════════════════════════════════
# MPU-6050
# ══════════════════════════════════════════════════════════════
class MPU6050:
    def __init__(self):
        bus.write_byte_data(0x69,0x6B,0x00); time.sleep(0.1)

    def _r(self, reg):
        h=bus.read_byte_data(0x69,reg)
        l=bus.read_byte_data(0x69,reg+1)
        v=(h<<8)+l
        return v-65536 if v>=0x8000 else v

    def read(self):
        ax=self._r(0x3B)/16384; ay=self._r(0x3D)/16384; az=self._r(0x3F)/16384
        gx=self._r(0x43)/131;   gy=self._r(0x45)/131;   gz=self._r(0x47)/131
        return {
            "ax":round(ax,3),"ay":round(ay,3),"az":round(az,3),
            "gx":round(gx,2),"gy":round(gy,2),"gz":round(gz,2),
            "mag":round(math.sqrt(ax**2+ay**2+az**2),3)
        }

# ══════════════════════════════════════════════════════════════
# DHT11
# ══════════════════════════════════════════════════════════════
class DHT11Sensor:
    def __init__(self):
        self.d = adafruit_dht.DHT11(board.D24)

    def read(self):
        try:
            t = self.d.temperature
            h = self.d.humidity
            return (round(t,1), round(h,1)) if t and h else (0.0, 0.0)
        except RuntimeError:
            return 0.0, 0.0

# ══════════════════════════════════════════════════════════════
# GPS THREAD
# ══════════════════════════════════════════════════════════════
def gps_thread():
    try:
        ser = serial.Serial('/dev/serial0', 9600, timeout=1)
        while True:
            try:
                line = ser.readline().decode('ascii',errors='replace').strip()
                if line.startswith(('$GPGGA','$GNGGA')):
                    m = pynmea2.parse(line)
                    with gps_lock:
                        gps.update({
                            "lat":  round(float(m.latitude),6)  if m.latitude  else 0.0,
                            "lon":  round(float(m.longitude),6) if m.longitude else 0.0,
                            "alt":  round(float(m.altitude),2)  if m.altitude  else 0.0,
                            "sats": int(m.num_sats) if m.num_sats else 0,
                            "fix":  m.gps_qual > 0  if m.gps_qual else False,
                        })
            except: continue
    except Exception as e:
        print(f"[GPS] {e}")

# ══════════════════════════════════════════════════════════════
# PACKET BUILDER
# ══════════════════════════════════════════════════════════════
def build_packet(ts, bmp, imu, dht, seq):
    with gps_lock: g = gps.copy()
    t,p,a = bmp
    dt,dh = dht
    return (
        f"SAT:{SAT_ID},SEQ:{seq},TIME:{ts},"
        f"LAT:{g['lat']},LON:{g['lon']},"
        f"ALT_GPS:{g['alt']},ALT_BARO:{a},"
        f"SATS:{g['sats']},FIX:{int(g['fix'])},"
        f"PRESS:{p},TEMP_BMP:{t},"
        f"TEMP_DHT:{dt},HUM:{dh},"
        f"AX:{imu['ax']},AY:{imu['ay']},AZ:{imu['az']},"
        f"GX:{imu['gx']},GY:{imu['gy']},GZ:{imu['gz']},"
        f"MAG:{imu['mag']}"
    )

# ══════════════════════════════════════════════════════════════
# MAIN
# ══════════════════════════════════════════════════════════════
def main():
    global running
    print("=" * 45)
    print("   CUBESAT BOOTING...")
    print("=" * 45)

    setup_gpio()
    start_leds()

    bmp = BMP280()
    mpu = MPU6050()
    dht = DHT11Sensor()
    lora_init()
    threading.Thread(target=gps_thread, daemon=True).start()

    print(f"[{get_time()}] All systems GO")

    seq      = 0
    alt_buf  = []

    try:
        while True:
            t0 = time.time()
            ts = get_time()

            # Read sensors
            bmp_data = bmp.read()
            imu_data = mpu.read()
            dht_data = dht.read()

            # Smooth altitude
            alt_buf = (alt_buf + [bmp_data[2]])[-5:]
            s_alt   = round(sum(alt_buf)/len(alt_buf), 2)

            # Build and send packet
            pkt = build_packet(ts, bmp_data, imu_data, dht_data, seq)
            ok  = lora_send(pkt)

            blink(LED_BLUE, 1 if ok else 3)

            print(f"[{ts}] TX{'OK' if ok else 'FAIL'} "
                  f"SEQ:{seq} ALT:{s_alt}m "
                  f"T:{bmp_data[0]}C "
                  f"GPS:{'FIX' if gps['fix'] else 'NOFIX'}")

            seq += 1
            time.sleep(max(0, TX_INTERVAL-(time.time()-t0)))

    except KeyboardInterrupt:
        print("\nShutdown")
    finally:
        running = False
        dht.d.exit()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
