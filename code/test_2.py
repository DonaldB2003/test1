#!/usr/bin/env python3
"""
CubeSat Main Flight Software
LEDs: RED = Power ON | GREEN BLINK = Code Running | BLUE BLINK = Data Transmitting
RTC: DS3231 for HH:MM:SS timestamps on packets and console
"""

import time
import math
import threading
import serial
import spidev
import RPi.GPIO as GPIO
import smbus2
import pynmea2
import Adafruit_DHT

# ─── PIN DEFINITIONS ──────────────────────────────────────────
LORA_RST        = 17
LORA_CS         = 8
LORA_DIO0       = 4
MOSFET_GATE     = 18
DHT_PIN         = 24
LED_RED         = 23
LED_GREEN       = 25
LED_BLUE        = 16

# ─── CONSTANTS ────────────────────────────────────────────────
ANTENNA_DEPLOY_ALTITUDE  = 100.0
BURN_WIRE_PULSE_DURATION = 3.0
SEA_LEVEL_PRESSURE       = 1013.25
I2C_BUS                  = 1
BMP280_ADDR              = 0x76
MPU6050_ADDR             = 0x68
DS3231_ADDR              = 0x68   # DS3231 default I2C address
LORA_FREQUENCY           = 433
SATELLITE_ID             = "CUBESAT_01"
TELEMETRY_INTERVAL       = 1.0

# ─── GLOBAL STATE ─────────────────────────────────────────────
antenna_deployed = False
bus              = smbus2.SMBus(I2C_BUS)
gps_data         = {"lat": 0.0, "lon": 0.0, "alt_gps": 0.0, "satellites": 0, "fix": False}
gps_lock         = threading.Lock()

# ══════════════════════════════════════════════════════════════
# DS3231 REAL-TIME CLOCK
# ══════════════════════════════════════════════════════════════
class DS3231:
    """
    DS3231 RTC over I2C.
    NOTE: DS3231 shares 0x68 address with MPU-6050.
          Use separate I2C bus or hardware address jumper if conflict occurs.
          Alternatively use smbus2 with i2c_bus=3 for DS3231 if needed.
    """
    def __init__(self, bus_obj, addr=DS3231_ADDR):
        self.bus  = bus_obj
        self.addr = addr
        # Verify by reading seconds register — should return 0x00–0x59 BCD
        try:
            self.bus.read_byte_data(self.addr, 0x00)
            print("[DS3231] Initialized")
        except Exception as e:
            raise RuntimeError(f"[DS3231] Not found: {e}")

    def _bcd_to_dec(self, val):
        """Convert BCD byte to decimal."""
        return (val >> 4) * 10 + (val & 0x0F)

    def _dec_to_bcd(self, val):
        """Convert decimal to BCD byte."""
        return ((val // 10) << 4) | (val % 10)

    def get_time(self):
        """Returns current time as HH:MM:SS string."""
        data = self.bus.read_i2c_block_data(self.addr, 0x00, 3)
        seconds = self._bcd_to_dec(data[0] & 0x7F)  # mask out oscillator stop bit
        minutes = self._bcd_to_dec(data[1])
        hours   = self._bcd_to_dec(data[2] & 0x3F)  # mask out 12/24 hr bit
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"

    def set_time(self, hours, minutes, seconds):
        """Set the time on DS3231."""
        self.bus.write_byte_data(self.addr, 0x00, self._dec_to_bcd(seconds))
        self.bus.write_byte_data(self.addr, 0x01, self._dec_to_bcd(minutes))
        self.bus.write_byte_data(self.addr, 0x02, self._dec_to_bcd(hours))
        print(f"[DS3231] Time set to {hours:02d}:{minutes:02d}:{seconds:02d}")

# ══════════════════════════════════════════════════════════════
# LED MANAGER
# ══════════════════════════════════════════════════════════════
class LEDManager:
    def __init__(self):
        GPIO.output(LED_RED, GPIO.HIGH)
        self._running     = True
        self._green_thread = threading.Thread(target=self._green_blink_loop, daemon=True)
        self._green_thread.start()

    def _green_blink_loop(self):
        while self._running:
            GPIO.output(LED_GREEN, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(LED_GREEN, GPIO.LOW)
            time.sleep(0.5)

    def tx_blink(self):
        def _b():
            GPIO.output(LED_BLUE, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(LED_BLUE, GPIO.LOW)
        threading.Thread(target=_b, daemon=True).start()

    def tx_fail_blink(self):
        def _b():
            for _ in range(3):
                GPIO.output(LED_BLUE, GPIO.HIGH)
                time.sleep(0.06)
                GPIO.output(LED_BLUE, GPIO.LOW)
                time.sleep(0.06)
        threading.Thread(target=_b, daemon=True).start()

    def stop(self):
        self._running = False
        GPIO.output(LED_RED,   GPIO.LOW)
        GPIO.output(LED_GREEN, GPIO.LOW)
        GPIO.output(LED_BLUE,  GPIO.LOW)

# ══════════════════════════════════════════════════════════════
# GPIO SETUP
# ══════════════════════════════════════════════════════════════
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LORA_RST,    GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LORA_CS,     GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(LORA_DIO0,   GPIO.IN)
    GPIO.setup(MOSFET_GATE, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LED_RED,     GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LED_GREEN,   GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(LED_BLUE,    GPIO.OUT, initial=GPIO.LOW)
    print("[BOOT] GPIO initialized")

# ══════════════════════════════════════════════════════════════
# BMP280
# ══════════════════════════════════════════════════════════════
class BMP280:
    def __init__(self):
        self.addr = BMP280_ADDR
        self._load_calibration()
        bus.write_byte_data(self.addr, 0xF4, 0x27)
        bus.write_byte_data(self.addr, 0xF5, 0xA0)
        time.sleep(0.1)
        print("[BMP280] Initialized")

    def _load_calibration(self):
        cal = bus.read_i2c_block_data(self.addr, 0x88, 24)
        self.dig_T1 = (cal[1] << 8) | cal[0]
        self.dig_T2 = self._s((cal[3]  << 8) | cal[2])
        self.dig_T3 = self._s((cal[5]  << 8) | cal[4])
        self.dig_P1 = (cal[7]  << 8) | cal[6]
        self.dig_P2 = self._s((cal[9]  << 8) | cal[8])
        self.dig_P3 = self._s((cal[11] << 8) | cal[10])
        self.dig_P4 = self._s((cal[13] << 8) | cal[12])
        self.dig_P5 = self._s((cal[15] << 8) | cal[14])
        self.dig_P6 = self._s((cal[17] << 8) | cal[16])
        self.dig_P7 = self._s((cal[19] << 8) | cal[18])
        self.dig_P8 = self._s((cal[21] << 8) | cal[20])
        self.dig_P9 = self._s((cal[23] << 8) | cal[22])

    def _s(self, v):
        return v - 65536 if v > 32767 else v

    def read(self):
        data  = bus.read_i2c_block_data(self.addr, 0xF7, 6)
        adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_T = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        var1  = ((adc_T / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
        var2  = ((adc_T / 131072.0) - (self.dig_T1 / 8192.0)) ** 2 * self.dig_T3
        t_fine      = var1 + var2
        temperature = t_fine / 5120.0
        var1 = t_fine / 2.0 - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 = var2 + var1 * self.dig_P5 * 2.0
        var2 = var2 / 4.0 + self.dig_P4 * 65536.0
        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1
        if var1 == 0:
            return temperature, 0.0, 0.0
        pressure = ((1048576.0 - adc_P) - var2 / 4096.0) * 6250.0 / var1
        pressure = pressure / 100.0
        altitude = 44330.0 * (1.0 - (pressure / SEA_LEVEL_PRESSURE) ** (1.0 / 5.255))
        return round(temperature, 2), round(pressure, 2), round(altitude, 2)

# ══════════════════════════════════════════════════════════════
# MPU-6050
# ══════════════════════════════════════════════════════════════
class MPU6050:
    def __init__(self):
        bus.write_byte_data(MPU6050_ADDR, 0x6B, 0x00)
        time.sleep(0.1)
        print("[MPU6050] Initialized")

    def _rw(self, reg):
        h = bus.read_byte_data(MPU6050_ADDR, reg)
        l = bus.read_byte_data(MPU6050_ADDR, reg + 1)
        v = (h << 8) + l
        return v - 65536 if v >= 0x8000 else v

    def read(self):
        ax = round(self._rw(0x3B) / 16384.0, 3)
        ay = round(self._rw(0x3D) / 16384.0, 3)
        az = round(self._rw(0x3F) / 16384.0, 3)
        gx = round(self._rw(0x43) / 131.0, 2)
        gy = round(self._rw(0x45) / 131.0, 2)
        gz = round(self._rw(0x47) / 131.0, 2)
        return {
            "accel_x": ax, "accel_y": ay, "accel_z": az,
            "gyro_x":  gx, "gyro_y":  gy, "gyro_z":  gz,
            "accel_mag": round(math.sqrt(ax**2 + ay**2 + az**2), 3)
        }

# ══════════════════════════════════════════════════════════════
# DHT22
# ══════════════════════════════════════════════════════════════
class DHT22Sensor:
    def __init__(self):
        self.sensor = Adafruit_DHT.DHT22
        self.pin    = DHT_PIN
        print("[DHT22] Initialized")

    def read(self):
        hum, temp = Adafruit_DHT.read_retry(self.sensor, self.pin)
        if hum is None or temp is None:
            return None, None
        return round(temp, 1), round(hum, 1)

# ══════════════════════════════════════════════════════════════
# NEO-6M GPS THREAD
# ══════════════════════════════════════════════════════════════
def gps_thread():
    try:
        ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)
        print("[GPS] Serial port opened")
        while True:
            try:
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    msg = pynmea2.parse(line)
                    with gps_lock:
                        gps_data["lat"]        = round(float(msg.latitude),  6) if msg.latitude  else 0.0
                        gps_data["lon"]        = round(float(msg.longitude), 6) if msg.longitude else 0.0
                        gps_data["alt_gps"]    = round(float(msg.altitude),  2) if msg.altitude  else 0.0
                        gps_data["satellites"] = int(msg.num_sats)               if msg.num_sats  else 0
                        gps_data["fix"]        = msg.gps_qual > 0                if msg.gps_qual  else False
            except Exception:
                continue
    except serial.SerialException as e:
        print(f"[GPS] Serial error: {e}")

# ══════════════════════════════════════════════════════════════
# LORA RA-02
# ══════════════════════════════════════════════════════════════
class LoRaRA02:
    REG_FIFO          = 0x00
    REG_OP_MODE       = 0x01
    REG_FRF_MSB       = 0x06
    REG_FRF_MID       = 0x07
    REG_FRF_LSB       = 0x08
    REG_PA_CONFIG     = 0x09
    REG_FIFO_TX_BASE  = 0x0E
    REG_FIFO_RX_BASE  = 0x0F
    REG_FIFO_ADDR     = 0x10
    REG_IRQ_FLAGS     = 0x12
    REG_RX_NB_BYTES   = 0x13
    REG_FIFO_RX_BYTE  = 0x25
    REG_MODEM_CONFIG1 = 0x1D
    REG_MODEM_CONFIG2 = 0x1E
    REG_PAYLOAD_LEN   = 0x22
    REG_VERSION       = 0x42
    MODE_LONG_RANGE   = 0x80
    MODE_SLEEP        = 0x00
    MODE_STDBY        = 0x01
    MODE_TX           = 0x03
    MODE_RX_CONT      = 0x05

    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 5000000
        self.spi.mode = 0b00
        self._reset()
        version = self._read(self.REG_VERSION)
        if version != 0x12:
            raise RuntimeError(f"[LoRa] RA-02 not found! Version: {hex(version)}")
        self._write(self.REG_OP_MODE,       self.MODE_LONG_RANGE | self.MODE_SLEEP)
        self._set_frequency(LORA_FREQUENCY)
        self._write(self.REG_FIFO_TX_BASE,  0x00)
        self._write(self.REG_FIFO_RX_BASE,  0x00)
        self._write(self.REG_PA_CONFIG,     0x8F)
        self._write(self.REG_MODEM_CONFIG1, 0x72)
        self._write(self.REG_MODEM_CONFIG2, 0x74)
        self._write(self.REG_OP_MODE,       self.MODE_LONG_RANGE | self.MODE_STDBY)
        print(f"[LoRa] Ready. Version: {hex(version)}")

    def _reset(self):
        GPIO.output(LORA_RST, GPIO.LOW);  time.sleep(0.01)
        GPIO.output(LORA_RST, GPIO.HIGH); time.sleep(0.1)

    def _write(self, reg, val):
        GPIO.output(LORA_CS, GPIO.LOW)
        self.spi.xfer2([reg | 0x80, val])
        GPIO.output(LORA_CS, GPIO.HIGH)

    def _read(self, reg):
        GPIO.output(LORA_CS, GPIO.LOW)
        r = self.spi.xfer2([reg & 0x7F, 0x00])
        GPIO.output(LORA_CS, GPIO.HIGH)
        return r[1]

    def _set_frequency(self, freq_mhz):
        frf = int((freq_mhz * 1e6) / (32e6 / 524288))
        self._write(self.REG_FRF_MSB, (frf >> 16) & 0xFF)
        self._write(self.REG_FRF_MID, (frf >> 8)  & 0xFF)
        self._write(self.REG_FRF_LSB,  frf        & 0xFF)

    def send(self, message: str):
        payload = message.encode('utf-8')
        self._write(self.REG_OP_MODE,     self.MODE_LONG_RANGE | self.MODE_STDBY)
        self._write(self.REG_FIFO_ADDR,   0x00)
        self._write(self.REG_PAYLOAD_LEN, len(payload))
        GPIO.output(LORA_CS, GPIO.LOW)
        self.spi.xfer2([self.REG_FIFO | 0x80] + list(payload))
        GPIO.output(LORA_CS, GPIO.HIGH)
        self._write(self.REG_OP_MODE, self.MODE_LONG_RANGE | self.MODE_TX)
        timeout = time.time() + 5.0
        while time.time() < timeout:
            if self._read(self.REG_IRQ_FLAGS) & 0x08:
                self._write(self.REG_IRQ_FLAGS, 0xFF)
                self._write(self.REG_OP_MODE, self.MODE_LONG_RANGE | self.MODE_STDBY)
                return True
            time.sleep(0.01)
        return False

    def receive_check(self):
        self._write(self.REG_OP_MODE, self.MODE_LONG_RANGE | self.MODE_RX_CONT)
        if self._read(self.REG_IRQ_FLAGS) & 0x40:
            self._write(self.REG_IRQ_FLAGS, 0xFF)
            length = self._read(self.REG_RX_NB_BYTES)
            ptr    = self._read(self.REG_FIFO_RX_BYTE)
            self._write(self.REG_FIFO_ADDR, ptr)
            GPIO.output(LORA_CS, GPIO.LOW)
            data = self.spi.xfer2([self.REG_FIFO & 0x7F] + [0x00] * length)
            GPIO.output(LORA_CS, GPIO.HIGH)
            return bytes(data[1:]).decode('utf-8', errors='replace')
        return None

    def cleanup(self):
        self.spi.close()

# ══════════════════════════════════════════════════════════════
# ANTENNA DEPLOYMENT
# ══════════════════════════════════════════════════════════════
def deploy_antenna(rtc, reason="altitude_trigger"):
    global antenna_deployed
    if antenna_deployed:
        return
    ts = rtc.get_time()
    print(f"[{ts}][ANTENNA] Deploying! Reason: {reason}")
    GPIO.output(MOSFET_GATE, GPIO.HIGH)
    time.sleep(BURN_WIRE_PULSE_DURATION)
    GPIO.output(MOSFET_GATE, GPIO.LOW)
    antenna_deployed = True
    ts = rtc.get_time()
    print(f"[{ts}][ANTENNA] Deployment complete. MOSFET OFF.")

# ══════════════════════════════════════════════════════════════
# TELEMETRY PACKET BUILDER
# ══════════════════════════════════════════════════════════════
def build_packet(timestamp, bmp_temp, pressure, baro_alt, imu, dht_temp, dht_hum, seq):
    with gps_lock:
        g = gps_data.copy()
    return (
        f"SAT:{SATELLITE_ID},SEQ:{seq},TIME:{timestamp},"
        f"LAT:{g['lat']},LON:{g['lon']},"
        f"ALT_GPS:{g['alt_gps']},ALT_BARO:{baro_alt},"
        f"SATS:{g['satellites']},FIX:{int(g['fix'])},"
        f"PRESS:{pressure},TEMP_BMP:{bmp_temp},"
        f"TEMP_DHT:{dht_temp},HUM:{dht_hum},"
        f"AX:{imu['accel_x']},AY:{imu['accel_y']},AZ:{imu['accel_z']},"
        f"GX:{imu['gyro_x']},GY:{imu['gyro_y']},GZ:{imu['gyro_z']},"
        f"AMAG:{imu['accel_mag']},ANT:{int(antenna_deployed)}"
    )

# ══════════════════════════════════════════════════════════════
# MAIN FLIGHT LOOP
# ══════════════════════════════════════════════════════════════
def main():
    print("=" * 50)
    print("   CUBESAT FLIGHT SOFTWARE BOOTING...")
    print("=" * 50)

    setup_gpio()
    led = LEDManager()

    try:
        bmp  = BMP280()
        mpu  = MPU6050()
        dht  = DHT22Sensor()
        rtc  = DS3231(bus)       # ← DS3231 initialized here
        lora = LoRaRA02()
    except Exception as e:
        print(f"[BOOT] FATAL: {e}")
        led.stop()
        GPIO.cleanup()
        return

    threading.Thread(target=gps_thread, daemon=True).start()

    boot_time = rtc.get_time()
    print(f"[{boot_time}][MAIN] All systems GO. Entering flight loop.")
    print("=" * 50)

    seq         = 0
    alt_samples = []

    try:
        while True:
            loop_start = time.time()

            # ── Get timestamp first ───────────────────────────
            timestamp = rtc.get_time()

            # ── Read sensors ──────────────────────────────────
            try:
                bmp_temp, pressure, baro_alt = bmp.read()
            except Exception as e:
                print(f"[{timestamp}][BMP280] Error: {e}")
                bmp_temp, pressure, baro_alt = 0.0, 0.0, 0.0

            try:
                imu = mpu.read()
            except Exception as e:
                print(f"[{timestamp}][MPU6050] Error: {e}")
                imu = {k: 0 for k in ["accel_x","accel_y","accel_z",
                                       "gyro_x","gyro_y","gyro_z","accel_mag"]}
            try:
                dht_temp, dht_hum = dht.read()
                if dht_temp is None:
                    raise ValueError("No data")
            except Exception as e:
                print(f"[{timestamp}][DHT22] Error: {e}")
                dht_temp, dht_hum = 0.0, 0.0

            # ── Altitude smoothing ────────────────────────────
            alt_samples.append(baro_alt)
            if len(alt_samples) > 5:
                alt_samples.pop(0)
            smoothed_alt = sum(alt_samples) / len(alt_samples)

            # ── Antenna deploy check ──────────────────────────
            if not antenna_deployed and smoothed_alt >= ANTENNA_DEPLOY_ALTITUDE:
                deploy_antenna(rtc, reason=f"altitude_{smoothed_alt:.1f}m")

            # ── Build & transmit packet ───────────────────────
            packet  = build_packet(timestamp, bmp_temp, pressure, baro_alt,
                                   imu, dht_temp, dht_hum, seq)
            success = lora.send(packet)

            if success:
                led.tx_blink()
            else:
                led.tx_fail_blink()

            print(f"[{timestamp}][TX#{seq:04d}] {'OK' if success else 'FAIL'} | "
                  f"Alt:{smoothed_alt:.1f}m | T:{bmp_temp}°C | "
                  f"GPS:{'FIX' if gps_data['fix'] else 'NO FIX'} | "
                  f"ANT:{'DEPLOYED' if antenna_deployed else 'STOWED'}")

            # ── Check for ground commands ─────────────────────
            incoming = lora.receive_check()
            if incoming:
                print(f"[{timestamp}][RX] Command: {incoming}")
                if "CMD:DEPLOY_ANT" in incoming and not antenna_deployed:
                    deploy_antenna(rtc, reason="ground_command")

            seq += 1
            time.sleep(max(0, TELEMETRY_INTERVAL - (time.time() - loop_start)))

    except KeyboardInterrupt:
        print(f"\n[{rtc.get_time()}][MAIN] Shutdown.")
    finally:
        led.stop()
        lora.cleanup()
        GPIO.cleanup()
        print("[MAIN] Cleanup done.")

if __name__ == "__main__":
    main()
