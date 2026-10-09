#!/usr/bin/env python3
"""
bmi330.py - BMI330 IMU over spidev: reset, configure, print accel/gyro/temp.

  python3 bmi330.py                         # /dev/spidev1.0, 10 Hz print, Ctrl-C to stop
  python3 bmi330.py --rate 50 --count 200
  python3 bmi330.py --acc-range 2 --gyr-range 250
  python3 bmi330.py --raw                   # raw counts instead of units

SPI protocol (BMI3xx): read = [0x80|reg, dummy] then 16-bit words LSB first,
address auto-increments; write = [reg, lo, hi], no dummy byte.
Register map is shared with the BMI323 family - check against your datasheet rev.
"""
import argparse
import math
import sys
import time

import spidev

# registers
CHIP_ID, ERR_REG, STATUS = 0x00, 0x01, 0x02
ACC_X = 0x03            # 0x03..0x05 acc XYZ, 0x06..0x08 gyr XYZ, 0x09 temp, 0x0A/0x0B sensor time
ACC_CONF, GYR_CONF = 0x20, 0x21
CMD = 0x7E

BMI330_ID = 0x48
SOFT_RESET = 0xDEAF
INVALID = -32768        # 0x8000: no valid sample yet

ODR_CODES = {12.5: 0x5, 25: 0x6, 50: 0x7, 100: 0x8, 200: 0x9, 400: 0xA, 800: 0xB}
ACC_RANGES = {2: 0, 4: 1, 8: 2, 16: 3}                       # g -> code
GYR_RANGES = {125: 0, 250: 1, 500: 2, 1000: 3, 2000: 4}      # dps -> code
MODE_NORMAL = 4


class BMI330:
    def __init__(self, bus, cs, speed):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, cs)
        self.spi.mode = 0
        self.spi.max_speed_hz = speed

    def close(self):
        self.spi.close()

    def read_words(self, reg, n=1):
        rx = self.spi.xfer2([0x80 | reg, 0x00] + [0x00] * (2 * n))
        d = rx[2:]
        return [d[2 * i] | (d[2 * i + 1] << 8) for i in range(n)]

    def read(self, reg):
        return self.read_words(reg, 1)[0]

    def write(self, reg, value):
        self.spi.xfer2([reg & 0x7F, value & 0xFF, (value >> 8) & 0xFF])

    def dummy_read(self):
        # rising CS edge switches the interface from I2C to SPI after power-up / reset
        self.read(CHIP_ID)

    def soft_reset(self):
        self.write(CMD, SOFT_RESET)
        time.sleep(0.005)
        self.dummy_read()


def s16(v):
    return v - 0x10000 if v & 0x8000 else v


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, default=1)
    p.add_argument("--cs", type=int, default=0)
    p.add_argument("--speed", type=int, default=1_000_000)
    p.add_argument("--odr", type=float, default=100, choices=sorted(ODR_CODES), help="sensor output data rate, Hz")
    p.add_argument("--acc-range", type=int, default=8, choices=sorted(ACC_RANGES), help="g")
    p.add_argument("--gyr-range", type=int, default=2000, choices=sorted(GYR_RANGES), help="deg/s")
    p.add_argument("--rate", type=float, default=10, help="print rate, Hz")
    p.add_argument("--count", type=int, default=0, help="samples to print (0 = until Ctrl-C)")
    p.add_argument("--raw", action="store_true", help="print raw counts")
    a = p.parse_args()

    imu = BMI330(a.bus, a.cs, a.speed)
    try:
        imu.dummy_read()
        cid = imu.read(CHIP_ID)
        print(f"CHIP_ID  0x{cid:04x} -> id 0x{cid & 0xFF:02x}", end="")
        if cid & 0xFF != BMI330_ID:
            print(f"  (expected 0x{BMI330_ID:02x}) - check wiring / bus / CS")
            sys.exit(1)
        print("  BMI330 OK")

        imu.soft_reset()
        print(f"after reset: STATUS 0x{imu.read(STATUS):04x}  ERR_REG 0x{imu.read(ERR_REG):04x}")

        odr = ODR_CODES[a.odr]
        acc_conf = (MODE_NORMAL << 12) | (ACC_RANGES[a.acc_range] << 4) | odr
        gyr_conf = (MODE_NORMAL << 12) | (GYR_RANGES[a.gyr_range] << 4) | odr
        imu.write(ACC_CONF, acc_conf)
        imu.write(GYR_CONF, gyr_conf)
        time.sleep(0.05)   # gyro start-up takes tens of ms

        rb_acc, rb_gyr, err = imu.read(ACC_CONF), imu.read(GYR_CONF), imu.read(ERR_REG)
        print(f"ACC_CONF wrote 0x{acc_conf:04x} read 0x{rb_acc:04x}   "
              f"GYR_CONF wrote 0x{gyr_conf:04x} read 0x{rb_gyr:04x}   ERR_REG 0x{err:04x}")
        if err:
            print("  ERR_REG nonzero - bit 5 = accel config error, bit 6 = gyro config error")
        if rb_acc != acc_conf or rb_gyr != gyr_conf:
            print("  readback mismatch - writes not landing (check MOSI)")

        acc_scale = a.acc_range / 32768.0     # g per LSB
        gyr_scale = a.gyr_range / 32768.0     # dps per LSB
        print(f"\naccel ±{a.acc_range} g, gyro ±{a.gyr_range} dps, ODR {a.odr} Hz; Ctrl-C to stop\n")
        if a.raw:
            print(f"{'ax':>7} {'ay':>7} {'az':>7} | {'gx':>7} {'gy':>7} {'gz':>7} | {'temp':>6} | {'time':>8}")
        else:
            print(f"{'ax g':>7} {'ay g':>7} {'az g':>7} {'|a|':>6} | "
                  f"{'gx dps':>8} {'gy dps':>8} {'gz dps':>8} | {'°C':>6} | {'t s':>9}")

        period = 1.0 / a.rate
        n = 0
        while a.count == 0 or n < a.count:
            t0 = time.monotonic()
            w = imu.read_words(ACC_X, 9)
            ax, ay, az, gx, gy, gz, traw = (s16(v) for v in w[:7])
            stime = (w[7] | (w[8] << 16)) * 39.0625e-6
            if ax == INVALID or gx == INVALID:
                print("  (no valid sample yet)")
            elif a.raw:
                print(f"{ax:7d} {ay:7d} {az:7d} | {gx:7d} {gy:7d} {gz:7d} | {traw:6d} | {w[7] | (w[8] << 16):8d}")
            else:
                fx, fy, fz = ax * acc_scale, ay * acc_scale, az * acc_scale
                mag = math.sqrt(fx * fx + fy * fy + fz * fz)
                temp = traw / 512.0 + 23.0
                print(f"{fx:7.3f} {fy:7.3f} {fz:7.3f} {mag:6.3f} | "
                      f"{gx * gyr_scale:8.2f} {gy * gyr_scale:8.2f} {gz * gyr_scale:8.2f} | "
                      f"{temp:6.1f} | {stime:9.3f}")
            n += 1
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print()
    finally:
        imu.close()


if __name__ == "__main__":
    main()
