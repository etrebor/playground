#!/usr/bin/env python3
"""
bmm350.py - BMM350 magnetometer over I2C: reset, OTP compensation, print field/temp.

  python3 bmm350.py                         # auto-find bus/address, poll at 10 Hz
  python3 bmm350.py --bus 2 --addr 0x14
  python3 bmm350.py --odr 50 --avg 4 --count 100
  python3 bmm350.py --int                   # data-ready interrupt on GPIO1_IO27
  python3 bmm350.py --raw

I2C protocol (BMM350): write [reg], then read N+2 bytes - the FIRST TWO
bytes are dummy. Multi-byte values are little-endian, address auto-increments.
Writes are plain [reg, value].
Register map / compensation follow Bosch's BMM350 SensorAPI - check against
your datasheet revision if values look wrong.
"""
import argparse
import glob
import math
import os
import re
import sys
import time

from smbus2 import SMBus, i2c_msg

ADDRS = (0x14, 0x15)          # ADSEL low / high
CHIP_ID_VAL = 0x33

R_CHIP_ID, R_REV_ID, R_ERR = 0x00, 0x01, 0x02
R_AGGR, R_AXIS_EN, R_PMU_CMD, R_PMU_ST0 = 0x04, 0x05, 0x06, 0x07
R_INT_CTRL, R_INT_STATUS = 0x2E, 0x30
R_MAG = 0x31                  # X,Y,Z,T: 24-bit each, 0x31..0x3C
R_OTP_CMD, R_OTP_MSB, R_OTP_LSB, R_OTP_STATUS = 0x50, 0x52, 0x53, 0x55
R_CMD = 0x7E

PMU_SUS, PMU_NM, PMU_UPD_OAE, PMU_FGR, PMU_BR = 0x00, 0x01, 0x02, 0x05, 0x07
SOFT_RESET = 0xB6
OTP_READ, OTP_PWR_OFF = 0x20, 0x80

ODR = {400: 0x2, 200: 0x3, 100: 0x4, 50: 0x5, 25: 0x6, 12.5: 0x7, 6.25: 0x8, 3.125: 0x9, 1.5625: 0xA}
AVG = {1: 0, 2: 1, 4: 2, 8: 3}
MAX_AVG = {400: 1, 200: 2, 100: 4}     # ODR above 50 Hz limits averaging

# LSB -> uT / degC (SensorAPI constants)
_POWER = 1000000.0 / 1048576.0
_ADC_GAIN, _LUT_GAIN = 1 / 1.5, 0.714607238769531
LSB_XY = _POWER / (14.55 * 19.46 * _ADC_GAIN * _LUT_GAIN)
LSB_Z = _POWER / (9.0 * 31.0 * _ADC_GAIN * _LUT_GAIN)
LSB_T = 1 / (0.00204 * _ADC_GAIN * _LUT_GAIN * 1048576)

IMX8MP_GPIO_BANKS = {"30200000": 1, "30210000": 2, "30220000": 3, "30230000": 4, "30240000": 5}


def fix_sign(v, bits):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


def i2c_bound(bus, addr):
    drv = f"/sys/bus/i2c/devices/{bus}-{addr:04x}/driver"
    return os.path.basename(os.readlink(drv)) if os.path.exists(drv) else None


class BMM350:
    def __init__(self, bus, addr):
        self.busnum, self.addr = bus, addr
        self.bus = SMBus(bus)
        self.comp = None

    def close(self):
        self.bus.close()

    def read(self, reg, n=1):
        w, r = i2c_msg.write(self.addr, [reg]), i2c_msg.read(self.addr, n + 2)
        self.bus.i2c_rdwr(w, r)
        return list(r)[2:]                     # drop 2 dummy bytes

    def read8(self, reg):
        return self.read(reg, 1)[0]

    def write(self, reg, val):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, [reg, val & 0xFF]))

    # ---- OTP / compensation
    def read_otp(self):
        words = []
        for a in range(32):
            self.write(R_OTP_CMD, OTP_READ | a)
            for _ in range(100):
                time.sleep(0.0003)
                st = self.read8(R_OTP_STATUS)
                if st & 0x01:
                    break
            else:
                raise RuntimeError(f"OTP read timeout at word {a}")
            if st & 0xE0:
                raise RuntimeError(f"OTP error, status 0x{st:02x} at word {a}")
            words.append((self.read8(R_OTP_MSB) << 8) | self.read8(R_OTP_LSB))
        return words

    @staticmethod
    def build_comp(o):
        c = {}
        c["off"] = [fix_sign(o[0x0E] & 0x0FFF, 12),
                    fix_sign(((o[0x0E] & 0xF000) >> 4) + (o[0x0F] & 0x00FF), 12),
                    fix_sign((o[0x0F] & 0x0F00) + (o[0x10] & 0x00FF), 12)]
        c["t_off"] = fix_sign(o[0x0D] & 0xFF, 8) / 5.0
        c["sens"] = [fix_sign((o[0x10] & 0xFF00) >> 8, 8) / 256.0,
                     fix_sign(o[0x11] & 0xFF, 8) / 256.0 + 0.01,
                     fix_sign((o[0x11] & 0xFF00) >> 8, 8) / 256.0]
        c["t_sens"] = fix_sign((o[0x0D] & 0xFF00) >> 8, 8) / 512.0
        c["tco"] = [fix_sign(o[i] & 0xFF, 8) / 32.0 for i in (0x12, 0x13, 0x14)]
        c["tcs"] = [fix_sign((o[i] & 0xFF00) >> 8, 8) / 16384.0 for i in (0x12, 0x13, 0x14)]
        c["tcs"][2] -= 0.0001
        c["t0"] = fix_sign(o[0x18], 16) / 512.0 + 23.0
        c["cxy"] = fix_sign(o[0x15] & 0xFF, 8) / 800.0
        c["cyx"] = fix_sign((o[0x15] & 0xFF00) >> 8, 8) / 800.0
        c["czx"] = fix_sign(o[0x16] & 0xFF, 8) / 800.0
        c["czy"] = fix_sign((o[0x16] & 0xFF00) >> 8, 8) / 800.0
        return c

    # ---- setup
    def soft_reset(self):
        self.write(R_CMD, SOFT_RESET)
        time.sleep(0.024)

    def magnetic_reset(self):
        self.write(R_PMU_CMD, PMU_SUS)
        time.sleep(0.006)
        self.write(R_PMU_CMD, PMU_BR)          # bit reset
        time.sleep(0.014)
        self.write(R_PMU_CMD, PMU_FGR)         # flux-guide reset
        time.sleep(0.018)

    def configure(self, odr, avg):
        self.write(R_AGGR, (AVG[avg] << 4) | ODR[odr])
        self.write(R_PMU_CMD, PMU_UPD_OAE)
        time.sleep(0.001)
        self.write(R_AXIS_EN, 0x07)
        self.write(R_PMU_CMD, PMU_NM)
        time.sleep(0.038)

    # ---- data
    def read_raw(self):
        d = self.read(R_MAG, 12)
        return [fix_sign(d[i] | (d[i + 1] << 8) | (d[i + 2] << 16), 24) for i in range(0, 12, 3)]

    def compensate(self, raw):
        c = self.comp
        x, y, z = raw[0] * LSB_XY, raw[1] * LSB_XY, raw[2] * LSB_Z
        t = raw[3] * LSB_T
        t = t - 25.49 if t > 0 else (t + 25.49 if t < 0 else t)
        t = (1 + c["t_sens"]) * t + c["t_off"]
        v = [x, y, z]
        dt = t - c["t0"]
        for i in range(3):
            v[i] *= 1 + c["sens"][i]
            v[i] += c["off"][i]
            v[i] += c["tco"][i] * dt
            v[i] /= 1 + c["tcs"][i] * dt
        den = 1 - c["cyx"] * c["cxy"]
        bx = (v[0] - c["cxy"] * v[1]) / den
        by = (v[1] - c["cyx"] * v[0]) / den
        bz = v[2] + (v[0] * (c["cyx"] * c["czy"] - c["czx"]) - v[1] * (c["czy"] - c["cxy"] * c["czx"])) / den
        return bx, by, bz, t


def find_device(bus_arg, addr_arg):
    buses = [bus_arg] if bus_arg is not None else sorted(
        int(p.split("-")[-1]) for p in glob.glob("/dev/i2c-*"))
    addrs = [addr_arg] if addr_arg is not None else list(ADDRS)
    for b in buses:
        for a in addrs:
            if i2c_bound(b, a):
                continue
            dev = BMM350(b, a)
            try:
                if dev.read8(R_CHIP_ID) == CHIP_ID_VAL:
                    return dev
            except OSError:
                pass
            dev.close()
    sys.exit(f"no BMM350 (chip id 0x33) found on bus(es) {buses} at {[hex(x) for x in addrs]}")


def resolve_gpio(name):
    import gpiod
    m = re.fullmatch(r"(?i)gpio(\d)_io(\d+)", name)
    if not m:
        sys.exit(f"use GPIOn_IOm form, got '{name}'")
    bank, line = int(m.group(1)), int(m.group(2))
    for path in glob.glob("/dev/gpiochip*"):
        with gpiod.Chip(path) as chip:
            if IMX8MP_GPIO_BANKS.get(chip.get_info().label.split(".")[0]) == bank:
                li = chip.get_line_info(line)
                if li.used:
                    sys.exit(f"{name} in use by '{li.consumer}'")
                return path, line
    sys.exit(f"GPIO bank {bank} not found")


def fmt(dev, raw, show_raw):
    if show_raw:
        return f"{raw[0]:9d} {raw[1]:9d} {raw[2]:9d} | {raw[3]:9d}"
    bx, by, bz, t = dev.compensate(raw)
    mag = math.sqrt(bx * bx + by * by + bz * bz)
    hdg = (math.degrees(math.atan2(by, bx)) + 360) % 360
    return f"{bx:8.2f} {by:8.2f} {bz:8.2f} {mag:7.2f} | {hdg:6.1f} | {t:6.2f}"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, help="I2C bus (default: search all)")
    p.add_argument("--addr", type=lambda s: int(s, 0), help="0x14 or 0x15 (default: try both)")
    p.add_argument("--odr", type=float, default=100, choices=sorted(ODR), help="Hz")
    p.add_argument("--avg", type=int, default=2, choices=sorted(AVG), help="samples averaged")
    p.add_argument("--rate", type=float, default=10, help="print rate, Hz")
    p.add_argument("--count", type=int, default=0, help="lines to print (0 = until Ctrl-C)")
    p.add_argument("--raw", action="store_true", help="print raw 24-bit counts")
    p.add_argument("--int", dest="use_int", action="store_true", help="pace reads by data-ready interrupt")
    p.add_argument("--int-gpio", default="GPIO1_IO27")
    a = p.parse_args()

    if a.avg > MAX_AVG.get(a.odr, 8):
        sys.exit(f"--avg {a.avg} not allowed at {a.odr} Hz (max {MAX_AVG.get(a.odr, 8)})")

    dev = find_device(a.bus, a.addr)
    req = None
    try:
        print(f"BMM350 on /dev/i2c-{dev.busnum} addr 0x{dev.addr:02x}, "
              f"CHIP_ID 0x{dev.read8(R_CHIP_ID):02x} REV_ID 0x{dev.read8(R_REV_ID):02x}")

        dev.soft_reset()
        otp = dev.read_otp()
        dev.write(R_OTP_CMD, OTP_PWR_OFF)
        dev.comp = dev.build_comp(otp)
        print("OTP read OK  (t0 {:.2f} degC, offsets {})".format(
            dev.comp["t0"], dev.comp["off"]))

        dev.magnetic_reset()
        dev.configure(a.odr, a.avg)
        print(f"PMU_STATUS_0 0x{dev.read8(R_PMU_ST0):02x}  ERR_REG 0x{dev.read8(R_ERR):02x}  "
              f"ODR {a.odr} Hz avg {a.avg}")

        if a.use_int:
            import gpiod
            from gpiod.line import Direction, Edge
            path, off = resolve_gpio(a.int_gpio)
            # drdy enable | output enable | push-pull | active high | pulsed
            dev.write(R_INT_CTRL, 0x8E)
            cfg = {off: gpiod.LineSettings(direction=Direction.INPUT, edge_detection=Edge.RISING)}
            req = gpiod.request_lines(path, consumer="bmm350-int", config=cfg)
            print(f"INT_CTRL 0x{dev.read8(R_INT_CTRL):02x}, waiting for rising edges on {a.int_gpio}")

        print()
        if a.raw:
            print(f"{'x raw':>9} {'y raw':>9} {'z raw':>9} | {'t raw':>9}")
        else:
            print(f"{'Bx uT':>8} {'By uT':>8} {'Bz uT':>8} {'|B|':>7} | {'hdg':>6} | {'degC':>6}")

        period = 1.0 / a.rate
        n, last_print = 0, 0.0
        edges, t_start = 0, time.monotonic()
        while a.count == 0 or n < a.count:
            if req:
                if not req.wait_edge_events(1.0):
                    print(f"  no interrupt in 1 s on {a.int_gpio} - check pinmux / INT wiring")
                    continue
                edges += len(req.read_edge_events())
                raw = dev.read_raw()
                now = time.monotonic()
                if now - last_print < period:
                    continue
                last_print = now
                rate = edges / (now - t_start)
                print(fmt(dev, raw, a.raw) + f" | int {rate:6.1f} Hz")
            else:
                t0 = time.monotonic()
                print(fmt(dev, dev.read_raw(), a.raw))
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
            n += 1
    except KeyboardInterrupt:
        print()
    finally:
        if req:
            req.release()
        dev.write(R_PMU_CMD, PMU_SUS)
        dev.close()


if __name__ == "__main__":
    main()
