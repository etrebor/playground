#!/usr/bin/env python3
"""
liquidlens.py - MAX14574 4-channel liquid lens driver test tool (I2C bus 1, 0x77).

No high voltage (safe):
  python3 liquidlens.py info                    # decode all registers
  python3 liquidlens.py temp                    # internal temperature sensor
  python3 liquidlens.py temp --ext              # carrier thermistor RT1 on TM1/TM2
  python3 liquidlens.py dump | watch 0x00 0x01 | write REG VAL...

High voltage on LL1-LL4 / COM (up to +/-70 V peak):
  python3 liquidlens.py check                   # open/short self-test of outputs
  python3 liquidlens.py set 512 --hold 30       # all channels, hold for meter/scope
  python3 liquidlens.py set 300 --ch 1          # one channel (diff limits checked)
  python3 liquidlens.py step 0 1023 128         # type meter readings, error table
  python3 liquidlens.py sweep 0 1023 64 --dwell 2 --bounce --cycles 0
  python3 liquidlens.py off

Datasheet: Maxim MAX14574 Rev 0 (12/10).
  Vrms = 24.4 V + 44.5 mV x code   (code 0..1023, 10-bit)
  Output LL_-COM is 3-level (+Vpeak, 0, -Vpeak, 0) at 5 kHz, Vpeak ~70.5 V fixed;
  RMS is set by duty: duty = (Vrms / Vpeak)^2
"""
import argparse
import sys
import time

from smbus2 import SMBus, i2c_msg

DEFAULT_BUS, DEFAULT_ADDR = 1, 0x77

# ---- SAFETY: highest code allowed. 1023 = 69.7 Vrms (driver maximum).
# Lower this if the connected lens is rated below that.
CODE_MAX = 1023

# electrical-characteristics limits (datasheet test conditions)
MAX_DIFF_ANY = 27.0       # Vrms between any two electrodes
MAX_DIFF_ADJ = 19.0       # Vrms between adjacent electrodes (ring 1-2-3-4-1)

# registers
R_STATUS, R_FAIL, R_TEMP, R_MODE, R_OIS_LSB = 0x00, 0x01, 0x02, 0x03, 0x04
R_LLV1, R_CMD, R_CONF = 0x05, 0x09, 0x0A
MODE_SM, MODE_ACTIVE = 0x01, 0x02
CMD_CHK_FAIL, CMD_UPD_OUT = 0x01, 0x02
CONF_TS_INT, CONF_TS_EXT = 0x80, 0x40

V0, VLSB, VPEAK = 24.4, 0.0445, 70.5

# external thermistor (carrier: RT1 NTCG064EF104FTNX between TM2-TM1, R32 82k TM1-GND)
R_SERIES = 82000.0       # R32
NTC_R25 = 100000.0       # RT1 at 25 C
NTC_B = 4250.0           # B constant - VERIFY against the TDK NTCG064EF104 datasheet
FSTU_MS = {0: 6.4, 1: 3.2, 2: 0.8, 3: 0.4}
IL_MA = {0: 270, 1: 410, 2: 570, 3: 720}


def vrms(code):
    return V0 + VLSB * code


def duty(code):
    return min(1.0, (vrms(code) / VPEAK) ** 2)


class MAX14574:
    def __init__(self, bus, addr):
        self.addr = addr
        self.bus = SMBus(bus)

    def close(self):
        self.bus.close()

    def read(self, reg, n=1):
        w, r = i2c_msg.write(self.addr, [reg]), i2c_msg.read(self.addr, n)
        self.bus.i2c_rdwr(w, r)
        return list(r)

    def write(self, reg, *vals):
        self.bus.i2c_rdwr(i2c_msg.write(self.addr, [reg] + [v & 0xFF for v in vals]))

    # ---- codes
    def get_codes(self):
        lsb, *msb = self.read(R_OIS_LSB, 5)
        return [(msb[i] << 2) | ((lsb >> (2 * i)) & 0x3) for i in range(4)]

    def put_codes(self, codes):
        lsb = 0
        for i, c in enumerate(codes):
            lsb |= (c & 0x3) << (2 * i)
        self.write(R_OIS_LSB, lsb, *[(c >> 2) & 0xFF for c in codes])   # autoincrement 0x04..0x08

    # ---- modes
    def configure(self, fstu, il, ts_int=False):
        self.write(R_CONF, (CONF_TS_INT if ts_int else 0) | (fstu << 2) | il)   # RES bits 0

    def enable(self, fstu, il):
        self.configure(fstu, il, ts_int=True)
        self.write(R_MODE, MODE_SM | MODE_ACTIVE)
        time.sleep(FSTU_MS[fstu] / 1000 + 0.01)       # boost ramp to 70 V

    def update(self, timeout=0.05):
        """UPD_OUT=1; returns True when the chip clears it (outputs updated)."""
        self.write(R_CMD, CMD_UPD_OUT)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if not self.read(R_CMD)[0] & CMD_UPD_OUT:
                return True
            time.sleep(0.001)
        return False

    def disable(self):
        self.write(R_MODE, MODE_SM)      # ACTIVE=0: outputs pulled down, not switching
        self.write(R_MODE, 0x00)         # SM=0: sleep

    def status_flags(self):
        s = self.read(R_STATUS)[0]
        flags = []
        if s & 0x10:
            flags.append("LL_TH thermal shutdown")
        if s & 0x04:
            flags.append("BST_FAIL VH not reaching Vpeak")
        return s, flags


def decode_fail(f):
    names = [(6, "FAIL_OP open"), (5, "FAIL_SH short"), (4, "COM_FAIL"),
             (3, "LL4_FAIL"), (2, "LL3_FAIL"), (1, "LL2_FAIL"), (0, "LL1_FAIL")]
    return [n for b, n in names if f & (1 << b)] or ["none"]


def check_limits(codes):
    v = [vrms(c) for c in codes]
    problems = []
    for i in range(4):
        for j in range(i + 1, 4):
            d = abs(v[i] - v[j])
            adj = (j - i == 1) or (i == 0 and j == 3)
            if d > MAX_DIFF_ANY or (adj and d > MAX_DIFF_ADJ):
                problems.append(f"LL{i + 1}-LL{j + 1} {d:.1f} Vrms")
    return problems


def code_ok(code):
    if not 0 <= code <= 1023:
        sys.exit(f"code {code} outside 0..1023")
    if code > CODE_MAX:
        sys.exit(f"code {code} above CODE_MAX {CODE_MAX} (lens safety limit, top of script)")


def chans(spec):
    return [0, 1, 2, 3] if spec == "all" else [int(c) - 1 for c in spec.split(",")]


def apply_code(dev, code, ch, force=False):
    codes = dev.get_codes()
    for i in ch:
        codes[i] = code
    bad = check_limits(codes)
    if bad and not force:
        raise SystemExit("electrode difference limit exceeded: " + ", ".join(bad) + " (use --force)")
    dev.put_codes(codes)
    ok = dev.update()
    s, flags = dev.status_flags()
    return codes, ok, flags


def line(code, ok, flags):
    d = duty(code)
    st = "updated" if ok else "UPD_OUT NOT CLEARED"
    fl = ("  !! " + "; ".join(flags)) if flags else ""
    return (f"code {code:4d} (0x{code:03x})  expect {vrms(code):5.2f} Vrms  "
            f"duty {d * 100:5.1f}% (+/-{VPEAK:.1f} V peak)  {st}{fl}")


# ---------------------------------------------------------------- commands
def cmd_info(dev, a):
    r = dev.read(0x00, 11)
    s, f, ts, mode, lsb = r[0], r[1], r[2], r[3], r[4]
    conf = r[10]
    print(f"MAX14574 @ bus {a.bus} 0x{dev.addr:02x}")
    print(f"  Status   0x{s:02x}  REV={s >> 5}  LL_TH={(s >> 4) & 1}  BST_FAIL={(s >> 2) & 1}")
    print(f"  Fail     0x{f:02x}  {', '.join(decode_fail(f))}   (valid after 'check')")
    print(f"  UserMode 0x{mode:02x}  SM={mode & 1} ({'normal' if mode & 1 else 'sleep'})  "
          f"ACTIVE={(mode >> 1) & 1} ({'switching' if mode & 2 else 'outputs pulled down'})")
    print(f"  DriverConf 0x{conf:02x}  TS_INT={conf >> 7} TS_EXT={(conf >> 6) & 1}  "
          f"FSTU={(conf >> 2) & 3} ({FSTU_MS[(conf >> 2) & 3]} ms)  IL={conf & 3} ({IL_MA[conf & 3]} mA)")
    if conf & 0x30:
        print("  WARNING: reserved DriverConf bits 5:4 set - must be 0")
    for i, c in enumerate(dev.get_codes()):
        print(f"  LL{i + 1} code {c:4d} -> {vrms(c):5.2f} Vrms")
    print(f"  TempSense 0x{ts:02x}" + ("" if ts else " (not enabled / not available)"))
    print(f"  raw 0x00-0x0A: " + " ".join(f"{b:02x}" for b in r))


def ntc_temp(code):
    import math
    if code in (0, 255):
        return None, None
    rt = R_SERIES * (255.0 / code - 1.0)          # code = 255 * R32 / (RT1 + R32)
    t = 1.0 / (1.0 / 298.15 + math.log(rt / NTC_R25) / NTC_B) - 273.15
    return rt, t


def cmd_temp(dev, a):
    conf = dev.read(R_CONF)[0]
    mode = dev.read(R_MODE)[0]
    if not mode & MODE_SM:
        dev.write(R_MODE, mode | MODE_SM)       # leave sleep (ACTIVE untouched -> no HV)
    dev.write(R_CONF, (conf & 0x0F) | (CONF_TS_EXT if a.ext else CONF_TS_INT))
    time.sleep(0.005)
    for _ in range(a.count):
        t = dev.read(R_TEMP)[0]
        if t == 0:
            print("TempSense 0x00 - measurement not available")
        elif a.ext:
            rt, tc = ntc_temp(t)
            v = t / 255.0
            if rt is None:
                print(f"TempSense 0x{t:02x} -> V_TM1 {v:.3f} V  (rail - thermistor open/short?)")
            else:
                print(f"TempSense 0x{t:02x} -> V_TM1 {v:.3f} V  RT1 {rt / 1000:7.1f} k  T {tc:6.1f} C (NTC)")
        else:
            tj = (t - 127) * 1.6 - 60
            note = "  (accuracy not guaranteed below 0x7F)" if t < 0x7F else ""
            print(f"TempSense 0x{t:02x} -> Tj {tj:6.1f} C{note}")
        time.sleep(a.interval)
    if not mode & MODE_SM:
        dev.write(R_MODE, mode)                 # back to sleep


def cmd_check(dev, a):
    mode = dev.read(R_MODE)[0]
    dev.configure(a.fstu, a.il)
    dev.write(R_MODE, mode | MODE_SM)
    time.sleep(FSTU_MS[a.fstu] / 1000 + 0.01)
    dev.write(R_CMD, CMD_CHK_FAIL)
    t0 = time.monotonic()
    while dev.read(R_CMD)[0] & CMD_CHK_FAIL and time.monotonic() - t0 < 0.1:
        time.sleep(0.001)
    f = dev.read(R_FAIL)[0]
    s, flags = dev.status_flags()
    print(f"Fail 0x{f:02x}: {', '.join(decode_fail(f))}")
    if flags:
        print("Status: " + "; ".join(flags))
    print("Note: channels with no lens electrode connected report as open/failed.")
    dev.write(R_MODE, mode)


def cmd_set(dev, a):
    code_ok(a.code)
    dev.enable(a.fstu, a.il)
    try:
        codes, ok, flags = apply_code(dev, a.code, chans(a.ch), a.force)
        print("output ON  " + line(a.code, ok, flags))
        print("  all channels: " + "  ".join(f"LL{i + 1}={vrms(c):.2f}" for i, c in enumerate(codes)))
        if a.hold is None:
            input("holding - measure now, Enter to turn output off ")
        else:
            time.sleep(a.hold)
    finally:
        if not a.leave_on:
            dev.disable()
            print("output OFF (ACTIVE=0, sleep)")


def code_list(a):
    step = a.step if a.start <= a.stop else -a.step
    codes = list(range(a.start, a.stop + (1 if step > 0 else -1), step))
    if codes[-1] != a.stop:
        codes.append(a.stop)          # always finish on the requested end code
    for c in codes:
        code_ok(c)
    return codes


def cmd_step(dev, a):
    codes = code_list(a)
    dev.enable(a.fstu, a.il)
    results = []
    print("type the measured Vrms at each step (Enter skips, q quits)\n")
    try:
        for c in codes:
            _, ok, flags = apply_code(dev, c, chans(a.ch), a.force)
            print(line(c, ok, flags))
            s = input("  measured Vrms: ").strip()
            if s.lower() == "q":
                break
            results.append((c, vrms(c), float(s) if s else None))
    finally:
        dev.disable()
        print("output OFF")
    meas = [r for r in results if r[2] is not None]
    if meas:
        print(f"\n{'code':>5} {'expect':>8} {'measured':>9} {'error':>7} {'err %':>6}")
        for c, e, m in results:
            if m is None:
                print(f"{c:5d} {e:8.2f} {'-':>9}")
            else:
                print(f"{c:5d} {e:8.2f} {m:9.2f} {m - e:7.2f} {100 * (m - e) / e:6.1f}")
        print("datasheet: output RMS voltage error +/-2.5 %")


def cmd_sweep(dev, a):
    codes = code_list(a)
    if a.bounce:
        codes = codes + codes[-2:0:-1]
    dev.enable(a.fstu, a.il)
    try:
        n = 0
        while a.cycles == 0 or n < a.cycles:
            print(f"-- cycle {n + 1}")
            for c in codes:
                _, ok, flags = apply_code(dev, c, chans(a.ch), a.force)
                print(line(c, ok, flags))
                time.sleep(a.dwell)
            n += 1
    except KeyboardInterrupt:
        print()
    finally:
        dev.disable()
        print("output OFF")


def cmd_off(dev, a):
    dev.disable()
    print("output OFF (ACTIVE=0, SM=0 sleep)")


def cmd_dump(dev, a):
    data = dev.read(0x00, a.count)
    for i in range(0, len(data), 8):
        print(f"  0x{i:02x}: " + " ".join(f"{b:02x}" for b in data[i:i + 8]))


def cmd_watch(dev, a):
    regs = [int(r, 0) for r in a.regs]
    last = None
    print("watching " + " ".join(f"0x{r:02x}" for r in regs) + " (Ctrl-C to stop)")
    while True:
        cur = [dev.read(r)[0] for r in regs]
        if cur != last:
            print(time.strftime("%H:%M:%S ") + "  ".join(f"[0x{r:02x}]=0x{v:02x}" for r, v in zip(regs, cur)))
            last = cur
        time.sleep(a.interval)


def cmd_write(dev, a):
    reg = int(a.reg, 0)
    vals = [int(v, 0) for v in a.values]
    dev.write(reg, *vals)
    print(f"wrote  [0x{reg:02x}..] " + " ".join(f"{v:02x}" for v in vals))
    print(f"readbk [0x{reg:02x}..] " + " ".join(f"{b:02x}" for b in dev.read(reg, len(vals))))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bus", type=int, default=DEFAULT_BUS)
    p.add_argument("--addr", type=lambda s: int(s, 0), default=DEFAULT_ADDR)
    p.add_argument("--fstu", type=int, default=0, choices=range(4),
                   help="boost startup 0=6.4ms 1=3.2 2=0.8 3=0.4 (default 0, gentlest)")
    p.add_argument("--il", type=int, default=0, choices=range(4),
                   help="inductor current limit 0=270mA 1=410 2=570 3=720 (default 0)")
    s = p.add_subparsers(dest="cmd", required=True)

    s.add_parser("info").set_defaults(fn=cmd_info)
    x = s.add_parser("temp"); x.add_argument("--count", type=int, default=5)
    x.add_argument("--interval", type=float, default=0.5)
    x.add_argument("--ext", action="store_true", help="external thermistor RT1 (TM1/TM2) instead of internal")
    x.set_defaults(fn=cmd_temp)
    s.add_parser("check").set_defaults(fn=cmd_check)
    x = s.add_parser("dump"); x.add_argument("--count", type=int, default=11); x.set_defaults(fn=cmd_dump)
    x = s.add_parser("watch"); x.add_argument("regs", nargs="+")
    x.add_argument("--interval", type=float, default=0.2); x.set_defaults(fn=cmd_watch)
    x = s.add_parser("write"); x.add_argument("reg"); x.add_argument("values", nargs="+"); x.set_defaults(fn=cmd_write)

    def hv(x):
        x.add_argument("--ch", default="all", help="channels, e.g. all or 1 or 1,3 (default all)")
        x.add_argument("--force", action="store_true", help="ignore electrode difference limits")

    x = s.add_parser("set"); x.add_argument("code", type=lambda v: int(v, 0)); hv(x)
    x.add_argument("--hold", type=float, help="seconds (default: until Enter)")
    x.add_argument("--leave-on", action="store_true"); x.set_defaults(fn=cmd_set)
    for name, fn in (("step", cmd_step), ("sweep", cmd_sweep)):
        x = s.add_parser(name); hv(x)
        x.add_argument("start", type=lambda v: int(v, 0)); x.add_argument("stop", type=lambda v: int(v, 0))
        x.add_argument("step", type=lambda v: int(v, 0))
        if name == "sweep":
            x.add_argument("--dwell", type=float, default=1.0)
            x.add_argument("--cycles", type=int, default=1, help="0 = until Ctrl-C")
            x.add_argument("--bounce", action="store_true")
        x.set_defaults(fn=fn)
    s.add_parser("off").set_defaults(fn=cmd_off)

    a = p.parse_args()
    if getattr(a, "step", 1) <= 0:
        sys.exit("step must be > 0")
    dev = MAX14574(a.bus, a.addr)
    try:
        a.fn(dev, a)
    except KeyboardInterrupt:
        print()
        if a.cmd in ("set", "step", "sweep"):
            dev.disable()
            print("output OFF")
    except OSError as e:
        sys.exit(f"I2C error on bus {a.bus} addr 0x{a.addr:02x}: {e}")
    finally:
        dev.close()


if __name__ == "__main__":
    main()
