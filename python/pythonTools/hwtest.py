#!/usr/bin/env python3
"""
hwtest.py - quick hardware checks for the QSXP-ML81 (i.MX8M Plus) carrier.

Uses: gpiod (libgpiod v2 bindings), smbus2, spidev.

  hwtest.py info                              # buses, chips, spidev nodes, versions
  hwtest.py i2c-scan 1                        # i2cdetect-style scan (UU = kernel driver bound)
  hwtest.py i2c-read 1 0x57 0x00 16           # 8-bit reg addr, read 16 bytes
  hwtest.py i2c-read 1 0x18 0x3000 2 -w 2     # 16-bit reg addr (sensor-style)
  hwtest.py i2c-write 1 0x57 0x00 0xAA 0x55   # write bytes after reg addr
  hwtest.py gpio-list [--all]                 # lines in use (or all lines)
  hwtest.py gpio-get GPIO1_IO05
  hwtest.py gpio-set GPIO1_IO05 1 --hold 5
  hwtest.py gpio-toggle GPIO1_IO05 --hz 10 --count 50
  hwtest.py spi-loop 1 0 --speed 1000000      # needs MOSI<->MISO jumper
  hwtest.py spi-probe 1 0                     # heuristic ID probe, all 4 modes
  hwtest.py spi-probe 1 0 --cs-gpio GPIO5_IO09 # CS as plain GPIO (only if not in DT cs-gpios)

GPIO names accept GPIOn_IOm, n:m, or /dev/gpiochipX:m.
"""
import argparse
import glob
import os
import re
import sys
import time

# i.MX8M Plus GPIO controller base address -> bank number
IMX8MP_GPIO_BANKS = {
    "30200000": 1, "30210000": 2, "30220000": 3,
    "30230000": 4, "30240000": 5,
}


# ----------------------------------------------------------------- GPIO
def _gpiod():
    try:
        import gpiod
    except ImportError:
        sys.exit("python3-gpiod not installed")
    if not hasattr(gpiod, "request_lines"):
        sys.exit("gpiod v1 bindings found; this script needs libgpiod v2 (python3-gpiod >= 2.0)")
    return gpiod


def gpio_chip_map():
    gpiod = _gpiod()
    out = {}
    for path in sorted(glob.glob("/dev/gpiochip*"), key=lambda p: int(p[13:])):
        with gpiod.Chip(path) as chip:
            info = chip.get_info()
            bank = IMX8MP_GPIO_BANKS.get(info.label.split(".")[0])
            out[path] = (bank, info)
    return out


def resolve_gpio(name):
    m = re.fullmatch(r"(?i)gpio(\d)_io(\d+)", name) or re.fullmatch(r"(\d):(\d+)", name)
    if m:
        bank, line = int(m.group(1)), int(m.group(2))
        for path, (b, _) in gpio_chip_map().items():
            if b == bank:
                return path, line
        sys.exit(f"GPIO bank {bank} not found (check gpio-list)")
    m = re.fullmatch(r"(/dev/gpiochip\d+):(\d+)", name)
    if m:
        return m.group(1), int(m.group(2))
    sys.exit(f"can't parse GPIO name '{name}'")


def _check_free(path, off, name):
    gpiod = _gpiod()
    with gpiod.Chip(path) as chip:
        li = chip.get_line_info(off)
    if li.used:
        sys.exit(f"{name} is in use by '{li.consumer or '?'}' - refusing to touch it")
    return li


def cmd_gpio_list(a):
    gpiod = _gpiod()
    for path, (bank, info) in gpio_chip_map().items():
        tag = f"GPIO{bank}" if bank else "?"
        print(f"{path} [{info.label}] {tag}, {info.num_lines} lines")
        with gpiod.Chip(path) as chip:
            for off in range(info.num_lines):
                li = chip.get_line_info(off)
                if li.used or a.all:
                    state = f"USED by {li.consumer or '?'}" if li.used else "free"
                    low = " active-low" if li.active_low else ""
                    print(f"   {tag}_IO{off:02d}  {state:28s} {li.direction.name.lower()}{low}")


def cmd_gpio_get(a):
    gpiod = _gpiod()
    from gpiod.line import Direction
    path, off = resolve_gpio(a.line)
    li = _check_free(path, off, a.line)
    if li.direction == Direction.OUTPUT:
        print(f"note: {a.line} is currently an output; reading reconfigures it as input")
    cfg = {off: gpiod.LineSettings(direction=Direction.INPUT)}
    with gpiod.request_lines(path, consumer="hwtest", config=cfg) as req:
        print(f"{a.line} ({path}:{off}) = {req.get_value(off).value}")


def cmd_gpio_set(a):
    gpiod = _gpiod()
    from gpiod.line import Direction, Value
    path, off = resolve_gpio(a.line)
    _check_free(path, off, a.line)
    val = Value.ACTIVE if a.value else Value.INACTIVE
    cfg = {off: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=val)}
    with gpiod.request_lines(path, consumer="hwtest", config=cfg):
        print(f"{a.line} ({path}:{off}) driven {a.value}")
        if a.hold is not None:
            time.sleep(a.hold)
        else:
            input("holding - press Enter to release ")


def cmd_gpio_toggle(a):
    gpiod = _gpiod()
    from gpiod.line import Direction, Value
    path, off = resolve_gpio(a.line)
    _check_free(path, off, a.line)
    half = 0.5 / a.hz
    cfg = {off: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)}
    with gpiod.request_lines(path, consumer="hwtest", config=cfg) as req:
        print(f"toggling {a.line} at {a.hz} Hz x{a.count} (Ctrl-C to stop)")
        try:
            for _ in range(a.count):
                req.set_value(off, Value.ACTIVE)
                time.sleep(half)
                req.set_value(off, Value.INACTIVE)
                time.sleep(half)
        except KeyboardInterrupt:
            pass


# ------------------------------------------------------------------ I2C
def _smbus():
    try:
        from smbus2 import SMBus, i2c_msg
    except ImportError:
        sys.exit("python3-smbus2 not installed")
    return SMBus, i2c_msg


def i2c_bound(bus, addr):
    drv = f"/sys/bus/i2c/devices/{bus}-{addr:04x}/driver"
    return os.path.basename(os.readlink(drv)) if os.path.exists(drv) else None


def cmd_i2c_scan(a):
    SMBus, i2c_msg = _smbus()
    bound = {}
    print("     " + "  ".join(f"{i:x}" for i in range(16)))
    with SMBus(a.bus) as bus:
        for row in range(0, 128, 16):
            cells = []
            for addr in range(row, row + 16):
                if addr < 0x03 or addr > 0x77:
                    cells.append("  ")
                    continue
                drv = i2c_bound(a.bus, addr)
                if drv:
                    bound[addr] = drv
                    cells.append("UU")
                    continue
                try:
                    bus.i2c_rdwr(i2c_msg.read(addr, 1))
                    cells.append(f"{addr:02x}")
                except OSError:
                    cells.append("--")
            print(f"{row:02x}: " + " ".join(cells))
    for addr, drv in bound.items():
        print(f"UU 0x{addr:02x} -> driver '{drv}'")


def _hexdump(start, data):
    for i in range(0, len(data), 16):
        chunk = data[i:i + 16]
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        print(f"{start + i:04x}: " + " ".join(f"{b:02x}" for b in chunk) + f"  {asc}")


def cmd_i2c_read(a):
    SMBus, i2c_msg = _smbus()
    addr, reg = int(a.addr, 0), int(a.reg, 0)
    w = i2c_msg.write(addr, list(reg.to_bytes(a.reg_width, "big")))
    r = i2c_msg.read(addr, a.count)
    with SMBus(a.bus) as bus:
        bus.i2c_rdwr(w, r)
    _hexdump(reg, list(r))


def cmd_i2c_write(a):
    SMBus, i2c_msg = _smbus()
    addr, reg = int(a.addr, 0), int(a.reg, 0)
    drv = i2c_bound(a.bus, addr)
    if drv and not a.force:
        sys.exit(f"0x{addr:02x} is owned by driver '{drv}'; use --force to write anyway")
    payload = list(reg.to_bytes(a.reg_width, "big")) + [int(x, 0) & 0xFF for x in a.data]
    with SMBus(a.bus) as bus:
        bus.i2c_rdwr(i2c_msg.write(addr, payload))
    print(f"wrote {len(a.data)} byte(s) to 0x{addr:02x} reg 0x{reg:0{a.reg_width * 2}x}")


# ------------------------------------------------------------------ SPI
def cmd_spi_loop(a):
    try:
        import spidev
    except ImportError:
        sys.exit("python3-spidev not installed")
    dev = f"/dev/spidev{a.bus}.{a.cs}"
    if not os.path.exists(dev):
        sys.exit(f"{dev} missing - needs a spidev child node in the devicetree")
    spi = spidev.SpiDev()
    spi.open(a.bus, a.cs)
    spi.max_speed_hz, spi.mode = a.speed, a.mode
    tx = [(0x55 if i % 2 else 0xAA) ^ (i & 0x0F) for i in range(a.len)]
    rx = spi.xfer2(list(tx))
    spi.close()
    bad = [i for i, (t, r) in enumerate(zip(tx, rx)) if t != r]
    if not bad:
        print(f"PASS: {a.len} bytes looped back at {a.speed} Hz mode {a.mode}")
    else:
        print(f"FAIL: {len(bad)}/{a.len} mismatched; first at byte {bad[0]}: "
              f"tx {tx[bad[0]]:02x} rx {rx[bad[0]]:02x}")
        if all(r == rx[0] for r in rx):
            print(f"      rx stuck at 0x{rx[0]:02x} - check MOSI/MISO jumper and pinmux")
        sys.exit(1)


# ID / WHO_AM_I commands answered by common SPI parts: (label, command bytes, response length)
SPI_PROBES = [
    ("JEDEC RDID 0x9F (NOR flash, FRAM, many)", [0x9F], 3),
    ("REMS 0x90 (flash mfr/dev id)",            [0x90, 0, 0, 0], 2),
    ("RES 0xAB (flash wake + id)",              [0xAB, 0, 0, 0], 1),
    ("RDSR 0x05 (flash/EEPROM status)",         [0x05], 1),
    ("rd 0x00|0x80 (ADI-style reg 0)",          [0x80], 1),
    ("rd 0x0F|0x80 (ST WHO_AM_I)",              [0x8F], 1),
    ("rd 0x75|0x80 (InvenSense WHO_AM_I)",      [0xF5], 1),
    ("rd 0xD0 (Bosch chip id)",                 [0xD0], 1),
    ("rd 0x00+dummy (Bosch BMI3xx chip id)",    [0x80, 0x00], 2),
]


def _classify(tx, resp):
    if all(b == 0xFF for b in resp):
        return "ff"
    if all(b == 0x00 for b in resp):
        return "00"
    if resp == tx[1:len(resp) + 1] or resp == tx[:len(resp)]:
        return "echo"
    return "DATA"


def cmd_spi_probe(a):
    try:
        import spidev
    except ImportError:
        sys.exit("python3-spidev not installed")
    dev = f"/dev/spidev{a.bus}.{a.cs}"
    if not os.path.exists(dev):
        sys.exit(f"{dev} missing - needs a spidev child node in the devicetree")

    req = cs_off = cs_line = None
    if a.cs_gpio:
        gpiod = _gpiod()
        from gpiod.line import Direction, Value
        path, cs_line = resolve_gpio(a.cs_gpio)
        _check_free(path, cs_line, a.cs_gpio)
        # Value.ACTIVE = CS asserted; --cs-high flips polarity for active-high selects
        cfg = {cs_line: gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE,
                                           active_low=not a.cs_high)}
        req = gpiod.request_lines(path, consumer="hwtest-cs", config=cfg)
        cs_on, cs_off = Value.ACTIVE, Value.INACTIVE

    spi = spidev.SpiDev()
    spi.open(a.bus, a.cs)
    spi.max_speed_hz = a.speed

    def xfer(data):
        if req:
            req.set_value(cs_line, cs_on)
            time.sleep(0.0001)
        rx = spi.xfer2(list(data))
        if req:
            req.set_value(cs_line, cs_off)
        time.sleep(0.001)
        return rx

    try:
        # loopback / short check first: a pattern no real device would echo
        pat = [0xA5, 0x5A, 0xC3, 0x3C]
        spi.mode = 0
        if xfer(pat) == pat:
            print("WARNING: MISO echoes MOSI exactly - shorted or loopback jumper fitted; results meaningless")

        hits = 0
        for mode in range(4):
            spi.mode = mode
            print(f"\n--- mode {mode} (CPOL={mode >> 1} CPHA={mode & 1}) @ {a.speed} Hz ---")
            for label, cmd, n in SPI_PROBES:
                rx = xfer(cmd + [0x00] * n)
                resp = rx[len(cmd):]
                kind = _classify(cmd + [0x00] * n, resp)
                hits += kind == "DATA"
                mark = "  <==" if kind == "DATA" else ""
                print(f"  {label:42s} {' '.join(f'{b:02x}' for b in resp):9s} [{kind}]{mark}")
    finally:
        spi.close()
        if req:
            req.release()

    print()
    if hits:
        print(f"{hits} probe(s) returned non-trivial data - something is decoding commands.")
        print("Compare modes: identical bytes = mode-tolerant; bit-shifted bytes = wrong CPHA in those modes.")
    else:
        print("No non-trivial responses. Absent/unpowered device, wrong CS, MISO not muxed,")
        print("or a device whose read protocol isn't in the probe list (e.g. write-only DAC, shift register).")


# ----------------------------------------------------------------- info
def cmd_info(a):
    print(f"python {sys.version.split()[0]}")
    for mod in ("gpiod", "smbus2", "spidev"):
        try:
            m = __import__(mod)
            print(f"{mod:8s} {getattr(m, '__version__', 'ok')}")
        except ImportError:
            print(f"{mod:8s} MISSING")
    print("\nI2C adapters:")
    for p in sorted(glob.glob("/sys/class/i2c-adapter/i2c-*")):
        with open(f"{p}/name") as f:
            print(f"  {os.path.basename(p):8s} {f.read().strip()}")
    print("\nGPIO chips:")
    try:
        for path, (bank, info) in gpio_chip_map().items():
            print(f"  {path:16s} [{info.label}] GPIO{bank} {info.num_lines} lines")
    except SystemExit as e:
        print(f"  {e}")
    spis = sorted(glob.glob("/dev/spidev*"))
    print("\nSPI: " + (" ".join(spis) if spis else "no /dev/spidev* nodes"))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    s = p.add_subparsers(dest="cmd", required=True)

    s.add_parser("info").set_defaults(fn=cmd_info)

    x = s.add_parser("gpio-list"); x.add_argument("--all", action="store_true"); x.set_defaults(fn=cmd_gpio_list)
    x = s.add_parser("gpio-get"); x.add_argument("line"); x.set_defaults(fn=cmd_gpio_get)
    x = s.add_parser("gpio-set"); x.add_argument("line"); x.add_argument("value", type=int, choices=(0, 1))
    x.add_argument("--hold", type=float, help="seconds to hold (default: until Enter)"); x.set_defaults(fn=cmd_gpio_set)
    x = s.add_parser("gpio-toggle"); x.add_argument("line"); x.add_argument("--hz", type=float, default=1.0)
    x.add_argument("--count", type=int, default=10); x.set_defaults(fn=cmd_gpio_toggle)

    x = s.add_parser("i2c-scan"); x.add_argument("bus", type=int); x.set_defaults(fn=cmd_i2c_scan)
    x = s.add_parser("i2c-read"); x.add_argument("bus", type=int); x.add_argument("addr"); x.add_argument("reg")
    x.add_argument("count", type=int, nargs="?", default=1)
    x.add_argument("-w", "--reg-width", type=int, choices=(1, 2), default=1); x.set_defaults(fn=cmd_i2c_read)
    x = s.add_parser("i2c-write"); x.add_argument("bus", type=int); x.add_argument("addr"); x.add_argument("reg")
    x.add_argument("data", nargs="+"); x.add_argument("-w", "--reg-width", type=int, choices=(1, 2), default=1)
    x.add_argument("--force", action="store_true"); x.set_defaults(fn=cmd_i2c_write)

    x = s.add_parser("spi-loop"); x.add_argument("bus", type=int); x.add_argument("cs", type=int)
    x.add_argument("--speed", type=int, default=1_000_000); x.add_argument("--mode", type=int, default=0, choices=range(4))
    x.add_argument("--len", type=int, default=32); x.set_defaults(fn=cmd_spi_loop)
    x = s.add_parser("spi-probe"); x.add_argument("bus", type=int); x.add_argument("cs", type=int)
    x.add_argument("--speed", type=int, default=500_000)
    x.add_argument("--cs-gpio", help="drive this GPIO as chip select around each transfer")
    x.add_argument("--cs-high", action="store_true", help="CS is active-high (default active-low)")
    x.set_defaults(fn=cmd_spi_probe)

    a = p.parse_args()
    try:
        a.fn(a)
    except OSError as e:
        sys.exit(f"{a.cmd}: {e}")


if __name__ == "__main__":
    main()
