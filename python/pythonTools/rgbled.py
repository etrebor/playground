#!/usr/bin/env python3
"""
rgbled.py - carrier RGB status LED (red GPIO4_IO24, green IO23, blue IO25).

  python3 rgbled.py        all off
  python3 rgbled.py -r     red on,   green + blue off
  python3 rgbled.py -g     green on, red + blue off
  python3 rgbled.py -b     blue on,  red + green off

Uses /sys/class/leds/{red,green,blue} (gpio-leds node from
imx8mp-status-leds.dtsi; polarity is set in the devicetree). If those
entries are missing (older dtb without the node), it falls back to
driving the GPIO4 lines directly with python3-gpiod (libgpiod v2);
add --active-low there if the LED is lit by a low pin.
"""
import argparse
import glob
import os
import sys

LED_SYSFS = "/sys/class/leds"
ORDER = ("red", "green", "blue")
LINES = {"red": 24, "green": 23, "blue": 25}   # GPIO4 offsets
GPIO4_LABEL = "30230000"


def set_sysfs(state):
    for name in ORDER:
        with open(os.path.join(LED_SYSFS, name, "brightness"), "w") as f:
            f.write("1" if state[name] else "0")


def set_gpiod(state, active_low):
    try:
        import gpiod
        from gpiod.line import Direction, Value
    except ImportError:
        sys.exit("no /sys/class/leds entries and python3-gpiod is not installed")
    path = None
    for p in sorted(glob.glob("/dev/gpiochip*")):
        with gpiod.Chip(p) as chip:
            if chip.get_info().label.startswith(GPIO4_LABEL):
                path = p
                break
    if path is None:
        sys.exit("GPIO4 controller (30230000.gpio) not found")
    cfg = {}
    for name in ORDER:
        high = state[name] != active_low
        cfg[LINES[name]] = gpiod.LineSettings(
            direction=Direction.OUTPUT,
            output_value=Value.ACTIVE if high else Value.INACTIVE)
    with gpiod.request_lines(path, consumer="rgbled", config=cfg):
        pass                                # i.MX keeps the level after release


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group()
    g.add_argument("-r", action="store_true", help="red on, green and blue off")
    g.add_argument("-g", action="store_true", help="green on, red and blue off")
    g.add_argument("-b", action="store_true", help="blue on, red and green off")
    ap.add_argument("--active-low", action="store_true",
                    help="GPIO fallback only: LED lit when the pin is low")
    a = ap.parse_args()

    state = {"red": a.r, "green": a.g, "blue": a.b}   # none given -> all off

    if all(os.path.exists(os.path.join(LED_SYSFS, n, "brightness")) for n in ORDER):
        set_sysfs(state)
        how = "sysfs"
    else:
        set_gpiod(state, a.active_low)
        how = "gpiod"

    print("  ".join(f"{n}={'ON' if state[n] else 'off'}" for n in ORDER) + f"   ({how})")


if __name__ == "__main__":
    main()
