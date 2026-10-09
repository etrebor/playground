#!/usr/bin/env python3
"""
bq35100_run.py - timed BQ35100 measurement session via the ATtiny1616 monitor.

Sequence:
  1. ~GE+   -> wait for "GE ON"
  2. log voltage ("n.nv") and ALERT lines for <seconds>
  3. ~GE-   -> wait for "GE OFF" (gauge saves its data first)
  4. ~PWR-  -> ATtiny drops PA4 10 s later
  5. power off Linux (must finish inside the 10 s window)

Recovery: remove and re-apply power to the ATtiny1616; PA4 defaults high
at reset, which powers the board back up.

Status LED (gpio-leds in the devicetree, see imx8mp-status-leds.dtsi):
  green          start-up
  green          3-reading voltage average back within 1 V of baseline
  blue           3-reading average more than 1 V below baseline
  red            3-reading average more than 2 V below baseline
  red + blue     ATtiny button pressed  ("btn+", PA5 pulled low)
  green          ATtiny button released ("btn-")
  green + blue   SoM button on GPIO5_IO20 pressed (pulled low)
  green          SoM button released
  red, blinking  "err" from the ATtiny (PC3 not opposite PB4), 0.2 s toggle,
                 until no "err" for 1.2 s
Baseline = average of the first three voltage readings of this run.
After any button event the LED holds that colour for 3 s; other LED
changes requested during the hold are deferred and the latest one is
applied when the hold ends. The voltage rule only changes the LED when its
own colour changes.

Serial uses only the standard library (termios). GPIO5_IO20 uses the
python3 gpiod (libgpiod v2) bindings; if they are missing, the script warns
and runs without that input.
Must run as root unless --no-poweroff is given.

Calibration (one-time, see README):
  bq35100_run.py --calibrate             # ~cal2: VIN + 2 series cells
  bq35100_run.py --vcal 6100             # ~vcal: gain to meter reading (mV)
  bq35100_run.py --calibrate --vcal 6100 # both, in order

Examples:
  bq35100_run.py 60
  bq35100_run.py 300 --log /home/root/gauge.csv
  bq35100_run.py 10 --no-poweroff       # bench test, no power-down
  bq35100_run.py 300 --forcebat         # ~forcebat+ once the baseline is set
"""
import argparse
import collections
import datetime
import glob
import os
import select
import subprocess
import sys
import termios
import time

DEFAULT_PORT = "/dev/ttymxc1"   # verify which ttymxcN is UART_B on your board

# /sys/class/leds/<name> - names come from the gpio-leds labels in the dtsi
LED_NAMES = {"red": "red", "green": "green", "blue": "blue"}
LED_SYSFS = "/sys/class/leds"

# After a button sets the LED, hold that colour this long. Other LED changes
# requested during the hold are deferred; the latest one is applied when the
# hold ends.
BUTTON_HOLD_S = 3.0

# "err" from the ATtiny (PC3 not the opposite of PB4): blink red, toggling
# every ERR_BLINK_S. The ATtiny repeats "err" every 0.5 s while the fault is
# present; blinking stops ERR_CLEAR_S after the last "err" and the LED returns
# to the colour it would otherwise show. LED requests during the error are
# remembered (latest wins) and applied when it clears.
ERR_BLINK_S = 0.2
ERR_CLEAR_S = 1.2

# SoM button: normally open to GND, 10k pull-up to 3V3 -> pressed = low.
# GPIO5_IO20 (GPIO5 = 30240000.gpio)
SOM_GPIO_CHIP_LABEL = "30240000.gpio"
SOM_GPIO_LINE = 20
SOM_GPIO_DEBOUNCE_MS = 20

# LED patterns: (red, green, blue)
COLORS = {
    "green":   (0, 1, 0),
    "blue":    (0, 0, 1),
    "red":     (1, 0, 0),
    "magenta": (1, 0, 1),     # red + blue   (ATtiny button pressed)
    "cyan":    (0, 1, 1),     # green + blue (SoM button pressed)
    "off":     (0, 0, 0),     # blink-off phase
}

BLUE_DROP_V = 1.0      # average more than this below baseline -> blue
RED_DROP_V = 2.0       # average more than this below baseline -> red
AVG_COUNT = 3


def parse_voltage(line):
    """Convert an ATtiny "n.nv" string to volts (float); None if not a voltage."""
    line = line.strip()
    if len(line) < 2 or line[-1] != "v":
        return None
    try:
        return float(line[:-1])
    except ValueError:
        return None


class StatusLed:
    """RGB status LED driven through /sys/class/leds, with a button hold."""

    def __init__(self):
        self.color = None
        self.ok = True
        self.hold_until = 0.0          # monotonic time the current hold ends
        self.pending = None            # (color, hold) deferred during a hold
        self.blinking = False          # error blink active
        self.err_until = 0.0           # blink stops at this time unless renewed
        self.next_toggle = 0.0
        self.blink_on = False
        self.restore = None            # colour to show when the error clears
        self.on_error_clear = None     # optional callback for logging

    def error(self):
        """Start (or keep) red blinking. Returns True if blinking just started."""
        now = time.monotonic()
        self.err_until = now + ERR_CLEAR_S
        if self.blinking:
            return False
        self.blinking = True
        self.restore = self.color if self.color not in (None, "off") else "green"
        self.blink_on = True
        self.set("red")
        self.next_toggle = now + ERR_BLINK_S
        return True

    def request(self, color, hold=False):
        """Set the LED, or defer the change if an error or button hold is active.

        hold=True (button events) starts a BUTTON_HOLD_S hold once applied.
        """
        if self.blinking:
            self.restore = color                   # apply when the error clears
            return
        now = time.monotonic()
        if now < self.hold_until:
            self.pending = (color, hold)           # latest request wins
            return
        self.set(color)
        if hold:
            self.hold_until = now + BUTTON_HOLD_S

    def poll(self):
        """Drive the error blink, and apply deferred changes when allowed."""
        now = time.monotonic()
        if self.blinking:
            if now >= self.err_until:
                self.blinking = False
                if self.on_error_clear:
                    self.on_error_clear()
                self.request(self.restore or "green")
            elif now >= self.next_toggle:
                self.blink_on = not self.blink_on
                self.set("red" if self.blink_on else "off")
                self.next_toggle = now + ERR_BLINK_S
            return
        if self.pending and now >= self.hold_until:
            color, hold = self.pending
            self.pending = None
            self.request(color, hold)

    def _write(self, name, value):
        path = os.path.join(LED_SYSFS, LED_NAMES[name], "brightness")
        with open(path, "w") as f:
            f.write("1" if value else "0")

    def set(self, color):
        """Show one of the COLORS patterns (red, green, blue, magenta, cyan)."""
        if color == self.color or not self.ok:
            return
        try:
            for name, on in zip(("red", "green", "blue"), COLORS[color]):
                self._write(name, on)
            self.color = color
        except OSError as e:
            # Missing LED must not stop the measurement run
            print(f"warning: LED control unavailable ({e}); continuing without LED",
                  file=sys.stderr, flush=True)
            self.ok = False


class VoltageMonitor:
    """Baseline from the first three readings; colour from the rolling 3-average."""

    def __init__(self, led, rep):
        self.led = led
        self.rep = rep
        self.window = collections.deque(maxlen=AVG_COUNT)
        self.baseline = None
        self.average = None            # latest average of three readings
        self.color = "green"           # colour the voltage rule last chose
        self.on_baseline = None        # optional callback, called once

    def add(self, volts):
        self.window.append(volts)
        if len(self.window) < AVG_COUNT:
            return
        self.average = sum(self.window) / AVG_COUNT

        if self.baseline is None:
            self.baseline = self.average
            self.rep.emit("BASELINE", f"{self.baseline:.2f}")
            if self.on_baseline:
                self.on_baseline()
            return

        drop = self.baseline - self.average
        if drop > RED_DROP_V:
            color = "red"
        elif drop > BLUE_DROP_V:
            color = "blue"
        else:
            color = "green"

        # Only act on a change of the voltage rule's own colour, so a button or
        # GPIO colour is not overwritten every 0.5 s.
        if color != self.color:
            self.color = color
            self.rep.emit("LED", f"{color} avg={self.average:.2f} drop={drop:.2f}")
            self.led.request(color)


def find_gpiochip(gpiod, label):
    """Return the /dev/gpiochipN path whose label matches, or None."""
    for path in sorted(glob.glob("/dev/gpiochip*")):
        try:
            with gpiod.Chip(path) as chip:
                if chip.get_info().label == label:
                    return path
        except OSError:
            continue
    return None


class SomInput:
    """Active-low SoM button on GPIO5_IO20; sets the LED on press/release."""

    def __init__(self, led, rep):
        self.led = led
        self.rep = rep
        self.req = None
        self.fd = None
        try:
            import gpiod
            from gpiod.line import Direction, Edge
        except ImportError as e:
            self._warn(f"python3 gpiod not available ({e})")
            return
        if not hasattr(gpiod, "request_lines"):
            self._warn("gpiod bindings are libgpiod v1; v2 is required")
            return

        path = find_gpiochip(gpiod, SOM_GPIO_CHIP_LABEL)
        if path is None:
            self._warn(f"no gpiochip labelled {SOM_GPIO_CHIP_LABEL}")
            return

        settings = dict(direction=Direction.INPUT, edge_detection=Edge.BOTH)
        try:
            try:
                cfg = gpiod.LineSettings(
                    debounce_period=datetime.timedelta(milliseconds=SOM_GPIO_DEBOUNCE_MS),
                    **settings)
                self.req = gpiod.request_lines(path, consumer="bq35100_run",
                                               config={SOM_GPIO_LINE: cfg})
            except OSError:
                # debounce not supported - request without it
                self.req = gpiod.request_lines(path, consumer="bq35100_run",
                                               config={SOM_GPIO_LINE: gpiod.LineSettings(**settings)})
        except OSError as e:
            self._warn(f"cannot request GPIO5_IO20 on {path} ({e})")
            return
        self.fd = self.req.fd
        self.rising = gpiod.EdgeEvent.Type.RISING_EDGE

    @staticmethod
    def _warn(msg):
        print(f"warning: {msg}; continuing without GPIO5_IO20 input",
              file=sys.stderr, flush=True)

    def handle(self):
        for ev in self.req.read_edge_events():
            if ev.event_type == self.rising:      # back high = released
                self.rep.emit("SOM_BTN", "released")
                self.led.request("green", hold=True)
            else:                                  # pulled low = pressed
                self.rep.emit("SOM_BTN", "pressed")
                self.led.request("cyan", hold=True)

    def close(self):
        if self.req:
            self.req.release()
            self.req = None


class Link:
    """Raw 115200 8N1 serial link with a line reader."""

    def __init__(self, dev):
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        cc = attrs[6]
        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, [
            0,                                              # iflag: raw
            0,                                              # oflag: raw
            termios.CS8 | termios.CREAD | termios.CLOCAL,   # 8N1, no flow ctl
            0,                                              # lflag: raw
            termios.B115200, termios.B115200, cc])
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.buf = b""
        self.watch = {}                 # extra fd -> callback, serviced while waiting
        self.pollers = []               # called at least every POLL_S while waiting

    POLL_S = 0.05                   # keeps the 0.2 s error blink even

    def add_watch(self, fd, callback):
        self.watch[fd] = callback

    def add_poller(self, callback):
        self.pollers.append(callback)

    def send(self, cmd):
        os.write(self.fd, cmd.encode("ascii"))
        termios.tcdrain(self.fd)

    def readline(self, timeout):
        """Return one stripped line, or None on timeout."""
        end = time.monotonic() + timeout
        while True:
            if b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                return line.decode("ascii", errors="replace").strip()
            remaining = end - time.monotonic()
            if remaining <= 0:
                return None
            wait = min(remaining, self.POLL_S) if self.pollers else remaining
            ready, _, _ = select.select([self.fd, *self.watch], [], [], wait)
            for fd in ready:
                if fd == self.fd:
                    self.buf += os.read(self.fd, 256)
                else:
                    self.watch[fd]()
            for poll in self.pollers:
                poll()

    def close(self):
        os.close(self.fd)


class Reporter:
    def __init__(self, log_path):
        self.log = open(log_path, "a", buffering=1) if log_path else None
        self.monitor = None
        self.led = None

    def emit(self, kind, value=""):
        ts = datetime.datetime.now().isoformat(timespec="milliseconds")
        print(f"{ts} {kind} {value}".rstrip(), flush=True)
        if self.log:
            value = str(value).replace(",", ";")   # keep the CSV to 3 columns
            self.log.write(f"{ts},{kind},{value}\n")

    def handle(self, line):
        if not line:
            return
        volts = parse_voltage(line)
        if volts is not None:
            self.emit("VOLTAGE", f"{volts:.1f}")
            if self.monitor:
                self.monitor.add(volts)
            return
        if line == "ALERT":
            self.emit("ALERT")
        elif line in ("5v on", "5v off"):
            self.emit("5V", line[3:])
        elif line in ("status on", "status off"):
            self.emit("Status", line[7:])
        elif line == "err":
            started = self.led.error() if self.led else True
            if started:                            # log once per error episode
                self.emit("ERR", "PC3 not opposite PB4")
        elif line == "btn+":
            self.emit("BTN", "pressed")
            if self.led:
                self.led.request("magenta", hold=True)   # red + blue
        elif line == "btn-":
            self.emit("BTN", "released")
            if self.led:
                self.led.request("green", hold=True)
        else:
            self.emit("ATTINY", line)

    def close(self):
        if self.log:
            self.log.flush()
            os.fsync(self.log.fileno())
            self.log.close()


def wait_for(link, rep, wanted, timeout):
    """Read lines until one in `wanted`; report everything else."""
    end = time.monotonic() + timeout
    while (remaining := end - time.monotonic()) > 0:
        line = link.readline(remaining)
        if line is None:
            break
        if line in wanted:
            return line
        rep.handle(line)
    return None


CAL_TIMEOUT_S = 30.0


def run_cal(link, rep, cmd, label):
    """Send a calibration command and wait for "CAL DONE..." or "CAL ERR ...".

    Returns True on CAL DONE. All CAL lines are logged as kind CAL.
    """
    rep.emit("CAL", f"sending {label}")
    link.send(cmd)
    end = time.monotonic() + CAL_TIMEOUT_S
    while (remaining := end - time.monotonic()) > 0:
        line = link.readline(remaining)
        if line is None:
            break
        if line.startswith("CAL "):
            rep.emit("CAL", line[4:])
            if line.startswith("CAL DONE"):
                return True
            if line.startswith("CAL ERR"):
                return False
        else:
            rep.handle(line)
    rep.emit("ERROR", f"{label}: no CAL DONE/ERR within {CAL_TIMEOUT_S:.0f} s")
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("seconds", type=float, nargs="?",
                    help="measurement run time in seconds (not needed with --calibrate/--vcal)")
    ap.add_argument("--port", default=DEFAULT_PORT, help=f"serial device (default {DEFAULT_PORT})")
    ap.add_argument("--log", help="append CSV (timestamp,kind,value) to this file")
    ap.add_argument("--no-poweroff", action="store_true",
                    help="skip ~PWR- and poweroff (bench testing)")
    ap.add_argument("--forcebat", action="store_true",
                    help="send ~forcebat+ (ATtiny drives PB5 low) once the baseline is set")
    ap.add_argument("--calibrate", action="store_true",
                    help="one-time gauge setup for 2 cells on VIN (sends ~cal2), then exit")
    ap.add_argument("--vcal", type=int, metavar="MV",
                    help="VIN gain calibration to the measured pack voltage in mV "
                         "(sends ~vcal<MV>), then exit; runs after --calibrate if both given")
    args = ap.parse_args()

    calibrating = args.calibrate or args.vcal is not None
    if args.vcal is not None and not 1 <= args.vcal <= 40000:
        ap.error("--vcal must be 1..40000 mV")
    if not calibrating:
        if args.seconds is None:
            ap.error("seconds is required (or use --calibrate / --vcal)")
        if args.seconds <= 0:
            ap.error("seconds must be > 0")
    if not calibrating and not args.no_poweroff and os.geteuid() != 0:
        ap.error("must run as root to power off (or use --no-poweroff)")

    led = StatusLed()
    led.set("green")                    # first thing: green at start-up

    link = Link(args.port)
    rep = Reporter(args.log)
    rep.led = led
    rep.monitor = VoltageMonitor(led, rep)

    link.add_poller(led.poll)
    led.on_error_clear = lambda: rep.emit("ERR", "cleared")

    if args.forcebat:
        def send_forcebat():
            link.send("~forcebat+")
            rep.emit("FORCEBAT", "+")
        rep.monitor.on_baseline = send_forcebat

    som_in = SomInput(led, rep)
    if som_in.fd is not None:
        link.add_watch(som_in.fd, som_in.handle)

    if calibrating:
        # Calibration only: no measurement run, no power-down
        try:
            ok = True
            if args.calibrate:
                ok = run_cal(link, rep, "~cal2", "~cal2")
            if ok and args.vcal is not None:
                ok = run_cal(link, rep, f"~vcal{args.vcal}\n", f"~vcal{args.vcal}")
            rep.emit("CAL", "complete" if ok else "FAILED")
            return 0 if ok else 1
        finally:
            som_in.close()
            link.close()
            rep.close()

    try:
        # 1. Enable gauge
        link.send("~GE+")
        reply = wait_for(link, rep, {"GE ON", "NOACK"}, 3.0)
        if reply != "GE ON":
            rep.emit("ERROR", f"gauge enable failed ({reply or 'no reply'})")
            link.send("~GE-")                       # make sure GE is low
            wait_for(link, rep, {"GE OFF"}, 15.0)
            return 1
        rep.emit("GE", "ON")

        # 2. Measure
        end = time.monotonic() + args.seconds
        while (remaining := end - time.monotonic()) > 0:
            line = link.readline(remaining)
            if line is not None:
                rep.handle(line)

        # 3. Disable gauge (G_DONE wait can take several seconds)
        link.send("~GE-")
        if wait_for(link, rep, {"GE OFF"}, 15.0):
            rep.emit("GE", "OFF")
        else:
            rep.emit("WARNING", "no GE OFF reply within 15 s")

        if args.no_poweroff:
            return 0

        # 4. Request delayed power-down, then 5. shut down
        rep.emit("PWR", "off requested; Linux poweroff now")
        som_in.close()
        rep.close()
        os.sync()
        link.send("~PWR-")
        link.close()
        link = None

        subprocess.run(["poweroff"], check=False)
        return 0

    finally:
        som_in.close()
        if link:
            link.close()
        rep.close() if rep.log and not rep.log.closed else None


if __name__ == "__main__":
    sys.exit(main())
