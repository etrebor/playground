#!/usr/bin/env python3
"""
wifisleep.py - put the CMP9611 WiFi (mlan0) to sleep and resume it.

  python3 wifisleep.py status
  python3 wifisleep.py sleep ps                 # 802.11 power save, stays connected
  python3 wifisleep.py resume ps
  python3 wifisleep.py sleep down               # interface down (radio off, disconnects)
  python3 wifisleep.py resume down
  python3 wifisleep.py sleep off                # unload driver (lowest power)
  python3 wifisleep.py resume off
  python3 wifisleep.py sleep off --for 60       # sleep, then resume automatically

Modes
  ps    chip dozes between AP beacons; link, IP and SSH stay up (needs 'iw')
  down  mlan0 down; wpa_supplicant keeps running and reassociates on resume
  off   init script stop + wpa_supplicant/dhcpcd stopped + moal/mlan unloaded;
        resume runs the init script start (firmware reload, ~seconds)

WiFi is the board's only network. 'down' and 'off' cut SSH. Over SSH they
are refused unless --for is given; the script then detaches, resumes on its
own and logs to /tmp/wifisleep.log. Reconnect after the delay.
"""
import argparse
import os
import shutil
import signal
import subprocess
import sys
import time

IFACE = "mlan0"
INIT_SCRIPT = "/etc/init.d/cmp9611-wifi"
MODULES = ("moal", "mlan")             # unload order: moal first
DAEMONS = ("wpa_supplicant", "dhcpcd")
LOG = "/tmp/wifisleep.log"


def sh(cmd, check=False):
    r = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if check and r.returncode:
        raise RuntimeError(f"'{cmd}' failed: {r.stdout.strip()}")
    return r.returncode, r.stdout.strip()


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


# ------------------------------------------------------------- state queries
def iface_exists():
    return os.path.exists(f"/sys/class/net/{IFACE}")


def operstate():
    try:
        with open(f"/sys/class/net/{IFACE}/operstate") as f:
            return f.read().strip()
    except OSError:
        return "absent"


def ipv4():
    _, out = sh(f"ip -4 -o addr show {IFACE} 2>/dev/null")
    for tok in out.split():
        if "/" in tok and tok[0].isdigit():
            return tok
    return None


def modules_loaded():
    try:
        with open("/proc/modules") as f:
            names = {line.split()[0] for line in f}
    except OSError:
        return []
    return [m for m in MODULES if m in names]


def pids(name):
    out = []
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                with open(f"/proc/{d}/comm") as f:
                    if f.read().strip() == name:
                        out.append(int(d))
            except OSError:
                pass
    return out


def power_save():
    if not shutil.which("iw") or not iface_exists():
        return None
    _, out = sh(f"iw dev {IFACE} get power_save")
    return out.split(":")[-1].strip() if ":" in out else out


def ssid():
    if shutil.which("iw") and iface_exists():
        _, out = sh(f"iw dev {IFACE} link")
        for line in out.splitlines():
            if line.strip().startswith("SSID:"):
                return line.split(":", 1)[1].strip()
    if shutil.which("wpa_cli"):
        _, out = sh(f"wpa_cli -i {IFACE} status 2>/dev/null")
        for line in out.splitlines():
            if line.startswith("ssid="):
                return line[5:]
    return None


def cmd_status(_a=None):
    print(f"interface   {IFACE}: {operstate()}")
    print(f"IPv4        {ipv4() or '-'}")
    print(f"SSID        {ssid() or '-'}")
    ps = power_save()
    print(f"power save  {ps if ps is not None else 'unknown (iw not installed)'}")
    print(f"modules     {', '.join(modules_loaded()) or 'none loaded'}")
    for d in DAEMONS:
        print(f"{d:<11} {'running ' + str(pids(d)) if pids(d) else 'not running'}")


def wait_connected(timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ip = ipv4()
        if operstate() == "up" and ip:
            return time.monotonic() - t0, ip
        time.sleep(0.5)
    return None, None


# ------------------------------------------------------------- sleep / resume
def sleep_mode(mode):
    if mode == "ps":
        if not shutil.which("iw"):
            raise RuntimeError("'iw' not installed - add IMAGE_INSTALL:append = \" iw\"")
        sh(f"iw dev {IFACE} set power_save on", check=True)
        log(f"power save: {power_save()}")
    elif mode == "down":
        sh(f"ip link set {IFACE} down", check=True)
        log(f"{IFACE} down ({operstate()})")
    elif mode == "off":
        if os.path.exists(INIT_SCRIPT):
            rc, out = sh(f"{INIT_SCRIPT} stop")
            log(f"{INIT_SCRIPT} stop -> rc {rc}" + (f": {out}" if out else ""))
        for d in DAEMONS:
            for pid in pids(d):
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
        time.sleep(1)
        if iface_exists():
            sh(f"ip link set {IFACE} down")
        for m in MODULES:
            if m in modules_loaded():
                sh(f"rmmod {m}", check=True)
        log(f"driver unloaded (modules: {', '.join(modules_loaded()) or 'none'}, {IFACE}: {operstate()})")


def resume_mode(mode, timeout):
    if mode == "ps":
        sh(f"iw dev {IFACE} set power_save off", check=True)
        log(f"power save: {power_save()}")
        return
    if mode == "down":
        sh(f"ip link set {IFACE} up", check=True)
    elif mode == "off":
        if not os.path.exists(INIT_SCRIPT):
            raise RuntimeError(f"{INIT_SCRIPT} missing - cannot restore WiFi automatically")
        rc, out = sh(f"{INIT_SCRIPT} start")
        log(f"{INIT_SCRIPT} start -> rc {rc}" + (f": {out}" if out else ""))
    log("waiting for link + IPv4 ...")
    dt, ip = wait_connected(timeout)
    if dt is None:
        log(f"NOT reconnected after {timeout} s (state {operstate()}) - check dmesg / wpa_supplicant")
    else:
        log(f"reconnected in {dt:.1f} s, {IFACE} {ip}, SSID {ssid() or '?'}")


def detach():
    """Survive the SSH session dropping: fork, new session, ignore HUP, log to file."""
    if os.fork():
        print(f"detached - progress in {LOG}; reconnect after the sleep period")
        os._exit(0)
    os.setsid()
    signal.signal(signal.SIGHUP, signal.SIG_IGN)
    fd = os.open(LOG, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
    os.dup2(fd, 1)
    os.dup2(fd, 2)
    devnull = os.open(os.devnull, os.O_RDONLY)
    os.dup2(devnull, 0)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    s = p.add_subparsers(dest="cmd", required=True)
    s.add_parser("status")
    for name in ("sleep", "resume"):
        x = s.add_parser(name)
        x.add_argument("mode", choices=("ps", "down", "off"))
        x.add_argument("--timeout", type=float, default=60, help="reconnect wait, s (default 60)")
        if name == "sleep":
            x.add_argument("--for", dest="duration", type=float,
                           help="resume automatically after this many seconds")
            x.add_argument("--force", action="store_true",
                           help="allow down/off over SSH without --for (you lose the connection)")
    a = p.parse_args()

    if os.geteuid() != 0:
        sys.exit("run as root")
    if a.cmd == "status":
        cmd_status()
        return

    try:
        if a.cmd == "resume":
            resume_mode(a.mode, a.timeout)
            return

        over_ssh = bool(os.environ.get("SSH_CONNECTION") or os.environ.get("SSH_CLIENT"))
        cuts_link = a.mode in ("down", "off")
        if cuts_link and over_ssh and a.duration is None and not a.force:
            sys.exit(f"'{a.mode}' disconnects WiFi - the only network - and this is an SSH session.\n"
                     f"Use --for SECONDS (auto-resume), run it from the serial console, or add --force.")
        if cuts_link and over_ssh and a.duration is not None:
            detach()

        log(f"sleep mode '{a.mode}'")
        sleep_mode(a.mode)
        if a.duration is not None:
            log(f"sleeping {a.duration:g} s")
            time.sleep(a.duration)
            log(f"resume mode '{a.mode}'")
            resume_mode(a.mode, a.timeout)
    except RuntimeError as e:
        log(f"ERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
