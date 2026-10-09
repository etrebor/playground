# bq35100_run.py

Timed battery measurement session for the QSXP-ML81 (i.MX8M Plus) board. The script talks over UART_B to an ATtiny1616 running `bq35100_monitor.c`, which controls a TI BQ35100 primary-battery gauge. It enables the gauge, logs voltage readings and alerts for a set number of seconds, disables the gauge, asks the ATtiny to cut board power, and shuts Linux down.

## How a run works

1. The status LED is set to **green**.
2. `~GE+` is sent; the script waits up to 3 s for `GE ON`.
3. Voltage (`n.nv`, every 0.5 s) and `ALERT` lines are logged for the requested number of seconds. The LED colour follows the voltage (see [Status LED](#status-led)).
4. `~GE-` is sent; the script waits up to 15 s for `GE OFF` while the gauge saves its accumulated data.
5. The log is flushed, filesystems are synced, and `~PWR-` is sent.
6. Linux runs `poweroff`. Ten seconds after `~PWR-`, the ATtiny drives PA4 low and removes board power.

**Recovery:** remove and re-apply power to the ATtiny1616. PA4 defaults high at reset, which powers the board back up.

## Hardware connections

| ATtiny1616 | Connects to | Function |
|---|---|---|
| PA1 (USART0 TX) | QSXP-ML81 `UART_RXD_B` | ATtiny → Linux, 115200 8N1 |
| PA2 (USART0 RX) | QSXP-ML81 `UART_TXD_B` | Linux → ATtiny |
| PA4 | Board power enable | High at reset; low 10 s after `~PWR-` |
| PA5 | Button (N.O. to GND, 10k pull-up) | Debounced; `btn+` on press, `btn-` on release |
| PB0 / PB1 | BQ35100 SCL / SDA | I2C, 100 kHz |
| PB2 | BQ35100 ALERT | Falling edge → `ALERT` message |
| PB3 | BQ35100 GE | Gauge enable |
| PB5 | FORCEBAT / second power control (external pull-up) | Input (released, pulled high) at reset; driven low by `~forcebat+`; switched to output high 10 s after PA4 drops; each PA5 press toggles it (output high → input, otherwise → output high) |
| PC3 | Complement of PB4 | Must always be the opposite level of PB4; otherwise `err` is sent |
| PB4 | 5 V rail sense | Reported as `5v on` / `5v off` after every voltage read. Must not exceed the ATtiny VDD |

| QSXP-ML81 pin | LED |
|---|---|
| SAI2_TXFS / GPIO4_IO24 | Red |
| SAI2_RXD0 / GPIO4_IO23 | Green |
| SAI2_TXC / GPIO4_IO25 | Blue |

| QSXP-ML81 pin | Input |
|---|---|
| I2C4_SCL / GPIO5_IO20 (pin 7) | Button (N.O. to GND, 10k pull-up); pressed = low |

The UART_B pad voltage on the QSXP-ML81 must match the ATtiny1616 supply voltage.

## Prerequisites

### Yocto image

Add Python to `local.conf` (karo-minimal does not include it):

```
IMAGE_INSTALL:append = " python3-core"
```

The script uses only the Python standard library; pyserial is not needed. Check that the rootfs still fits the 128 MB partition after adding Python.

### Status LED devicetree

`imx8mp-status-leds.dtsi` also muxes I2C4_SCL as GPIO5_IO20 (so `&i2c4` must stay disabled) and exposes the LED as `/sys/class/leds/red`, `/sys/class/leds/green` and `/sys/class/leds/blue`.

1. Add the file to `meta-cmp9611-wifi` alongside `imx8mp-cmp9611-wifi.dtsi`, add it to `SRC_URI` in `linux-karo_%.bbappend`, install it into `${S}/arch/arm64/boot/dts/freescale/` in `do_configure:append`, and `#include` it from `imx8mp-qsxp-ml81.dts`.
2. Make sure `CONFIG_LEDS_GPIO=y` is set (add it to `cmp9611-wifi.cfg` if needed).
3. Make sure no other node (for example `&sai2`) claims the SAI2 pads.
4. If the LEDs are wired with the cathode to the SoC pin, change the entries to `GPIO_ACTIVE_LOW`.

Check after boot:

```
ls /sys/class/leds/
echo 1 > /sys/class/leds/green/brightness
echo 0 > /sys/class/leds/green/brightness
```

If the LED files are missing, the script prints a warning and continues without the LED.

### Serial port

The default port is `/dev/ttymxc1`. Confirm which `ttymxcN` is UART_B on your board:

```
dmesg | grep ttymxc
```

Pass a different device with `--port`. Nothing else may use that UART: no getty, no serial console, and no Bluetooth HCI attachment.

### ATtiny1616 firmware

Flash `bq35100_monitor.c` (Microchip Studio, UPDI). The firmware must be the version that replies `GE ON` / `GE OFF` and delays PA4 power-down by 10 s.

## Installation

Copy the script to the board (dropbear SSH is in the image):

```
scp bq35100_run.py root@<board-ip>:/usr/bin/
ssh root@<board-ip> chmod +x /usr/bin/bq35100_run.py
```

## Usage

```
bq35100_run.py SECONDS [--port DEVICE] [--log FILE] [--no-poweroff]
```

| Argument | Description |
|---|---|
| `SECONDS` | Measurement time in seconds (required, > 0) |
| `--port DEVICE` | Serial device for UART_B (default `/dev/ttymxc1`) |
| `--log FILE` | Append CSV records to `FILE` |
| `--no-poweroff` | Stop after `~GE-`; do not send `~PWR-` or power off. For bench testing |
| `--forcebat` | Send `~forcebat+` once, right after the baseline voltage is set |
| `--calibrate` | One-time gauge setup (sends `~cal2`), waits for `CAL DONE` / `CAL ERR`, then exits. `SECONDS` not needed |
| `--vcal MV` | VIN gain calibration to the meter reading in mV (sends `~vcal<MV>`), then exits. Runs after `--calibrate` if both are given |

The script must run as root unless `--no-poweroff` is used.

### Examples

One-time gauge calibration (measure the pack with a meter first; no root needed):

```
bq35100_run.py --calibrate --vcal 6100
```

Exit code 0 means `CAL DONE` was received for every step; 1 means `CAL ERR` or no reply within 30 s.

Bench test for 10 s, no power-down:

```
bq35100_run.py 10 --no-poweroff
```

Normal 60 s run, then power off:

```
bq35100_run.py 60
```

Five-minute run with a CSV log on the user partition:

```
bq35100_run.py 300 --log /home/root/gauge.csv
```

**Always do a `--no-poweroff` run first** on a new setup to confirm the gauge, UART and LED work before letting the script cut power.

## Output

Each event is printed to the console as `timestamp KIND value`. With `--log`, the same event is appended to the file as `timestamp,KIND,value`.

| KIND | Value | Meaning |
|---|---|---|
| `GE` | `ON` / `OFF` | Gauge enabled / disabled and data saved |
| `VOLTAGE` | e.g. `3.6` | Battery voltage in volts, one decimal |
| `BASELINE` | e.g. `3.60` | Average of the first three readings of this run |
| `LED` | `blue avg=2.53 drop=1.07` | LED colour change, with current 3-reading average and drop from baseline |
| `ALERT` | | BQ35100 ALERT pin asserted |
| `FORCEBAT` | `+` | `~forcebat+` sent (with `--forcebat`) |
| `5V` | `on` / `off` | ATtiny PB4 level, after each voltage reading |
| `Status` | `on` / `off` | ATtiny PC3 level, after each `5V` line |
| `ERR` | `PC3 not opposite PB4` / `cleared` | Start and end of an `err` episode from the ATtiny |
| `BTN` | `pressed` / `released` | ATtiny button (PA5) |
| `SOM_BTN` | `pressed` / `released` | SoM button (GPIO5_IO20) |
| `ATTINY` | text | Any other line from the ATtiny (`I2CERR`, unknown text) |
| `ERROR` | text | Gauge did not enable; run aborted |
| `WARNING` | text | No `GE OFF` reply within 15 s |
| `PWR` | text | `~PWR-` sent; poweroff starting |
| `CAL` | text | Calibration progress from the ATtiny (`CAL ...` lines), `complete` or `FAILED` |

Example:

```
2026-10-03T14:02:11.204 GE ON
2026-10-03T14:02:11.702 VOLTAGE 3.6
2026-10-03T14:02:12.202 VOLTAGE 3.6
2026-10-03T14:02:12.702 VOLTAGE 3.6
2026-10-03T14:02:12.703 BASELINE 3.60
...
2026-10-03T14:03:11.330 GE OFF
2026-10-03T14:03:11.335 PWR off requested; Linux poweroff now
```

## Status LED

| LED | Set by |
|---|---|
| Green | Start-up; voltage average back within 1 V of baseline; either button released |
| Blue | 3-reading average more than 1 V below baseline |
| Red | 3-reading average more than 2 V below baseline |
| Red + blue | ATtiny button pressed |
| Green + blue | SoM button pressed |
| Red, blinking (0.2 s) | `err` from the ATtiny: PC3 is not the opposite level of PB4. Stops 1.2 s after the last `err`; LED changes requested meanwhile are applied then |

After any button press or release the LED holds that colour for 3 s (`BUTTON_HOLD_S`). Other LED changes requested during the hold, including a quick release, are deferred, and the most recent one is applied when the hold ends. The voltage rule only changes the LED when its own colour changes.

- The baseline is the average of the first three voltage readings **of the current run**. It is not saved between runs, so a slow decline across many runs will not change the colour; only a drop within one run will.
- The average is a rolling average of the last three readings, so it updates every 0.5 s.
- The voltage colour is not latched: if the average recovers, the LED returns to blue or green.
- Buttons and GPIO5_IO20 are only acted on while the script is running.

Thresholds and LED names are constants at the top of the script (`BLUE_DROP_V`, `RED_DROP_V`, `AVG_COUNT`, `LED_NAMES`).

## Exit codes

| Code | Meaning |
|---|---|
| 0 | Run completed (and poweroff started, unless `--no-poweroff`) |
| 1 | Gauge did not enable (`NOACK` or no reply); GE was set low, no power-down |
| 2 | Invalid arguments, or not run as root without `--no-poweroff` |

## ATtiny serial protocol

Commands (Linux → ATtiny, no line ending needed):

| Command | Action | Reply |
|---|---|---|
| `~GE+` | Raise GE, wait for gauge, send GAUGE_START | `GE ON` or `NOACK` |
| `~GE-` | GAUGE_STOP, wait for G_DONE, lower GE | `GE OFF` |
| `~PWR+` | PA4 high now; cancels a pending power-down; releases PB5 unless `~forcebat+` is active | none |
| `~PWR-` | PA4 low after 10 s, then PB5 switched to output high 10 s later | none |
| `~forcebat+` | PB5 driven low | none |
| `~forcebat-` | PB5 released (input; external pull-up) | none |
| `~cal2` | One-time setup for a 2-cell pack on VIN: unseal, Operation Config A [EXTVCELL]=1, Series Cell Count=2, RESET | `CAL ...` lines, ending `CAL DONE` or `CAL ERR ...` |
| `~vcal<mV>` + LF | Scale VIN Gain so Voltage() matches the meter reading, e.g. `~vcal6100` | `CAL ...` lines, ending `CAL DONE` or `CAL ERR ...` |

Messages (ATtiny → Linux, LF-terminated): `n.nv` (voltage every 0.5 s while GE is on), `ALERT`, `I2CERR`, `GE ON`, `GE OFF`, `NOACK`, `btn+`, `btn-`, `5v on`, `5v off` (after every voltage reading or I2CERR), `status on`, `status off` (PC3, after the 5v line), `err` (after the status line when PC3 equals PB4).

You can test the link by hand with a terminal program (for example `microcom -s 115200 /dev/ttymxc1`, if busybox includes it) and typing `~GE+`.

## Troubleshooting

| Symptom | Check |
|---|---|
| `ERROR gauge enable failed (no reply)` | Wrong `--port`; TX/RX swapped; ATtiny not powered or not flashed; UART used by a getty or console |
| `ERROR gauge enable failed (NOACK)` | ATtiny is talking but the BQ35100 does not answer: I2C wiring, pull-ups, GE wiring |
| `ATTINY I2CERR` lines | Intermittent I2C: pull-up values, cable length, noise |
| Garbled characters | Baud mismatch: check the ATtiny `F_CPU` matches its FREQSEL fuse (20 MHz → 3333333, 16 MHz → 2666667) |
| `warning: ... continuing without GPIO5_IO20 input` | python3 gpiod (v2) missing, chip label not `30240000.gpio`, or the line is in use; check with `gpioinfo` |
| `warning: LED control unavailable` | LED dtsi not included, `CONFIG_LEDS_GPIO` off, or SAI2 pads claimed elsewhere |
| LED colours inverted (on when it should be off) | Change the dtsi entries to `GPIO_ACTIVE_LOW` |
| `WARNING no GE OFF reply within 15 s` | Gauge did not report G_DONE; check the G_DONE bit position against the BQ35100 TRM |
| Board loses power before shutdown finishes | Time `poweroff` on the debug console; if it takes longer than 10 s, increase `PWR_OFF_DELAY_TICKS` in the firmware (each tick is 0.5 s) |
| Board does not come back after power is re-applied | Add an external pull-up on PA4; enable the ATtiny brown-out detector (BODCFG fuse) for slow supply ramps |
