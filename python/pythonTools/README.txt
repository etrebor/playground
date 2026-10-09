QSXP-ML81 carrier - hardware test tools
=======================================

Test scripts for the Ka-Ro QSXP-ML81 (i.MX8M Plus) custom carrier. Each
tool has a matching .txt file with full usage and notes.

Install all to userfs (rootfs is read-only):
  scp -O *.py *.sh *.txt root@<board-ip>:/mnt/userfs/
-O is required: dropbear has no SFTP server.
Reflashing userfs wipes /mnt/userfs - copy the tools back afterwards.


TOOLS
-----
  hwtest.py        I2C scan/read/write, GPIO list/get/set/toggle,
                   SPI loopback and heuristic SPI probe.
                   README: hwtest.txt

  bmi330.py        BMI330 IMU over SPI (/dev/spidev1.0): accel, gyro,
                   temperature to console.
                   README: bmi330.txt

  bmm350.py        BMM350 magnetometer over I2C (bus 2, 0x14): compensated
                   field, heading, temperature; optional data-ready IRQ.
                   README: bmm350.txt

  liquidlens.py    MAX14574 liquid lens driver (bus 1, 0x77): register
                   dump/watch/write; set/step/sweep for meter or scope once
                   LENS_MAP is filled from the Corning datasheet.
                   README: liquidlens.txt

  rgbled.py        RGB status LED (GPIO4 IO24/23/25): off, on, set, test.
                   README: rgbled.txt

  usb_integrity.sh USB host port loopback test into the OTG port
                   (mass-storage gadget), checksum compare.
                   README: usb_integrity.txt


BOARD MAP (as found on target)
------------------------------
  I2C bus 0   PMIC PCA9450C (0x25)        - read only
  I2C bus 1   I2C_A: camera sensor 0x18, EEPROM 0x57, MAX14574 0x77
  I2C bus 2   I2C_B: BMM350 0x14, INT GPIO1_IO27
  SPI         /dev/spidev1.0 = ECSPI1, BMI330
  UART        ttymxc0 = UART2 console, ttymxc1 = UART4,
              ttymxc2 = UART1 = CMP9611 Bluetooth (serdev when BT node active)
  USB         USBH pins 43-45, USB OTG pins 46-48


REQUIREMENTS
------------
  python3, python3-smbus2, python3-spidev, python3-gpiod (libgpiod v2)
  usb_integrity.sh: busybox sh only, plus the g_mass_storage gadget module
  Run as root.
