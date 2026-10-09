#!/bin/sh
# usb_integrity.sh - write a pattern through the USB host port to the OTG
# mass-storage gadget, read it back and compare checksums.
#
#   sh usb_integrity.sh [DEVICE] [SIZE_MB]
#     DEVICE   block device of the gadget as seen by the host (default /dev/sda)
#     SIZE_MB  size of a NEW /tmp/pattern (default 32; ignored if it exists)
#
# Setup (once): gadget loaded, loopback cable USBH <-> OTG connected
#   dd if=/dev/zero of=/mnt/userfs/usbtest.img bs=1M count=64
#   modprobe g_mass_storage file=/mnt/userfs/usbtest.img removable=1
#
# WARNING: overwrites the start of DEVICE. Refuses anything not on USB.

DEV=${1:-/dev/sda}
SIZE_MB=${2:-32}
PATTERN=/tmp/pattern
READBACK=/tmp/readback
NAME=$(basename "$DEV")

fail() { echo "ERROR: $*"; exit 1; }

# --- wait for the device (only delay in the script: enumeration after plug-in)
i=0
while [ ! -b "$DEV" ] && [ $i -lt 10 ]; do
    [ $i -eq 0 ] && echo "waiting for $DEV ..."
    sleep 1
    i=$((i + 1))
done
[ -b "$DEV" ] || fail "$DEV not present - gadget loaded? cable connected? (check dmesg)"

# --- safety: must be a USB block device (never the eMMC)
readlink -f "/sys/block/$NAME" | grep -q "/usb" || fail "$DEV is not a USB device - refusing to write"

# --- pattern: reuse if present, else create
if [ -f "$PATTERN" ]; then
    echo "using existing $PATTERN"
else
    echo "creating $PATTERN (${SIZE_MB} MB random)"
    dd if=/dev/urandom of="$PATTERN" bs=1M count="$SIZE_MB" 2>/dev/null || fail "could not create $PATTERN"
fi
BYTES=$(wc -c < "$PATTERN")
BLOCKS=$(( (BYTES + 1048575) / 1048576 ))

# --- device must be large enough
DEV_BYTES=$(( $(cat "/sys/block/$NAME/size") * 512 ))
[ "$DEV_BYTES" -ge "$BYTES" ] || fail "$DEV is $DEV_BYTES bytes, pattern is $BYTES bytes"

# --- old readback out of the way
[ -f "$READBACK" ] && rm -f "$READBACK" && echo "removed old $READBACK"

# --- write through USBH, flush, drop page cache so the read really hits USB
echo "writing $BYTES bytes to $DEV ..."
dd if="$PATTERN" of="$DEV" bs=1M 2>&1 | tail -1
sync
echo 3 > /proc/sys/vm/drop_caches

# --- read back exactly the pattern length
echo "reading back ..."
dd if="$DEV" of="$READBACK" bs=1M count="$BLOCKS" 2>&1 | tail -1
truncate -s "$BYTES" "$READBACK" 2>/dev/null || \
    { head -c "$BYTES" "$READBACK" > "$READBACK.tmp" && mv "$READBACK.tmp" "$READBACK"; }

# --- compare
S1=$(md5sum < "$PATTERN" | cut -d' ' -f1)
S2=$(md5sum < "$READBACK" | cut -d' ' -f1)
echo "pattern  $S1"
echo "readback $S2"
if [ "$S1" = "$S2" ]; then
    echo "Files compare"
    exit 0
else
    echo "Files differ"
    exit 1
fi
