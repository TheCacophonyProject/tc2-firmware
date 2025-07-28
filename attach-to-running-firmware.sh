#!/usr/bin/env bash

# Trap Ctrl+C to exit cleanly
trap 'echo "Stopping..."; exit 0' INT
echo "Attaching probe"
while true; do
    probe-rs attach ./target/thumbv6m-none-eabi/release/tc2-firmware --chip RP2040 --log-format "{L} {s}" 2>/dev/null
    sleep 0.5
done
