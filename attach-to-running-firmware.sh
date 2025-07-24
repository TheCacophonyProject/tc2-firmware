#!/usr/bin/env bash

# Trap Ctrl+C to exit cleanly
trap 'echo "Stopping..."; exit 0' INT

while true; do
    echo "Attaching probe"
    probe-rs attach ./target/thumbv6m-none-eabi/release/tc2-firmware --chip RP2040 --log-format "{L} {s}"
    exit_code=$?
    echo "probe-rs exited with code $exit_code, restarting..."
    sleep 0.5
done
