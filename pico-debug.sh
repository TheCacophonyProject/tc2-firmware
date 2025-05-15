
#!/bin/bash

# Exit if command fails
set -e

cargo build

probe-rs run ./target/thumbv6m-none-eabi/debug/tc2-firmware --chip RP2040
