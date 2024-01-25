#!/bin/bash

# Exit if command fails
set -e

device=$1
build_type=$2

if [[ -z $device ]]; then
  echo "please provide device name. Usage \`install.sh [devicename] [--release]\`"
  exit 1
fi

if [[ $build_type == "--release" ]]; then
  echo "Building in release mode..."
  cargo build --release
  scp ./target/thumbv6m-none-eabi/release/tc2-firmware $device:
else
  echo "Building in debug mode..."
  cargo build
  scp ./target/thumbv6m-none-eabi/debug/tc2-firmware $device:
fi

ssh $device 'tc2-hat-rp2040 --elf tc2-firmware'
