#!/bin/sh
cargo build --release && arm-none-eabi-objcopy -O binary target/thumbv6m-none-eabi/release/braille-pas braille-pas.bin && st-flash write braille-pas.bin 0x8000000 && du -b braille-pas.bin
