# Octopus V1.1 Flash Plan

## Goal
Flash Octopus V1.1 with USB-to-CAN bridge firmware to test motors, bed heater, and thermistors while waiting for Pi 5.

## What's Baked into Firmware (reflash to change)
- Communication mode: USB-to-CAN bridge
- CAN speed: 1000000 (1M)

## What's in printer.cfg (change anytime)
- All pin assignments
- Stepper/heater/fan settings
- Everything else

---

## Menuconfig Settings - Octopus V1.1

```
[*] Enable extra low-level configuration options
    Micro-controller Architecture: STMicroelectronics STM32
    Processor model: STM32F446
    Bootloader offset: 32KiB bootloader
    Clock Reference: 12 MHz crystal
    Communication interface: USB to CAN bus bridge (USB on PA11/PA12)
    CAN bus interface: CAN bus (on PD0/PD1)
    CAN bus speed: 1000000
```

---

## Flash Process (SD Card)

1. Compile firmware on Linux/WSL2
2. Rename `klipper.bin` → `firmware.bin`
3. Copy to FAT32 SD card (≤32GB)
4. Power off Octopus
5. Insert SD card
6. Power on, wait 10 seconds
7. Success if file renamed to `FIRMWARE.CUR`

---

## Status
- [ ] Get Linux environment (WSL2 or live USB)
- [ ] Clone Klipper repo
- [ ] Run menuconfig with above settings
- [ ] Compile (`make`)
- [ ] Flash via SD card
- [ ] Set up temp Linux host for testing
- [ ] Write basic printer.cfg for testing
- [ ] Test steppers, bed heater, thermistors

---

## Later
- Flash SHT36 V3 Max (can do now or after Pi 5 arrives)
- Full printer.cfg with all features
