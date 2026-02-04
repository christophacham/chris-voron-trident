# Flash Plan - Octopus V1.1 + SHT36 V3 Max

## Goal
Flash both boards to test motors, bed heater, thermistors, and toolhead while waiting for Pi 5.

## What's Baked into Firmware (reflash to change)
- Communication mode
- CAN speed: 1000000 (1M)

## What's in printer.cfg (change anytime)
- All pin assignments
- Stepper/heater/fan settings
- Everything else

---

## 1. Octopus V1.1 - USB-to-CAN Bridge

### Menuconfig
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

### Flash (SD Card)
1. Compile firmware on Linux
2. Rename `klipper.bin` → `firmware.bin`
3. Copy to FAT32 SD card (≤32GB)
4. Power off Octopus
5. Insert SD card
6. Power on, wait 10 seconds
7. Success if file renamed to `FIRMWARE.CUR`

---

## 2. SHT36 V3 Max - CAN Toolhead

### Menuconfig
```
[*] Enable extra low-level configuration options
    Micro-controller Architecture: Raspberry Pi RP2040
    Bootloader offset: 16KiB bootloader
    Communication interface: CAN bus
    CAN RX gpio: gpio1
    CAN TX gpio: gpio0
    CAN bus speed: 1000000
```

**Note:** RX=gpio1, TX=gpio0 is swapped from default!

### Flash (USB Boot)
1. Compile firmware on Linux
2. Disconnect 24V power from SHT36
3. Hold BOOT button on SHT36
4. Connect USB cable to PC
5. Release BOOT button - mounts as USB drive
6. Copy `klipper.uf2` to the drive
7. Board auto-reboots when done

---

## Status
- [ ] Get Linux environment ready
- [ ] Clone Klipper repo
- [ ] Compile Octopus firmware (menuconfig + make)
- [ ] Flash Octopus via SD card
- [ ] Compile SHT36 firmware (menuconfig + make)
- [ ] Flash SHT36 via USB boot
- [ ] Set up CAN interface on Linux host
- [ ] Find CAN UUIDs for both boards
- [ ] Write basic printer.cfg for testing
- [ ] Test steppers, bed heater, thermistors
- [ ] Test toolhead (fans, sensors)
