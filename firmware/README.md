# Kalico Firmware for Voron Trident

Pre-built Kalico firmware for the Voron Trident CAN bus setup.

## Firmware Files

| File | Board | Configuration |
|------|-------|---------------|
| `octopus-v1.1-can-bridge.bin` | BTT Octopus V1.1 | USB-to-CAN bridge (STM32F446, 1Mbit CAN) |
| `sht36-v3-max-can.bin` | Mellow Fly SHT36 V3 Max | CAN toolhead (RP2040, 1Mbit CAN) |

## Rebuilding Firmware

Requires Docker. Run from this directory:

```bash
# Build both firmwares
./build-firmware.sh

# Build only Octopus
./build-firmware.sh octopus

# Build only SHT36
./build-firmware.sh sht36
```

## Flashing Instructions

### BTT Octopus V1.1

#### Option 1: SD Card (easiest)
1. Copy `octopus-v1.1-can-bridge.bin` to a FAT32 micro SD card
2. Rename the file to `firmware.bin`
3. Insert SD card into the Octopus
4. Power cycle the board
5. File will be renamed to `FIRMWARE.CUR` on success

#### Option 2: DFU Mode via Windows (STM32CubeProgrammer)

**Requirements:**
- USB-C cable
- STM32CubeProgrammer (download from ST or use installer in `../BIGTREETECH-OCTOPUS-V1.0/Firmware/DFU Update bootloader/install software/`)
- Java Runtime (included in install software folder)

**Steps:**
1. Install Java, then STM32CubeProgrammer
2. Place jumper on **BOOT0** pin (labeled on board)
3. Connect USB-C cable to PC
4. Press and hold **RESET** button for 5 seconds
5. Open STM32CubeProgrammer
6. Select **USB** connection type (right side panel)
7. Click **Refresh** - should show "USB1" with "STM32F4STM32"
8. Click **Connect**
9. Click **Open File** and select `octopus-v1.1-can-bridge.bin`
10. Set start address to `0x08008000` (for 32KB bootloader offset)
11. Click **Download**
12. Wait for "File download complete"
13. Click **Disconnect**
14. Remove BOOT0 jumper
15. Press RESET button

#### Option 3: DFU Mode via Linux (dfu-util)

```bash
# Install dfu-util
sudo apt install dfu-util

# Put board in DFU mode:
# 1. Place jumper on BOOT0 pin
# 2. Connect USB-C cable
# 3. Press RESET button (or power cycle)

# Verify detection
lsusb | grep "0483:df11"
# Should show: "STMicroelectronics STM Device in DFU Mode"

# Flash using make (from kalico directory)
cd ~/kalico
make flash FLASH_DEVICE=0483:df11

# Or flash directly with dfu-util
dfu-util -a 0 -s 0x08008000:leave -D octopus-v1.1-can-bridge.bin

# After flashing:
# 1. Remove BOOT0 jumper
# 2. Press RESET or power cycle
```

### Mellow Fly SHT36 V3 Max

The SHT36 ships with Katapult bootloader pre-installed.

#### Option 1: Flash via CAN (normal method)

```bash
# Find the board UUID
python3 ~/katapult/scripts/flashtool.py -i can0 -q

# Enter Katapult mode: double-click RESET button quickly
# LED will flash to indicate bootloader mode

# Flash
sudo service klipper stop
python3 ~/katapult/scripts/flashtool.py -i can0 -f sht36-v3-max-can.bin -u <YOUR_UUID>
sudo service klipper start
```

#### Option 2: USB Boot Mode (recovery/first flash)

**WARNING:** Do NOT connect USB and 24V CAN power simultaneously!

1. Disconnect CAN cable (24V)
2. Hold **BOOT** button on SHT36
3. Connect USB cable to PC
4. Release BOOT button
5. Verify detection: `lsusb | grep "2e8a:0003"`
6. Flash:
   ```bash
   cd ~/kalico
   make flash FLASH_DEVICE=2e8a:0003
   ```

## Configuration Summary

### Octopus V1.1 (USB-to-CAN Bridge)
- MCU: STM32F446 @ 180MHz
- Bootloader: 32KiB offset
- Crystal: 12MHz
- USB: PA11/PA12
- CAN: PD0/PD1 @ 1,000,000 baud

### SHT36 V3 Max (CAN Toolhead)
- MCU: RP2040
- Bootloader: 16KiB (Katapult)
- CAN RX: gpio1
- CAN TX: gpio0
- CAN Speed: 1,000,000 baud
- Startup pins: !gpio5 (LED off)

## Klipper/Kalico Config

```ini
[mcu]
canbus_uuid: <OCTOPUS_UUID>

[mcu SHT36]
canbus_uuid: <SHT36_UUID>
```

Find UUIDs with:
```bash
~/klippy-env/bin/python ~/kalico/scripts/canbus_query.py can0
```
