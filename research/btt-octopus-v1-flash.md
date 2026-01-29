# Flashing Klipper Firmware on BTT Octopus V1.0 for Voron Trident

## 1. MCU Identification

The **BTT Octopus V1.0** uses the **STM32F446ZET6** microcontroller (ARM Cortex-M4, 180 MHz, 512KB flash, 128KB SRAM).

> **Note:** The Octopus V1.1 also uses the STM32F446. The Octopus **Pro** V1.0 may use an STM32F446 or STM32F429 depending on variant. Verify the chip printed on your board before proceeding.

---

## 2. Klipper menuconfig Settings

### Option A: USB Communication (standard, no CAN)

```
cd ~/klipper
make clean
make menuconfig
```

| Setting | Value |
|---|---|
| Enable extra low-level configuration options | **[*]** |
| Micro-controller Architecture | **STMicroelectronics STM32** |
| Processor model | **STM32F446** |
| Bootloader offset | **32KiB bootloader** |
| Clock Reference | **12 MHz crystal** |
| Communication interface | **USB (on PA11/PA12)** |

### Option B: USB-to-CAN Bus Bridge Mode (for CAN toolhead boards)

| Setting | Value |
|---|---|
| Enable extra low-level configuration options | **[*]** |
| Micro-controller Architecture | **STMicroelectronics STM32** |
| Processor model | **STM32F446** |
| Bootloader offset | **32KiB bootloader** |
| Clock Reference | **12 MHz crystal** |
| Communication interface | **USB to CAN bus bridge (USB on PA11/PA12)** |
| CAN bus interface | **CAN bus (on PD0/PD1)** |
| CAN bus speed | **1000000** |

After configuring, press `Q` to exit and `Y` to save, then run:

```bash
make
```

The compiled firmware will be at `~/klipper/out/klipper.bin`.

---

## 3. Flashing Methods

### Method 1: SD Card (simplest, recommended for first flash)

1. Copy `~/klipper/out/klipper.bin` to a FAT32-formatted micro SD card
2. Rename the file to `firmware.bin`
3. Insert the SD card into the Octopus
4. Power cycle the board
5. The board will flash automatically; the file will be renamed to `FIRMWARE.CUR` on success

### Method 2: DFU Mode (via USB)

1. Disconnect all peripherals from the Octopus
2. Install jumper on **BOOT0** (labelled J75 on V1.0)
3. Install jumper on **V_Bus_USBC** (labelled J68) if powering via USB
4. Connect USB-C to the host computer
5. Verify the board is detected:
   ```bash
   lsusb   # Should show 0483:df11 STMicroelectronics STM Device in DFU Mode
   ```
6. Flash:
   ```bash
   make flash FLASH_DEVICE=0483:df11
   ```
7. Remove the BOOT0 jumper and power cycle

### Method 3: USB Flash (board already running Klipper)

```bash
sudo service klipper stop
cd ~/klipper
make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32f446xx_XXXXXXXXXXXX-if00
sudo service klipper start
```

> **Important:** If the board is in CAN bridge mode, you **cannot** flash over CAN (flashing restarts the board into bootloader, killing the CAN interface). Use USB serial via Katapult/CanBoot or SD card instead.

---

## 4. Voron Trident-Specific Notes

### Safety Warning

Some Octopus boards ship with firmware that **turns on all heaters and fans at power-on**. Leave heaters disconnected until Klipper firmware is loaded and verified.

### Jumper Configuration

- **UART mode (TMC2209):** Insert the single green jumper for UART; remove the other three SPI jumpers
- **DIAG jumpers:** Remove all DIAG jumpers to prevent TMC2209 DIAG pins from interfering with endstops
- **USB 5V jumper:** Remove to prevent conflict between USB 5V and the board's DC-DC 5V supply
- **Fan voltage jumpers:** Insert jumpers into V_FUSED positions to set fan voltage to system supply voltage
- **Probe voltage:** Insert jumper into V_FUSED for probe supply

### Official Voron Trident Config

The official Voron Trident Octopus config is available at:
```
https://github.com/VoronDesign/Voron-Trident/blob/main/Firmware/Octopus/Trident_Octopus_Config.cfg
```

Key items to customize in `printer.cfg`:
- MCU serial path (or CAN UUID)
- Thermistor types for hotend and bed
- Leadscrew `rotation_distance` (for Trident Z, typically 8mm for TR8x8 or 2mm for TR8x2)
- Z endstop switch location and `position_endstop` offset
- PID tune values (run `PID_CALIBRATE` after initial setup)

### Wiring Reference

- [Voron Trident BTT Octopus Wiring Guide](https://docs.vorondesign.com/build/electrical/v1_octopus_wiring.html)
- [LDO Voron Trident Wiring Guide](https://docs.ldomotors.com/voron/voron-trident/wiring_guide_rev_a)

---

## 5. CAN Bus Setup with Mellow Fly SHT36 V3

### Architecture

```
Raspberry Pi  --USB-->  BTT Octopus (CAN Bridge)  --CAN H/L-->  Mellow Fly SHT36 V3
```

The Octopus acts as a USB-to-CAN bridge. Both the Octopus MCU and the SHT36 V3 appear as CAN nodes identified by UUID.

### Step-by-Step

#### A. Flash Octopus with CAN Bridge Firmware

Use the **Option B** menuconfig settings from Section 2 above. Flash via SD card or DFU.

#### B. Configure CAN Network on Host

Create `/etc/network/interfaces.d/can0`:

```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ifconfig $IFACE txqueuelen 128
```

Then bring up the interface:

```bash
sudo ifup can0
# or reboot
```

Verify:

```bash
ip -s link show can0
```

#### C. Flash the Mellow Fly SHT36 V3

The SHT36 V3 uses an **RP2040** MCU (check the [official Mellow docs](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/) for your exact version). Flash Klipper or Katapult to it via USB first, then subsequent updates can go over CAN.

Klipper menuconfig for the SHT36 V3:

| Setting | Value |
|---|---|
| Micro-controller Architecture | **Raspberry Pi RP2040** |
| Bootloader offset | **16KiB bootloader** (if using Katapult) |
| Communication interface | **CAN bus** |
| CAN bus speed | **1000000** (must match Octopus) |
| CAN RX/TX GPIO pins | Check Mellow docs for V3 pinout |

#### D. Discover CAN UUIDs

With the Octopus flashed in bridge mode and the SHT36 V3 on the CAN bus:

```bash
~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
```

This will output UUIDs for both boards, for example:

```
Found canbus_uuid=aabbccddee11, Application: Klipper
Found canbus_uuid=ff00112233aa, Application: Klipper
```

#### E. Configure printer.cfg

```ini
[mcu]
canbus_uuid: aabbccddee11    # Octopus UUID

[mcu sht36]
canbus_uuid: ff00112233aa    # SHT36 V3 UUID
```

All SHT36 V3 pins are then referenced with the `sht36:` prefix, e.g.:

```ini
[extruder]
step_pin: sht36:gpio18
dir_pin: sht36:gpio19
enable_pin: !sht36:gpio17
# ... etc, per Mellow pinout docs
```

#### F. CAN Bus Wiring

- Use **twisted pair** wiring (Cat5e 24AWG stranded is ideal)
- One pair for CAN_H and CAN_L
- Additional wires for 24V power to the toolhead board
- Enable the **120 ohm termination resistor** on the SHT36 V3 (jumper on board)
- The Octopus has a built-in 120 ohm termination resistor on its CAN port
- Total: two termination resistors, one at each end of the bus

#### G. Updating Firmware Later

- **SHT36 V3:** Can be updated over CAN using Katapult (formerly CanBoot)
- **Octopus:** Must be updated via USB (SD card or DFU), NOT over CAN, because the bridge itself would go down during flash

---

## Sources

- [Voron Documentation - Octopus Klipper Firmware](https://docs.vorondesign.com/build/software/octopus_klipper.html)
- [Voron Trident BTT Octopus Wiring](https://docs.vorondesign.com/build/electrical/v1_octopus_wiring.html)
- [BTT Octopus V1.0 GitHub - Klipper README](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-V1.0/blob/master/Firmware/Klipper/README.md)
- [Voron Trident Octopus Config](https://github.com/VoronDesign/Voron-Trident/blob/main/Firmware/Octopus/Trident_Octopus_Config.cfg)
- [Klipper CAN Bus Documentation](https://www.klipper3d.org/CANBUS.html)
- [Mellow Fly SHT36 V3 Documentation](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/)
- [Voron Forum - Octopus CAN Bridge Mode](https://forum.vorondesign.com/threads/how-to-configure-klipper-on-octopus-using-can_bridge_mode.1429/)
- [Team FDM - CAN Bus Octopus + SHT36 Tutorial](https://www.teamfdm.com/tutorials/article/101-canbus-octopus-canboot-and-mellow-fly-sht36v2/)
- [Klipper CAN Bus Setup Guide](https://maz0r.github.io/klipper_canbus/)
- [BTT Octopus Wiki](https://bttwiki.com/Octopus.html)
- [LDO Voron Trident Wiring Guide](https://docs.ldomotors.com/voron/voron-trident/wiring_guide_rev_a)
