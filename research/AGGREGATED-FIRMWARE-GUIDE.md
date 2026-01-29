# Chris Voron Trident - Firmware & Electronics Aggregated Guide

## Hardware Overview

| Component | MCU | Communication | Role |
|-----------|-----|---------------|------|
| **BTT Octopus V1.0** | STM32F446ZET6 | USB-to-CAN bridge | Main board (steppers, bed heater, CPAP fan, MINI 12864) |
| **Mellow Fly SHT36 V3 Max** | RP2040 | CAN bus (1M bps) | Toolhead (extruder, hotend heater, hotend fan 2510, Eddy probe) |
| **MINI 12864** | — | SPI via EXP1/EXP2 | Display on Octopus |
| **Raspberry Pi 5** *(incoming)* | — | USB host | Klipper host (Kalico fork) |
| **RPi Touch Display 2** *(incoming)* | — | DSI/USB | KlipperScreen |

---

## 1. BTT Octopus V1.0 - Firmware Flash

### Klipper/Kalico menuconfig (USB-to-CAN Bridge Mode)

```
Micro-controller Architecture: STMicroelectronics STM32
Processor model:               STM32F446
Bootloader offset:             32KiB bootloader
Clock Reference:               12 MHz crystal
Communication interface:       USB to CAN bus bridge (USB on PA11/PA12)
CAN bus interface:              CAN bus (on PD0/PD1)
CAN bus speed:                  1000000
```

### Build & Flash

```bash
cd ~/kalico    # or ~/klipper
make menuconfig   # set options above
make clean && make -j4
```

**SD Card method (recommended):**
1. Copy `out/klipper.bin` to a FAT32 microSD card
2. Rename to `firmware.bin`
3. Insert into Octopus, power cycle
4. Board flashes automatically (LED blinks during flash)

**DFU method (alternate):**
1. Set BOOT0 jumper
2. Hold RESET ~5 seconds
3. `dfu-util -a 0 -d 0483:df11 --dfuse-address 0x08000000:leave -D out/klipper.bin`
4. Remove BOOT0 jumper, reset

### Important Notes
- **Remove all DIAG jumpers** unless using sensorless homing
- **Remove USB 5V power jumper** (power from PSU only)
- **Factory firmware may turn on heaters at power-on** — flash Klipper immediately
- Cannot reflash Octopus over CAN — always use SD card or USB

### Key Pins (from BTT repo & Kalico configs)

| Function | Pin |
|----------|-----|
| MOTOR0 (X) step/dir/en | PF13 / PF12 / PF14 |
| MOTOR1 (Y) step/dir/en | PG0 / PG1 / PF15 |
| MOTOR2_1 (Z) step/dir/en | PF11 / PG3 / PG5 |
| MOTOR3 (Z1) step/dir/en | PG4 / PC1 / PA0 |
| MOTOR4 (Z2) step/dir/en | PF9 / PF10 / PG2 |
| Bed heater (SSR) | PA1 |
| Bed thermistor | PF3 |
| CAN bus (PD0=RX, PD1=TX) | PD0 / PD1 |
| FAN0 (part cooling) | PA8 |
| FAN1 (hotend fan) | PE5 |
| FAN2 (controller fan) | PD12 |
| FAN3 (CPAP candidate) | PD13 |

### CPAP Fan Wiring
- Use a spare fan header (FAN2 `PD12` or FAN3 `PD13`) for the CPAP PWM signal
- Power the CPAP fan directly from the 24V PSU (not through the board MOSFET if current exceeds header rating)
- The fan headers are PWM-capable, suitable for CPAP signal control
- Consider a MOSFET module for high-current CPAP fans

---

## 2. Mellow Fly SHT36 V3 Max - Firmware Flash

### Klipper/Kalico menuconfig (CAN Mode)

```
Enable extra low-level configuration options
Micro-controller Architecture: Raspberry Pi RP2040
Bootloader offset:             16KiB bootloader
Communication interface:       CAN bus
CAN RX gpio number:            1       ← NOT default (default is 4)
CAN TX gpio number:            0       ← NOT default (default is 5)
CAN bus speed:                  1000000
GPIO pins to set at startup:   !gpio5
```

### Build & Flash

```bash
cd ~/kalico
make menuconfig   # set options above
make clean && make -j4
```

**Over CAN (normal updates — Katapult pre-installed):**
```bash
# Find UUID
python3 ~/kalico/scripts/canbus_query.py can0

# Flash
python3 ~/kalico/lib/canboot/flash_can.py -i can0 -f ./out/klipper.bin -u <UUID>
```

**USB Boot (initial/recovery):**
1. Hold BOOT button on SHT36
2. Connect USB-C cable to Pi
3. `make flash FLASH_DEVICE=2e8a:0003`

**WARNING:** Never connect USB and 24V CAN power simultaneously.

### Katapult Bootloader
- Pre-installed at factory (1M CAN speed)
- Pre-compiled UF2: `https://cdn.mellow.klipper.cn/BL/FLY_SHT36V3_SB2040V3_katapult_CAN_1M.uf2`
- Double-click reset button to enter bootloader mode

### Key Pins

| Function | Pin |
|----------|-----|
| Extruder step/dir/en | gpio7 / gpio6 / !gpio14 |
| TMC2209 UART | gpio15 |
| Hotend heater | gpio23 |
| Hotend thermistor (NTC) | gpio27 |
| PT100/PT1000 (MAX31865) | gpio17 |
| Part cooling fan | gpio21 |
| **Hotend fan (2510)** | **gpio13** |
| Endstop | gpio16 |
| Probe (TAP/Klicky) | gpio22 |
| Neopixel | gpio26 |
| Accelerometer (LIS2DW) CS | gpio12 |

---

## 3. Mellow Eddy Probe (LDC1612 - onboard Max variant)

The LDC1612 is integrated on the SHT36 V3 Max PCB — **no separate firmware needed**. It communicates via I2C to the same RP2040 MCU.

### Klipper Config

```ini
[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
z_offset: 0.8
i2c_address: 43
i2c_mcu: SHT36
i2c_bus: i2c1e
x_offset: 0
y_offset: 0
speed: 40
lift_speed: 5
```

### Calibration Procedure

```
# 1. Enable force move (add to printer.cfg temporarily)
[force_move]
enable_force_move: true

# 2. Set kinematic position
SET_KINEMATIC_POSITION z=80

# 3. Calibrate drive current
LDC_CALIBRATE_DRIVE_CURRENT CHIP=fly_eddy_probe

# 4. Position probe ~20mm above bed center, then:
PROBE_EDDY_CURRENT_CALIBRATE CHIP=fly_eddy_probe
```

Supports rapid bed mesh scanning:
```
BED_MESH_CALIBRATE METHOD=scan SCAN_MODE=rapid
```

---

## 4. CAN Bus Wiring

### Topology

```
[Raspberry Pi 5] --USB--> [BTT Octopus V1.0] --CAN bus--> [Fly SHT36 V3 Max]
                           (USB-to-CAN bridge)
```

**No separate CAN adapter (U2C) needed.** The Octopus acts as the bridge.

### Wiring (4 wires between Octopus and SHT36)

| Wire | From (Octopus) | To (SHT36) |
|------|----------------|------------|
| CANH | CAN_H (PD1) | CAN_H |
| CANL | CAN_L (PD0) | CAN_L |
| +24V | 24V | VIN |
| GND | GND | GND |

### Termination
- **120 ohm at both ends** (Octopus + SHT36)
- SHT36 V3 has onboard 120 ohm (DIP switch controlled)
- Verify: measure ~60 ohms between CANH and CANL with multimeter

### Linux CAN Interface (on Raspberry Pi 5)

```bash
sudo nano /etc/network/interfaces.d/can0
```
```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ifconfig $IFACE txqueuelen 1024
    pre-up ip link set can0 type can bitrate 1000000
    pre-up ip link set can0 txqueuelen 1024
```

### Finding UUIDs

```bash
# Stop Klipper first
sudo systemctl stop klipper

# Query CAN bus
python3 ~/kalico/scripts/canbus_query.py can0
# Returns two UUIDs: one for Octopus, one for SHT36
```

---

## 5. MINI 12864 Display (on Octopus EXP1/EXP2)

Both BTT products — **no cable reversal needed**.

```ini
[display]
lcd_type: uc1701
cs_pin: EXP1_3
a0_pin: EXP1_4
rst_pin: EXP1_5
encoder_pins: ^EXP2_5, ^EXP2_3
click_pin: ^!EXP1_2
contrast: 63
spi_software_mosi_pin: PA7
spi_software_miso_pin: PA6
spi_software_sclk_pin: PA5

[output_pin beeper]
pin: EXP1_1

[neopixel btt_mini12864]
pin: EXP1_6
chain_count: 3
color_order: RGB
initial_RED: 0.4
initial_GREEN: 0.4
initial_BLUE: 0.4
```

---

## 6. printer.cfg Structure (skeleton)

```ini
[mcu]
canbus_uuid: <OCTOPUS_UUID>
canbus_interface: can0

[mcu SHT36]
canbus_uuid: <SHT36_UUID>
canbus_interface: can0

[printer]
kinematics: corexy
max_velocity: 300
max_accel: 3000
max_z_velocity: 15
max_z_accel: 350

# --- Steppers on Octopus ---
[stepper_x]
step_pin: PF13
dir_pin: PF12
enable_pin: !PF14
endstop_pin: SHT36:gpio16   # or toolhead endstop
# ...

# --- Extruder on SHT36 ---
[extruder]
step_pin: SHT36:gpio7
dir_pin: SHT36:gpio6
enable_pin: !SHT36:gpio14
heater_pin: SHT36:gpio23
sensor_pin: SHT36:gpio27
# ...

# --- Fans ---
[fan]                         # Part cooling (on SHT36 or Octopus depending on setup)
pin: SHT36:gpio21

[heater_fan hotend_fan]       # 2510 hotend fan on SHT36
pin: SHT36:gpio13

[fan_generic cpap_fan]        # CPAP on Octopus
pin: PD13                     # FAN3 header
# ...

# --- Probe ---
[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
i2c_mcu: SHT36
i2c_bus: i2c1e
i2c_address: 43
z_offset: 0.8
x_offset: 0
y_offset: 0

# --- Display ---
[display]
lcd_type: uc1701
cs_pin: EXP1_3
a0_pin: EXP1_4
rst_pin: EXP1_5
# ... (see section 5)
```

---

## 7. Kalico-Specific Notes

Using Kalico (Klipper fork) provides these relevant benefits:
- **Modules enabled by default:** `force_move`, `respond`, `exclude_object`
- **Deterministic CAN UUIDs** via custom hash (avoids UUID changes)
- **Enhanced canbus_query.py** — shows all devices even after assignment
- **probe_eddy_current / LDC1612** fully supported
- **Model Predictive Control (MPC)** for heaters (alternative to PID)
- Firmware compilation is identical to Klipper — same menuconfig

---

## 8. Flash Order (Recommended)

1. **Flash Octopus first** (SD card method, USB-to-CAN bridge mode)
2. **Set up CAN interface** on Raspberry Pi
3. **Find Octopus CAN UUID** via `canbus_query.py`
4. **Flash SHT36 V3 Max** over CAN (Katapult pre-installed)
5. **Find SHT36 CAN UUID**
6. **Build printer.cfg** with both UUIDs
7. **Calibrate Eddy probe**
8. **Connect MINI 12864** to EXP1/EXP2

---

## Source References (local)

| Source | Path |
|--------|------|
| Kalico firmware source | `/home/sandra/source/kalico/` |
| Kalico CAN docs | `/home/sandra/source/kalico/docs/CANBUS.md` |
| Kalico Octopus config | `/home/sandra/source/kalico/config/generic-bigtreetech-octopus-v1.1.cfg` |
| Kalico Eddy module | `/home/sandra/source/kalico/klippy/extras/probe_eddy_current.py` |
| Mellow SHT36 V3 flash guide | `/home/sandra/source/mellow-klipper-docs/docs/board/fly_sht36_v3/flash.md` |
| Mellow Eddy config | `/home/sandra/source/mellow-klipper-docs/docs/board/fly_sht36_v3/1612.md` |
| Mellow SHT36 V3 pin cfg | `/home/sandra/source/mellow-klipper-docs/docs/board/fly_sht36_v3/cfg.md` |
| Mellow CAN setup | `/home/sandra/source/mellow-klipper-docs/docs/advanced/can.md` |
| BTT Octopus pin diagram | `/home/sandra/source/BIGTREETECH-OCTOPUS-V1.0/Hardware/BIGTREETECH Octopus - PIN.pdf` |
| BTT Octopus schematic | `/home/sandra/source/BIGTREETECH-OCTOPUS-V1.0/Hardware/BIGTREETECH Octopus - SCH.pdf` |
| BTT Octopus Voron config | `/home/sandra/source/BIGTREETECH-OCTOPUS-V1.0/Octopus works on Voron v2.4/Firmware/Klipper/BTT_OctoPus_Voron2_Config.cfg` |
| Trident assembly manual | `/home/sandra/source/Assembly_Manual_Trident-1.pdf` |
