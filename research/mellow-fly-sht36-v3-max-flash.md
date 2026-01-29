# Mellow Fly SHT36 V3 Max - Klipper Firmware Flashing Guide

> Research compiled 2026-01-29. Sources linked throughout.

---

## 1. MCU Chip

The Fly SHT36 V3 (and V3 Max variant) uses a **Raspberry Pi RP2040** - a dual-core ARM Cortex-M0+ running at 133 MHz with 264 KB SRAM. The RP2040 uses a **12 MHz external crystal** (Klipper auto-detects this; no manual clock selection needed in menuconfig).

The "Max" variant adds an onboard **LDC1612** eddy current sensor chip connected via I2C. The standard V3 and V3 Max share the same RP2040 MCU and firmware build process.

---

## 2. Klipper `make menuconfig` Settings

### CAN Bus Mode (recommended)

```
[*] Enable extra low-level configuration options
    Micro-controller Architecture  --->  Raspberry Pi RP2040
    Bootloader offset              --->  16KiB bootloader
    Communication interface        --->  CAN bus
    CAN bus speed                  --->  1000000
(1) CAN RX gpio number                  (change from default 4)
(0) CAN TX gpio number                  (change from default 5)
(!gpio5) GPIO pins to set at micro-controller startup
```

**Critical:** The CAN RX/TX GPIOs are **gpio1** (RX) and **gpio0** (TX) - these differ from the RP2040 defaults of 4 and 5. Getting this wrong means no CAN communication.

The `!gpio5` startup pin keeps the status LED in a known state.

### Serial / RS232 Mode (alternative)

```
    Micro-controller Architecture  --->  Raspberry Pi RP2040
    Bootloader offset              --->  (leave default / No bootloader for USB-direct)
    Communication interface        --->  Serial (on UART0 GPIO1/GPIO0)
    Baud rate                      --->  250000
(!gpio5) GPIO pins to set at micro-controller startup
```

### Build Commands

```bash
cd ~/klipper
make clean && rm -rf .config
make menuconfig    # configure as above, save and exit
make -j4
```

Successful CAN build produces: `out/klipper.bin`
Successful serial/USB build produces: `out/klipper.uf2`

---

## 3. Flashing Methods

### Pre-installed State

The board ships with **Katapult** (formerly CanBoot) pre-flashed, configured for CAN at **1M baud**. You do NOT need to flash Katapult yourself unless you wipe it.

### Method A: Flash Klipper via CAN (normal updates)

This is the standard method once the board is already running Katapult or Klipper on CAN.

1. **Find the board UUID:**
   ```bash
   ~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
   ```
   Or with Katapult's tool:
   ```bash
   python3 ~/katapult/scripts/flashtool.py -i can0 -q
   ```

2. **Enter Katapult mode** - double-click the reset button quickly (LED will flash indicating bootloader mode).

3. **Flash Klipper firmware:**
   ```bash
   sudo service klipper stop
   python3 ~/katapult/scripts/flashtool.py -i can0 -f ~/klipper/out/klipper.bin -u <YOUR_UUID>
   ```
   Or with the older method:
   ```bash
   python3 ~/klipper/lib/canboot/flash_can.py -u <YOUR_UUID>
   ```

4. Success shows: `CAN Flash Success`

5. Restart Klipper:
   ```bash
   sudo service klipper start
   ```

### Method B: Initial USB/DFU Flash (recovery or first-time without Katapult)

1. **Hold the BOOT button** on the SHT36 board.
2. Connect USB cable to host (do NOT have 24V CAN connected simultaneously).
3. Release BOOT button.
4. Verify detection:
   ```bash
   lsusb
   # Should show: 2e8a:0003 Raspberry Pi RP2 Boot
   ```
5. Flash:
   ```bash
   cd ~/klipper
   make flash FLASH_DEVICE=2e8a:0003
   ```

**WARNING:** Do NOT connect USB and the CAN connector (24V) at the same time. Only one power source should be active during flashing.

---

## 4. Mellow Eddy Probe Integration (LDC1612)

The Mellow Eddy probe is **integrated on the SHT36 V3 Max board** via an onboard **LDC1612** inductive sensing chip. It does **NOT** need separate firmware - it communicates over I2C to the same RP2040 MCU that runs Klipper.

### Klipper Configuration

```ini
[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
i2c_address: 43
i2c_mcu: SHT36
i2c_bus: i2c1e
x_offset: 0          # Measure and set your actual X offset
y_offset: 21.42      # Measure and set your actual Y offset
z_offset: 2          # Calibrate and adjust
i2c_speed: 4000000

[temperature_probe fly_eddy_probe]
sensor_type: Generic 3950
sensor_pin: SHT36:gpio28
horizontal_move_z: 2
```

### Important Notes

- Remove any existing `[probe]`, `[bltouch]`, or similar probe sections from your config - only one probe can be active.
- The `x_offset` and `y_offset` must be measured for your specific toolhead mounting.
- The temperature probe section enables thermal drift compensation.
- **Known issue:** Some users report "sensor not in valid range" errors. A workaround involves changing the LDC1612 frequency to `40000000` in `~/klipper/klippy/extras/ldc1612.py`, though check if this has been fixed in newer Klipper builds first.
- Calibration requires: heatbed clear of debris, clean nozzle, manual leveling done first on multi-Z machines.

### Calibration Procedure

Follow the standard Klipper eddy probe calibration:
```
PROBE_EDDY_CURRENT_CALIBRATE CHIP=fly_eddy_probe
```

---

## 5. CAN Bus Configuration

### Bitrate

- **1,000,000 (1M)** - factory default, recommended
- 500,000 also supported
- Must match across the entire CAN bus (host adapter, mainboard bridge, toolhead)

### Termination Resistor

The SHT36 V3 has an onboard 120-ohm termination resistor. CAN bus requires exactly **two** 120-ohm termination resistors - one at each end of the bus. Typically:

- One on the host/mainboard CAN adapter (or the mainboard if it is the CAN bridge)
- One on the SHT36 toolhead board (the far end of the bus)

Check for a jumper or DIP switch on the SHT36 board to enable/disable the termination resistor. Verify proper termination by measuring resistance between CAN-H and CAN-L with power off - should read approximately **60 ohms** (two 120-ohm resistors in parallel).

### Host CAN Interface Setup

```bash
# /etc/network/interfaces.d/can0
auto can0
iface can0 can static
    bitrate 1000000
    up ifconfig $IFACE txqueuelen 1024
```

Or via systemd:
```ini
# /etc/systemd/network/80-can.network
[Match]
Name=can0

[CAN]
BitRate=1M
```

### Klipper MCU Section

```ini
[mcu SHT36]
canbus_uuid: <YOUR_UUID>    # obtained from canbus_query.py
```

---

## 6. Pin Assignments

### Hotend Heater

```ini
[extruder]
heater_pin: SHT36:gpio23
```

### Thermistor / Temperature Sensor

```ini
# Standard NTC thermistor (e.g., ATC Semitec 104GT-2)
[extruder]
sensor_type: ATC Semitec 104GT-2
sensor_pin: SHT36:gpio27

# PT1000 (direct, no amplifier)
# sensor_type: PT1000
# sensor_pin: SHT36:gpio27

# PT100/PT1000 via onboard MAX31865 (if equipped)
# sensor_type: MAX31865
# sensor_pin: SHT36:gpio17  # verify CS pin for your board revision
# spi_bus: spi0_gpio4_gpio3_gpio2
```

### Fans

```ini
# Part cooling fan (PWM controlled, typically 5015 blower)
[fan]
pin: SHT36:gpio21

# Hotend cooling fan (2510, always-on when hot)
[heater_fan hotend_fan]
pin: SHT36:gpio13
heater: extruder
heater_temp: 50.0
```

Fan voltage is selectable via jumper: **5V, 12V, or Vin (24V)**. Set the jumper according to your fan voltage rating. The MOSFET orientation on the board does not matter.

### Extruder Stepper (TMC2209 onboard)

```ini
[extruder]
step_pin: SHT36:gpio7
dir_pin: SHT36:gpio6
enable_pin: !SHT36:gpio14
microsteps: 16
rotation_distance: 22.6789511  # calibrate for your extruder
gear_ratio: 50:10              # e.g., Clockwork 2 / Stealthburner
nozzle_diameter: 0.400
filament_diameter: 1.750

[tmc2209 extruder]
uart_pin: SHT36:gpio15
run_current: 0.6
stealthchop_threshold: 999999
```

### Additional Pins

```ini
# Endstop (e.g., X endstop on toolhead)
[stepper_x]
endstop_pin: SHT36:gpio16

# Probe / BLTouch signal
# probe_pin: SHT36:gpio22
# BLTouch servo: SHT36:gpio24

# Stealthburner LEDs (Neopixel)
[neopixel sb_leds]
pin: SHT36:gpio26
chain_count: 3
color_order: GRB

# Onboard accelerometer (LIS2DW)
[lis2dw]
cs_pin: SHT36:gpio12
spi_bus: spi0_gpio4_gpio3_gpio2

# Eddy probe temperature (gpio28, see Section 4)
# SHT36:gpio28

# MCU temperature monitoring
[temperature_sensor SHT36]
sensor_type: temperature_mcu
sensor_mcu: SHT36
```

### Pin Summary Table

| Function | Pin |
|---|---|
| Hotend heater | `SHT36:gpio23` |
| Thermistor (NTC/PT1000) | `SHT36:gpio27` |
| Part cooling fan | `SHT36:gpio21` |
| Hotend cooling fan | `SHT36:gpio13` |
| Extruder step | `SHT36:gpio7` |
| Extruder dir | `SHT36:gpio6` |
| Extruder enable | `!SHT36:gpio14` |
| Extruder UART (TMC2209) | `SHT36:gpio15` |
| Endstop | `SHT36:gpio16` |
| Probe signal | `SHT36:gpio22` |
| BLTouch servo | `SHT36:gpio24` |
| Neopixel LEDs | `SHT36:gpio26` |
| Accelerometer CS | `SHT36:gpio12` |
| Eddy temp sensor | `SHT36:gpio28` |
| CAN RX | `gpio1` |
| CAN TX | `gpio0` |

---

## 7. Katapult (CanBoot) Bootloader

### Overview

Katapult is a lightweight bootloader that enables firmware updates over CAN bus without physical access to the board. The SHT36 V3 **ships with Katapult pre-installed** at 1M CAN baud.

### If You Need to Re-flash Katapult

Mellow provides pre-compiled Katapult UF2 files:

```bash
# Flash pre-compiled Katapult for CAN at 1M
sudo ~/klipper/lib/rp2040_flash/rp2040_flash ~/FLY_Katapult/BL/SHT36/FLY_SHT36V3_katapult_CAN_1M.uf2
```

Or build from source:

```bash
cd ~/katapult
make clean && make menuconfig
```

Katapult menuconfig for CAN:
```
    Micro-controller Architecture  --->  Raspberry Pi RP2040
    Build Katapult deployment application  ---> 16KiB bootloader
    Communication interface        --->  CAN bus
(1) CAN RX gpio number
(0) CAN TX gpio number
    CAN bus speed                  --->  1000000
```

Then build and flash via USB boot mode (hold BOOT, connect USB):
```bash
make -j4
make flash FLASH_DEVICE=2e8a:0003
```

**After flashing Katapult, fully power-cycle the board once.**

### Entering Katapult Mode

- **Double-click** the reset button quickly, OR
- **Quickly cycle power twice**
- An LED will flash to indicate the board is in Katapult bootloader mode
- The board will show as `Application: Katapult` when queried

### Key Points

- Katapult is ONLY a bootloader - it cannot run Klipper. You must flash Klipper on top of it.
- Some users report that the `canboot.bin` file works better than `katapult.bin` when building from source (they are functionally identical, just renamed).
- Katapult may cause issues with KlipperScreen (homing timeouts reported). If this occurs, consider flashing Klipper directly without the bootloader for USB mode.

---

## Sources

- [Mellow Official FLY Docs - SHT36 V3 Introduction](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/)
- [Mellow Official FLY Docs - SHT36 V3 Configuration](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/cfg/)
- [Mellow Official FLY Docs - SHT36 V3 Eddy Usage](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/1612/)
- [Mellow GitHub - flash.md](https://github.com/Mellow-3D/klipper-docs/blob/master/docs/board/fly_sht36_v3/flash.md)
- [Mellow GitHub - cfg.md](https://github.com/Mellow-3D/klipper-docs/blob/master/docs/board/fly_sht36_v3/cfg.md)
- [SHT36 Max V3 General Info (TeamGloomy/RRF)](https://teamgloomy.github.io/fly_sht36_max_v3_general.html)
- [SHT36 Max V3 Pin Names (TeamGloomy/RRF)](https://teamgloomy.github.io/fly_sht36_max_v3_pins.html)
- [Community Gist - SHT36 V3 Setup](https://gist.github.com/ammmze/123a5ef5189935596847848a76236dbe)
- [Klipper Eddy Probe Documentation](https://www.klipper3d.org/Eddy_Probe.html)
- [Katapult (CanBoot) Repository](https://github.com/Arksine/katapult)
- [Mellow SHT36 V3 on Amazon (product listing confirming LDC1612 Eddy)](https://www.amazon.com/Mellow-Klipper-Compatible-Printer-LDC1612/dp/B0DR71JMFZ)
