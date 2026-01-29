# CAN Bus Setup Overview: Octopus V1.0 + Mellow Fly SHT36 V3 Max

## System Architecture

```
[Raspberry Pi 5] --USB--> [BTT Octopus V1.0 (CAN Bridge)] --CAN bus--> [Mellow Fly SHT36 V3 Max]
                                  |                                            |
                           MINI 12864 display                          2510 hotend fan
                           CPAP fan (PWM signal)                       extruder motor
                           stepper motors (X/Y/Z)                      hotend heater
                           bed heater                                  thermistor
                           endstops                                    accelerometer (LIS2DW)
```

---

## 1. Can the Octopus V1.0 Act as a USB-to-CAN Bridge?

**Yes.** The BTT Octopus V1.0 (STM32F446) fully supports Klipper's "USB to CAN bus bridge" mode. No separate adapter (U2C, UTOC, etc.) is required.

When flashed with CAN bridge firmware, the Octopus:

- No longer appears as a USB serial device (`/dev/serial/by-id/` will be empty for it)
- Appears as a USB CAN adapter visible via `lsusb`
- Creates a `can0` network interface on the host
- Is itself addressed by a `canbus_uuid` (not a serial path)

This eliminates the need for a separate BTT U2C or Mellow UTOC adapter.

### Firmware Compilation (Octopus V1.0 CAN Bridge)

```
make menuconfig
```

Settings:
| Option | Value |
|---|---|
| Micro-controller architecture | STMicroelectronics STM32 |
| Processor model | STM32F446 |
| Bootloader offset | 32KiB bootloader |
| Clock reference | 12 MHz crystal |
| Communication interface | **USB to CAN bus bridge (USB on PA11/PA12)** |
| CAN bus speed | 1000000 |

Then:
```bash
make clean
make
```

Flash via DFU or Katapult (formerly CanBoot). For DFU: install jumper on BOOT0 (J75), connect USB, flash, remove jumper.

### Firmware Compilation (SHT36 V3 Max)

The SHT36 V3 uses an **RP2040** MCU.

```
make menuconfig
```

Settings:
| Option | Value |
|---|---|
| Micro-controller architecture | Raspberry Pi RP2040 |
| Bootloader | 16KiB bootloader |
| Communication interface | CAN bus |
| CAN RX gpio | 1 |
| CAN TX gpio | 0 |
| CAN bus speed | 1000000 |

Flash by holding the BOOT button while plugging in USB-C, then drag the `.uf2` file to the mounted drive.

---

## 2. CAN Bus Wiring (Octopus to SHT36)

### Physical Connections

Only **four wires** run from the Octopus to the SHT36:

| Wire | Description |
|------|-------------|
| CANH | CAN High signal |
| CANL | CAN Low signal |
| +24V | Power supply for SHT36 (from PSU or Octopus) |
| GND  | Ground |

Use **twisted pair** wiring for CANH and CANL. Keep the CAN data lines away from stepper motor wires and high-current paths.

### Termination Resistors

CAN bus requires **120-ohm termination resistors** at each end of the bus.

- The **Octopus V1.0** has a 120-ohm termination resistor that can be enabled via a jumper. Check the board documentation for the exact jumper location.
- The **SHT36 V3 Max** also has a termination resistor jumper. Enable it since the SHT36 is at the far end of the bus.

**Verification:** With both termination resistors enabled and the bus unpowered, measure resistance between CANH and CANL with a multimeter. You should read approximately **60 ohms** (two 120-ohm resistors in parallel).

### Wiring Topology

CAN bus is a **linear bus**, not a star topology. The Octopus should be at one end and the SHT36 at the other, with termination enabled on both. Do not create branches or stubs longer than a few centimeters.

---

## 3. CAN Bus Bitrate

The recommended bitrate is **1,000,000 bps (1 Mbit/s)**. This must match across:

1. Octopus firmware (`make menuconfig` CAN bus speed)
2. SHT36 firmware (`make menuconfig` CAN bus speed)
3. Linux host CAN interface configuration

Both boards must be compiled with the same bitrate or they will not communicate.

---

## 4. Host CAN Interface Configuration

On the Raspberry Pi 5, create the CAN network interface:

```bash
sudo nano /etc/network/interfaces.d/can0
```

Contents:
```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ip link set $IFACE txqueuelen 128
```

Then bring it up:
```bash
sudo ifup can0
```

Verify:
```bash
ip -s link show can0
```

---

## 5. Finding CAN UUIDs

**Important:** Stop Klipper before querying, as running Klipper claims the CAN devices.

```bash
sudo systemctl stop klipper
~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0
```

This will output lines like:
```
Found canbus_uuid=11aa22bb33cc, Application: Klipper
Found canbus_uuid=44dd55ee66ff, Application: Klipper
```

Two UUIDs will appear: one for the Octopus (bridge MCU) and one for the SHT36. The UUID is derived from each chip's unique hardware serial number and is stable across reflashes.

To identify which UUID belongs to which board, you can disconnect the SHT36 CAN cable and re-query -- the remaining UUID is the Octopus.

---

## 6. Basic printer.cfg Structure

```ini
#####################################################################
# Main MCU - BTT Octopus V1.0 (CAN Bridge)
#####################################################################
[mcu]
canbus_uuid: OCTOPUS_UUID_HERE
# Note: NOT serial: -- the Octopus is on CAN bus when in bridge mode

#####################################################################
# Toolhead MCU - Mellow Fly SHT36 V3 Max
#####################################################################
[mcu sht36]
canbus_uuid: SHT36_UUID_HERE

#####################################################################
# Printer Kinematics
#####################################################################
[printer]
kinematics: corexy
max_velocity: 300
max_accel: 3000
max_z_velocity: 15
max_z_accel: 350

#####################################################################
# Steppers (on Octopus)
#####################################################################
[stepper_x]
step_pin: PF13
dir_pin: PF12
enable_pin: !PF14
# ... endstop, rotation_distance, etc.

[stepper_y]
step_pin: PG0
dir_pin: PG1
enable_pin: !PF15
# ...

[stepper_z]
step_pin: PF11
dir_pin: !PG3
enable_pin: !PG5
# ...

#####################################################################
# Extruder (on SHT36)
#####################################################################
[extruder]
step_pin: sht36:gpio7
dir_pin: sht36:gpio6
enable_pin: !sht36:gpio14
heater_pin: sht36:gpio23
sensor_pin: sht36:gpio27
sensor_type: ATC Semitec 104GT-2    # adjust for your thermistor
# ... rotation_distance, nozzle_diameter, etc.

[tmc2209 extruder]
uart_pin: sht36:gpio15
run_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
# Hotend Fan - 2510 (on SHT36)
#####################################################################
[heater_fan hotend_fan]
pin: sht36:gpio5          # check your SHT36 V3 Max pinout
heater: extruder
heater_temp: 50.0
# Fan voltage selectable via jumper on SHT36 (5V, 12V, or Vin)

#####################################################################
# CPAP Part Cooling Fan (signal on Octopus, power separate)
#####################################################################
# See Section 7 below for wiring details
[fan]
pin: PB0                  # PWM-capable GPIO on Octopus (example)
# OR for CPAP with separate controller:
# pin: <pwm_capable_pin>
# enable_pin: <mosfet_pin>
# cycle_time: 0.01
# hardware_pwm: True      # recommended to avoid audible whine

#####################################################################
# Bed Heater (on Octopus)
#####################################################################
[heater_bed]
heater_pin: PA1
sensor_pin: PF3
sensor_type: Generic 3950
# ...

#####################################################################
# Accelerometer (on SHT36)
#####################################################################
[lis2dw]
cs_pin: sht36:gpio12
spi_bus: spi0_gpio4_gpio3_gpio2

[resonance_tester]
accel_chip: lis2dw
probe_points: 150, 150, 20

#####################################################################
# Temperature Sensors
#####################################################################
[temperature_sensor sht36_mcu]
sensor_type: temperature_mcu
sensor_mcu: sht36

[temperature_sensor octopus_mcu]
sensor_type: temperature_mcu

#####################################################################
# Display - MINI 12864 (on Octopus EXP1/EXP2)
#####################################################################
# See Section 8 below
```

---

## 7. CPAP Fan Wiring Considerations

CPAP/centrifugal blower fans (e.g., WS7040, Roborock/Xiaomi fans) require special wiring because they use three-phase BLDC motors with a separate motor controller.

### Wiring Summary

| Connection | Where | Notes |
|------------|-------|-------|
| Fan motor wires (3-phase) | BLDC controller board | Three wires from fan to its controller |
| Controller +24V | PSU directly or always-on 24V | **Not** a standard fan MOSFET header (insufficient current rating) |
| Controller GND | PSU ground | Common ground with MCU |
| PWM signal | PWM-capable GPIO on Octopus | 3.3V or 5V logic; see below |
| Enable (optional) | MOSFET output on Octopus | Controls power on/off separately |

### Key Points

1. **Do NOT use a fan MOSFET header for the PWM signal.** Fan headers on the Octopus are switched-load outputs behind MOSFETs. The CPAP controller needs a direct logic-level PWM signal from a GPIO pin.

2. **3.3V vs 5V logic:** The Octopus GPIOs output 3.3V. Many CPAP controllers expect 5V PWM. At 3.3V you may only achieve ~60% of maximum fan speed. Solutions:
   - Use a **logic level shifter** (3.3V to 5V) inline on the PWM wire
   - Use the **Neopixel data pin** on the Octopus, which outputs 5V logic
   - Some CPAP controllers accept 3.3V fine -- check the controller datasheet

3. **Power delivery:** CPAP fans can draw 2-3A at full speed. Wire the controller's power input directly to the PSU or a sufficiently rated always-on output. Standard fan headers on most MCU boards are rated for only ~1A.

4. **Enable pin:** Use Klipper's `enable_pin:` to control a 24V MOSFET that switches power to the CPAP controller. This prevents the fan from spinning at boot before Klipper initializes.

5. **PWM frequency:** Use `hardware_pwm: True` and a low `cycle_time` (e.g., 0.001) to push the PWM frequency above the audible range and avoid whine.

### Example Klipper Config (CPAP with enable pin)

```ini
[fan]
pin: PE5                  # hardware PWM-capable pin on Octopus
enable_pin: PD15          # MOSFET output for power control
cycle_time: 0.002         # 500 Hz or use hardware_pwm for higher
hardware_pwm: True
```

### Signal Wire Routing

The PWM signal wire runs from the Octopus (in the electronics bay) through the cable chains back to the toolhead area where the CPAP controller is mounted. This is a single low-current signal wire alongside any other wires already in the cable chain. The CPAP fan's power wires should also be routed but can follow a separate path if needed.

---

## 8. MINI 12864 Display on Octopus EXP1/EXP2

### Physical Connection

Connect the display's EXP1 to the Octopus EXP1, and EXP2 to EXP2 using the ribbon cables.

**Connector orientation note:** If both the display and the Octopus are BigTreeTech products, connect straight through -- no connector reversal needed. If the display is a different brand, you may need to reverse the connector housings (flip the plug 180 degrees) on both cables. Refer to the [Voron mini12864 guide](https://docs.vorondesign.com/build/electrical/mini12864_klipper_guide.html) for details.

### Klipper Configuration

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

**Note on Neopixel pin conflict:** If you are using the EXP1_6 pin for the mini12864 Neopixel backlight, you cannot also use that specific Neopixel output for CPAP fan PWM. Plan your pin assignments accordingly.

---

## 9. Startup and Verification Checklist

1. Flash Klipper on Octopus with CAN bridge firmware
2. Flash Klipper on SHT36 V3 Max with CAN firmware (matching bitrate)
3. Wire CANH, CANL, +24V, GND between Octopus and SHT36
4. Enable 120-ohm termination on both boards
5. Verify 60 ohms between CANH and CANL (bus unpowered)
6. Configure `can0` interface on the Raspberry Pi
7. Query UUIDs: `~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0`
8. Add both UUIDs to `printer.cfg`
9. Start Klipper and verify communication: `FIRMWARE_RESTART`

---

## 10. Troubleshooting Tips

- **No UUIDs found:** Check wiring, termination, matching bitrate across all three (host, Octopus firmware, SHT36 firmware). Verify `can0` is up with `ip link show can0`.
- **Only one UUID:** The missing board likely has a firmware/bitrate mismatch or a wiring issue.
- **Octopus not showing as CAN adapter:** Re-flash firmware; verify CAN bridge was selected in menuconfig.
- **Timer too close errors:** Increase `txqueuelen` in the can0 config (try 256 or 1024).
- **Cannot flash Octopus over CAN:** This is expected -- the bridge board kills CAN when it reboots to bootloader. Flash the Octopus via USB/DFU instead.
- **SHT36 firmware updates:** Can be done over CAN using Katapult, or via USB by holding the BOOT button.

---

## Sources

- [Klipper CAN Bus Documentation](https://www.klipper3d.org/CANBUS.html)
- [BTT Octopus V1.0 Klipper Firmware (GitHub)](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-V1.0/blob/master/Firmware/Klipper/README.md)
- [Voron Forum: Octopus CAN Bridge Mode](https://forum.vorondesign.com/threads/how-to-configure-klipper-on-octopus-using-can_bridge_mode.1429/)
- [Voron Octopus Klipper Firmware Docs](https://docs.vorondesign.com/build/software/octopus_klipper.html)
- [Mellow FLY SHT36 V3 Documentation](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/)
- [Mellow SHT36 V3 Config Reference](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/cfg/)
- [SHT36 Max V3 General Info (TeamGloomy)](https://teamgloomy.github.io/fly_sht36_max_v3_general.html)
- [Polar-Ted SHT CAN Board Guide (GitHub)](https://github.com/Polar-Ted/Mellow_SHT_CAN_Boards)
- [Klipper CAN Bus Setup Guide (GitHub)](https://github.com/HRading/klipper_canbus_setup)
- [Voron Mini12864 Display Guide](https://docs.vorondesign.com/build/electrical/mini12864_klipper_guide.html)
- [Voron Trident Octopus Wiring](https://docs.vorondesign.com/build/electrical/v1_octopus_wiring.html)
- [CPAP Fan Wiring Discussion (TeamFDM)](https://www.teamfdm.com/forums/topic/2229-how-do-you-wire-properly-a-cpap-to-use-the-pwm-signal-for-cooling/)
- [CPAP Fan Discussion (Voron Forum)](https://forum.vorondesign.com/threads/going-cpap.1889/)
- [4-Pin PWM Fan on Octopus (Nicholas Sherlock)](https://www.nicksherlock.com/2022/01/driving-a-4-pin-computer-pwm-fan-on-the-btt-octopus-using-klipper/)
- [Klipper CAN Bus Maz0r Guide](https://maz0r.github.io/klipper_canbus/toolhead/sht36-42.html)
