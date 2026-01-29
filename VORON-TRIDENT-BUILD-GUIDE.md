# Chris's Voron Trident 250mm — Complete Build & Configuration Guide

> Last updated: 2026-01-29

---

## Table of Contents

1. [Hardware Overview](#1-hardware-overview)
2. [Electronics Architecture](#2-electronics-architecture)
3. [Firmware Flashing](#3-firmware-flashing)
4. [CAN Bus Setup](#4-can-bus-setup)
5. [Homing Strategy](#5-homing-strategy)
6. [Toolhead Configuration](#6-toolhead-configuration)
7. [Fan Configuration](#7-fan-configuration)
8. [Probe & Leveling](#8-probe--leveling)
9. [Display](#9-display)
10. [Macros & Sensors](#10-macros--sensors)
11. [Non-Planar Printing](#11-non-planar-printing)
12. [Full printer.cfg Starting Point](#12-full-printercfg-starting-point)
13. [Calibration Checklist](#13-calibration-checklist)
14. [Reference Links](#14-reference-links)

---

## 1. Hardware Overview

| Component | Model | Notes |
|---|---|---|
| Frame | Voron Trident 250mm | CopyMaster3D DIY Kit |
| Mainboard | BTT Octopus V1.0 | STM32F446ZET6, USB-to-CAN bridge |
| Toolhead board | Mellow Fly SHT36 V3 Max | RP2040, CAN bus, onboard LDC1612 + LIS2DW + MAX31865 |
| Host | Raspberry Pi 5 | Running Kalico (Klipper fork) |
| Display | Mini 12864 v1.0 | SPI on Octopus EXP1/EXP2 |
| Touchscreen | RPi Touch Display 2 | DSI/USB, KlipperScreen |
| Toolhead | RX Projects V3 | [MakerWorld link](https://makerworld.com/en/models/1781340-rx-projects-v3-the-coolest-voron-toolhead#profileId-2209684) |
| Hotend | Bambu Lab H2S / H2D | Direct drive, internal gearing |
| Part cooling | BigTreeTech CPAP ECO | Centrifugal blower with integrated controller |
| Hotend fan | Delta 2510 5V | On SHT36 gpio13, jumper set to 5V |
| Probe | Fly Eddy (LDC1612) | Onboard SHT36 V3 Max, I2C |
| Z endstop | Sexbolt | Physical nozzle-touch endstop |
| Kinematics | CoreXY | Sensorless X/Y homing, sexbolt Z homing |

### Bambu Lab H2S / H2D Hotend

- **Thermistor:** NTC 100K 3950 (`Generic 3950` in Klipper)
- **Heater cartridge:** 24V, 40W (verify wattage printed on cartridge)
- **Max temp:** ~300C
- **Extruder type:** Direct drive with internal gearing
- **Rotation distance:** Starting point ~4.783 (from provided config — calibrate with [esteps calculator](https://www.service-uplink.de/esteps_cal/calculator.php))
- **Nozzle:** Bambu Lab proprietary — expensive, protect during Z homing

> **Important:** This is NOT a BIQU H2. The Bambu Lab H2S/H2D is a different product entirely.

---

## 2. Electronics Architecture

```
[Raspberry Pi 5] ──USB──> [BTT Octopus V1.0] ──CAN bus──> [Mellow Fly SHT36 V3 Max]
                              (CAN bridge)                         (toolhead)
                                │                                      │
                          Mini 12864 display                    Bambu H2S hotend
                          CPAP ECO (PWM signal)                 Delta 2510 5V fan
                          X/Y/Z stepper motors                  Extruder motor (TMC2209)
                          Bed heater (SSR)                      Fly Eddy (LDC1612)
                          Sexbolt Z endstop                     LIS2DW accelerometer
                                                                Filament sensor
                                                                Neopixel LEDs
```

### CAN Bus Topology

```
Octopus (CAN bridge, 120Ω termination) ───twisted pair─── SHT36 V3 Max (120Ω termination)
```

- 4 wires: CANH, CANL, +24V, GND
- Bus speed: 1,000,000 bps (1M)
- No separate U2C/UTOC adapter needed — Octopus is the bridge
- Verify 60Ω between CANH and CANL with bus unpowered

See: [research/can-bus-setup-overview.md](research/can-bus-setup-overview.md)

---

## 3. Firmware Flashing

### Flash Order

1. Flash **Octopus** first (SD card method)
2. Set up **CAN interface** on Pi
3. Find **Octopus CAN UUID**
4. Flash **SHT36 V3 Max** over CAN (Katapult pre-installed)
5. Find **SHT36 CAN UUID**

### Octopus V1.0 — USB-to-CAN Bridge Mode

```
Micro-controller Architecture: STMicroelectronics STM32
Processor model:               STM32F446
Bootloader offset:             32KiB bootloader
Clock Reference:               12 MHz crystal
Communication interface:       USB to CAN bus bridge (USB on PA11/PA12)
CAN bus interface:              CAN bus (on PD0/PD1)
CAN bus speed:                  1000000
```

```bash
cd ~/kalico
make menuconfig   # set options above
make clean && make -j4
# Copy out/klipper.bin to FAT32 SD card as firmware.bin
# Insert into Octopus, power cycle
```

See: [research/btt-octopus-v1-flash.md](research/btt-octopus-v1-flash.md)

### SHT36 V3 Max — CAN Mode

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

```bash
cd ~/kalico
make menuconfig   # set options above
make clean && make -j4

# Flash over CAN (Katapult pre-installed):
python3 ~/kalico/scripts/canbus_query.py can0    # find UUID
python3 ~/kalico/lib/canboot/flash_can.py -i can0 -f ./out/klipper.bin -u <UUID>
```

**WARNING:** Never connect USB and 24V CAN power simultaneously to the SHT36.

See: [research/mellow-fly-sht36-v3-max-flash.md](research/mellow-fly-sht36-v3-max-flash.md)

---

## 4. CAN Bus Setup

### Linux CAN Interface (Raspberry Pi 5)

Create `/etc/network/interfaces.d/can0`:
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
sudo systemctl stop klipper
python3 ~/kalico/scripts/canbus_query.py can0
# Returns two UUIDs: Octopus + SHT36
# Disconnect SHT36 CAN cable and re-query to identify which is which
```

See: [research/can-bus-setup-overview.md](research/can-bus-setup-overview.md)

---

## 5. Homing Strategy

| Axis | Method | Pin / Config |
|---|---|---|
| X | Sensorless (TMC2209 StallGuard) | `tmc2209_stepper_x:virtual_endstop` |
| Y | Sensorless (TMC2209 StallGuard) | `tmc2209_stepper_y:virtual_endstop` |
| Z | Sexbolt (physical endstop) | Octopus endstop pin (e.g. `PG10`) |

### Why not sensorless Z?

- TMC2209 StallGuard on leadscrews is **dangerous** — too much mechanical advantage, nozzle crashes before stall is detected
- 3 independent Z motors: one stalling while others push **twists the gantry**
- The Fly Eddy (LDC1612) **can** act as a virtual Z endstop but has thermal drift risk — unsafe for the expensive Bambu H2S nozzle

### Sensorless X/Y Configuration

```ini
[stepper_x]
endstop_pin: tmc2209_stepper_x:virtual_endstop
homing_retract_dist: 0
homing_speed: 40

[tmc2209 stepper_x]
diag_pin: ^PG6      # DIAG pin for MOTOR0 on Octopus
driver_SGTHRS: 90   # tune this value (0-255)

[stepper_y]
endstop_pin: tmc2209_stepper_y:virtual_endstop
homing_retract_dist: 0
homing_speed: 40

[tmc2209 stepper_y]
diag_pin: ^PG9      # DIAG pin for MOTOR1 on Octopus
driver_SGTHRS: 90   # tune this value (0-255)
```

> **Note:** Install DIAG jumpers on X and Y motor slots on the Octopus. Remove DIAG jumpers for all other slots.

### Sexbolt Z Configuration

```ini
[stepper_z]
endstop_pin: PG10   # verify pin for your wiring
position_endstop: -0.5  # calibrate this
homing_speed: 8
second_homing_speed: 3
homing_retract_dist: 3
```

See: [research/z-homing-sensorless-research.md](research/z-homing-sensorless-research.md)

---

## 6. Toolhead Configuration

### Pin Map — Mellow Fly SHT36 V3 Max

| Function | Pin |
|---|---|
| Extruder step | `SHT36:gpio7` |
| Extruder dir | `SHT36:gpio6` |
| Extruder enable | `!SHT36:gpio14` |
| TMC2209 UART | `SHT36:gpio15` |
| Hotend heater | `SHT36:gpio23` |
| Thermistor (NTC) | `SHT36:gpio27` |
| MAX31865 CS (PT100/PT1000) | `SHT36:gpio17` |
| Part cooling fan | `SHT36:gpio21` |
| Hotend fan | `SHT36:gpio13` |
| Endstop | `SHT36:gpio16` |
| Probe signal | `SHT36:gpio22` |
| Neopixel | `SHT36:gpio26` |
| Accelerometer (LIS2DW) CS | `SHT36:gpio12` |
| Eddy temp sensor | `SHT36:gpio28` |
| CAN RX / TX | `gpio1` / `gpio0` |

**Source:** [Mellow SHT36 V3 Config Reference](https://github.com/Mellow-3D/klipper-docs/blob/master/docs/board/fly_sht36_v3/cfg.md) — [FLY Docs CAN Firmware](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/firmware/can)

### Extruder (Bambu H2S on SHT36)

```ini
[extruder]
step_pin: SHT36:gpio7
dir_pin: SHT36:gpio6
enable_pin: !SHT36:gpio14
microsteps: 16
rotation_distance: 4.783        # starting point — CALIBRATE
max_extrude_only_distance: 300.0
max_extrude_cross_section: 10.0
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: SHT36:gpio23
sensor_type: Generic 3950
sensor_pin: SHT36:gpio27
min_temp: 0
max_temp: 290
min_extrude_temp: 170
pressure_advance: 0.022         # starting point — CALIBRATE
control: pid                    # run PID_CALIBRATE
pid_Kp: 26.213
pid_Ki: 1.304
pid_Kd: 131.721

[tmc2209 extruder]
uart_pin: SHT36:gpio15
run_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0
```

---

## 7. Fan Configuration

### Hotend Fan — Delta 2510 5V

```ini
[heater_fan hotend_fan]
pin: SHT36:gpio13
heater: extruder
heater_temp: 50.0
fan_speed: 1.0
```

> **Jumper:** Set the SHT36 fan voltage jumper for gpio13 to the **5V** position.

### Part Cooling — BigTreeTech CPAP ECO

The CPAP ECO has an integrated controller board. The PWM signal wire runs from the **Octopus** (not the SHT36) to the CPAP controller. Power comes directly from the 24V PSU.

```ini
[fan]
pin: PD13                    # FAN3 header on Octopus (PWM signal)
# OR if CPAP needs separate enable:
# enable_pin: PD12           # FAN2 header as enable
cycle_time: 0.002
hardware_pwm: True
```

> **Wiring:**
> - PWM signal → Octopus FAN3 (PD13) or another PWM-capable pin
> - +24V → PSU directly (not through Octopus fan MOSFET)
> - GND → Common ground
>
> **Logic level:** Octopus GPIOs output 3.3V. Verify the CPAP ECO accepts 3.3V logic. If not, use a logic level shifter.
>
> **TODO:** Confirm BTT CPAP ECO wiring diagram and PWM logic voltage.

### Controller Fan (electronics bay)

```ini
[controller_fan controller_fan]
pin: PD12                    # FAN2 on Octopus
kick_start_time: 0.5
heater: heater_bed
fan_speed: 0.5
```

---

## 8. Probe & Leveling

### Fly Eddy (LDC1612) — Bed Mesh & Z-Tilt Only

The LDC1612 is integrated on the SHT36 V3 Max PCB. No separate wiring needed — it communicates via I2C.

```ini
[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
z_offset: 0.8               # calibrate
i2c_address: 43
i2c_mcu: SHT36
i2c_bus: i2c1e
x_offset: 0
y_offset: 0
speed: 40
lift_speed: 5

[temperature_probe fly_eddy_probe]
sensor_type: Generic 3950
sensor_pin: SHT36:gpio28
horizontal_move_z: 2
```

> **Do NOT use the Fly Eddy as Z endstop** — thermal drift risk with the Bambu H2S nozzle. Use the sexbolt instead.

### Calibration

```bash
# 1. Enable force move (temporarily)
# [force_move]
# enable_force_move: true

SET_KINEMATIC_POSITION z=80
LDC_CALIBRATE_DRIVE_CURRENT CHIP=fly_eddy_probe
# Position probe ~20mm above bed center:
PROBE_EDDY_CURRENT_CALIBRATE CHIP=fly_eddy_probe
```

### Z-Tilt Adjust (Trident 3-point)

```ini
[z_tilt]
z_positions:
    -50, 18      # front left Z motor
    125, 298     # rear center Z motor
    300, 18      # front right Z motor
speed: 200
horizontal_move_z: 5
retries: 5
retry_tolerance: 0.0075
```

> Adjust `z_positions` for your actual 250mm Trident motor locations.

### Bed Mesh

```ini
[bed_mesh]
speed: 200
horizontal_move_z: 5
mesh_min: 30, 30
mesh_max: 220, 220
probe_count: 25, 25          # eddy allows high-res meshes
algorithm: bicubic
```

Supports rapid scanning:
```
BED_MESH_CALIBRATE METHOD=scan SCAN_MODE=rapid
```

---

## 9. Display

### Mini 12864 v1.0 on Octopus EXP1/EXP2

Both are BTT products — connect straight through, no cable reversal needed.

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

See: [research/AGGREGATED-FIRMWARE-GUIDE.md](research/AGGREGATED-FIRMWARE-GUIDE.md)

---

## 10. Macros & Sensors

### Filament Sensor

```ini
[filament_switch_sensor material_0]
switch_pin: ^SHT36:gpio16       # remap from EBBCan to SHT36 — verify pin
pause_on_runout: true
runout_gcode:
    RESPOND PREFIX="FILAMENT" MSG="Runout detected! Printer paused."

insert_gcode:
    LOAD_FILAMENT
    M117 Filament reloaded
    RESPOND TYPE=echo MSG="Filament Loading"
```

> **TODO:** Verify the correct SHT36 pin for the filament sensor switch. The original config used `EBBCan:PB3` — this must be remapped to an available SHT36 gpio. Check if `gpio16` (endstop) or another pin is appropriate for your wiring.

### Filament Load / Unload

```ini
[gcode_macro LOAD_FILAMENT]
gcode:
    M83
    G92 E0
    G1 E35 F350
    G1 E-1.5 F1500
    G92 E0
    M83
    RESPOND TYPE=echo MSG="Filament Loaded"

[gcode_macro UNLOAD_FILAMENT]
gcode:
    M83
    G92 E0.0
    G1 E-40 F450
    G92 E0.0
    M82
    RESPOND TYPE=echo MSG="Unloading Filament"
```

### Filament Cut

```ini
[gcode_macro FILAMENT_CUT]
description: Cut filament using cutter arm
gcode:
    {% set fast_feed = 15000 %}
    {% set slow_feed = 800 %}
    {% set retract_feed = 3000 %}
    {% set extrude_feed = 600 %}

    G90
    G1 X225 F{fast_feed}
    G1 Y255 F{fast_feed}
    G4 P100
    G1 X247 F{slow_feed}
    G1 X245 F{retract_feed}
    G1 X247 F{slow_feed}
    G1 X245 F{retract_feed}
    G1 X247 F{slow_feed}
    G1 E-2 F{extrude_feed}
    G1 X230 F{retract_feed}
    G1 E8 F{extrude_feed}
    G1 E-14 F{extrude_feed}
    G90
```

> **Note:** XY coordinates (X225, Y255, X247) must be adjusted for your cutter arm position.

---

## 11. Non-Planar Printing

This is the reason for choosing Kalico over upstream Klipper.

### Why Kalico?

- Native support for non-planar toolpaths
- Enhanced probe and kinematics modules
- The Fly Eddy (LDC1612) may need to be **detached** for maximum non-planar Z clearance — the sensor sits under the toolhead and limits how close the nozzle can approach steep angles
- Kalico provides flexibility to switch probing strategies when the Eddy is removed

### Considerations

- Non-planar printing requires specialized slicer output (e.g., from non-planar slicing research)
- The toolhead must have maximum clearance for angled approaches
- When the Fly Eddy is detached, alternative Z probing is needed (sexbolt remains for homing; bed mesh may require manual or alternative probing)

### Related research

- `/home/sandra/source/non-planar/` — Non-planar printing research
- `/home/sandra/source/voron-trident-non-planar/` — Trident-specific non-planar setup

---

## 12. Full printer.cfg Starting Point

```ini
#####################################################################
#  Chris's Voron Trident 250mm — Kalico Configuration
#  Mainboard: BTT Octopus V1.0 (USB-to-CAN bridge)
#  Toolhead:  Mellow Fly SHT36 V3 Max (CAN bus)
#  Hotend:    Bambu Lab H2S / H2D
#  Probe:     Fly Eddy LDC1612 (mesh/tilt only)
#  Z endstop: Sexbolt
#  Homing:    Sensorless X/Y, sexbolt Z
#####################################################################

#####################################################################
#  MCU
#####################################################################

[mcu]
canbus_uuid: <OCTOPUS_UUID>
canbus_interface: can0

[mcu SHT36]
canbus_uuid: <SHT36_UUID>
canbus_interface: can0

#####################################################################
#  PRINTER
#####################################################################

[printer]
kinematics: corexy
max_velocity: 300
max_accel: 3000
max_z_velocity: 15
max_z_accel: 350
square_corner_velocity: 5.0

#####################################################################
#  X STEPPER — Sensorless Homing
#####################################################################

[stepper_x]
step_pin: PF13
dir_pin: PF12
enable_pin: !PF14
rotation_distance: 40
microsteps: 32
full_steps_per_rotation: 200
endstop_pin: tmc2209_stepper_x:virtual_endstop
position_min: 0
position_endstop: 250
position_max: 250
homing_speed: 40
homing_retract_dist: 0
homing_positive_dir: true

[tmc2209 stepper_x]
uart_pin: PC4
interpolate: False
run_current: 0.8
sense_resistor: 0.110
stealthchop_threshold: 0
diag_pin: ^PG6
driver_SGTHRS: 90           # TUNE THIS (0-255)

#####################################################################
#  Y STEPPER — Sensorless Homing
#####################################################################

[stepper_y]
step_pin: PG0
dir_pin: PG1
enable_pin: !PF15
rotation_distance: 40
microsteps: 32
full_steps_per_rotation: 200
endstop_pin: tmc2209_stepper_y:virtual_endstop
position_min: 0
position_endstop: 250
position_max: 250
homing_speed: 40
homing_retract_dist: 0
homing_positive_dir: true

[tmc2209 stepper_y]
uart_pin: PD11
interpolate: False
run_current: 0.8
sense_resistor: 0.110
stealthchop_threshold: 0
diag_pin: ^PG9
driver_SGTHRS: 90           # TUNE THIS (0-255)

#####################################################################
#  Z STEPPERS — Sexbolt Homing, 3 Independent Motors
#####################################################################

[stepper_z]
step_pin: PF11
dir_pin: !PG3
enable_pin: !PG5
rotation_distance: 8         # TR8x8; use 2 for TR8x2
microsteps: 32
full_steps_per_rotation: 200
endstop_pin: PG10            # SEXBOLT — verify pin
position_endstop: -0.5       # CALIBRATE
position_max: 240
position_min: -5
homing_speed: 8
second_homing_speed: 3
homing_retract_dist: 3

[tmc2209 stepper_z]
uart_pin: PC6
interpolate: False
run_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0

[stepper_z1]
step_pin: PG4
dir_pin: !PC1
enable_pin: !PA0
rotation_distance: 8
microsteps: 32
full_steps_per_rotation: 200

[tmc2209 stepper_z1]
uart_pin: PC7
interpolate: False
run_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0

[stepper_z2]
step_pin: PF9
dir_pin: !PF10
enable_pin: !PG2
rotation_distance: 8
microsteps: 32
full_steps_per_rotation: 200

[tmc2209 stepper_z2]
uart_pin: PF2
interpolate: False
run_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#  EXTRUDER — Bambu Lab H2S on SHT36
#####################################################################

[extruder]
step_pin: SHT36:gpio7
dir_pin: SHT36:gpio6
enable_pin: !SHT36:gpio14
microsteps: 16
rotation_distance: 4.783     # CALIBRATE
max_extrude_only_distance: 300.0
max_extrude_cross_section: 10.0
nozzle_diameter: 0.400
filament_diameter: 1.750
heater_pin: SHT36:gpio23
sensor_type: Generic 3950
sensor_pin: SHT36:gpio27
min_temp: 0
max_temp: 290
min_extrude_temp: 170
pressure_advance: 0.022     # CALIBRATE
control: pid
pid_Kp: 26.213              # RUN PID_CALIBRATE
pid_Ki: 1.304
pid_Kd: 131.721

[tmc2209 extruder]
uart_pin: SHT36:gpio15
run_current: 0.6
sense_resistor: 0.110
stealthchop_threshold: 0

#####################################################################
#  BED HEATER
#####################################################################

[heater_bed]
heater_pin: PA1
sensor_pin: PF3
sensor_type: Generic 3950
min_temp: 0
max_temp: 120
control: pid
pid_Kp: 68.738              # RUN PID_CALIBRATE
pid_Ki: 2.766
pid_Kd: 427.080

#####################################################################
#  FANS
#####################################################################

# Part cooling — BTT CPAP ECO (PWM on Octopus)
[fan]
pin: PD13
cycle_time: 0.002
hardware_pwm: True

# Hotend fan — Delta 2510 5V (on SHT36, jumper to 5V)
[heater_fan hotend_fan]
pin: SHT36:gpio13
heater: extruder
heater_temp: 50.0
fan_speed: 1.0

# Electronics bay fan
[controller_fan controller_fan]
pin: PD12
kick_start_time: 0.5
heater: heater_bed
fan_speed: 0.5

#####################################################################
#  PROBE — Fly Eddy LDC1612 (mesh & tilt ONLY)
#####################################################################

[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
z_offset: 0.8               # CALIBRATE
i2c_address: 43
i2c_mcu: SHT36
i2c_bus: i2c1e
x_offset: 0
y_offset: 0
speed: 40
lift_speed: 5

[temperature_probe fly_eddy_probe]
sensor_type: Generic 3950
sensor_pin: SHT36:gpio28
horizontal_move_z: 2

#####################################################################
#  HOMING
#####################################################################

[safe_z_home]
home_xy_position: 125, 125
speed: 100
z_hop: 10

[homing_override]
axes: xyz
set_position_z: 0
gcode:
    {% set home_all = 'X' not in params and 'Y' not in params and 'Z' not in params %}
    G91
    G1 Z5 F600              # lift Z 5mm before homing X/Y
    G90

    {% if home_all or 'X' in params %}
        G28 X
        G1 X125 F6000       # move away from endstop
    {% endif %}

    {% if home_all or 'Y' in params %}
        G28 Y
        G1 Y125 F6000       # move away from endstop
    {% endif %}

    {% if home_all or 'Z' in params %}
        G1 X125 Y125 F6000  # move to center for sexbolt
        G28 Z
        G1 Z10 F600
    {% endif %}

#####################################################################
#  Z-TILT (Trident 3-point leveling)
#####################################################################

[z_tilt]
z_positions:
    -50, 18
    125, 298
    300, 18
speed: 200
horizontal_move_z: 5
retries: 5
retry_tolerance: 0.0075

#####################################################################
#  BED MESH
#####################################################################

[bed_mesh]
speed: 200
horizontal_move_z: 5
mesh_min: 30, 30
mesh_max: 220, 220
probe_count: 25, 25
algorithm: bicubic

#####################################################################
#  ACCELEROMETER
#####################################################################

[lis2dw]
cs_pin: SHT36:gpio12
spi_software_sclk_pin: SHT36:gpio2
spi_software_mosi_pin: SHT36:gpio3
spi_software_miso_pin: SHT36:gpio4

[resonance_tester]
accel_chip: lis2dw
probe_points: 125, 125, 20

#####################################################################
#  TEMPERATURE SENSORS
#####################################################################

[temperature_sensor SHT36]
sensor_type: temperature_mcu
sensor_mcu: SHT36

[temperature_sensor Octopus]
sensor_type: temperature_mcu

#####################################################################
#  DISPLAY — Mini 12864 v1.0
#####################################################################

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

#####################################################################
#  FILAMENT SENSOR
#####################################################################

# TODO: Verify correct SHT36 pin for filament sensor
# Original config used EBBCan:PB3 — remap to available SHT36 gpio
# [filament_switch_sensor material_0]
# switch_pin: ^SHT36:gpio??
# pause_on_runout: true
# runout_gcode:
#     RESPOND PREFIX="FILAMENT" MSG="Runout detected!"

#####################################################################
#  MACROS
#####################################################################

[gcode_macro LOAD_FILAMENT]
gcode:
    M83
    G92 E0
    G1 E35 F350
    G1 E-1.5 F1500
    G92 E0
    M83
    RESPOND TYPE=echo MSG="Filament Loaded"

[gcode_macro UNLOAD_FILAMENT]
gcode:
    M83
    G92 E0.0
    G1 E-40 F450
    G92 E0.0
    M82
    RESPOND TYPE=echo MSG="Unloading Filament"

[gcode_macro FILAMENT_CUT]
description: Cut filament at cutter arm
gcode:
    {% set fast_feed = 15000 %}
    {% set slow_feed = 800 %}
    {% set retract_feed = 3000 %}
    {% set extrude_feed = 600 %}
    G90
    G1 X225 F{fast_feed}
    G1 Y255 F{fast_feed}
    G4 P100
    G1 X247 F{slow_feed}
    G1 X245 F{retract_feed}
    G1 X247 F{slow_feed}
    G1 X245 F{retract_feed}
    G1 X247 F{slow_feed}
    G1 E-2 F{extrude_feed}
    G1 X230 F{retract_feed}
    G1 E8 F{extrude_feed}
    G1 E-14 F{extrude_feed}
    G90

#####################################################################
#  KALICO MODULES
#####################################################################

[respond]
[exclude_object]
[force_move]
enable_force_move: true

[idle_timeout]
timeout: 1800
```

---

## 13. Calibration Checklist

After initial setup and successful communication:

1. [ ] **PID tune hotend:** `PID_CALIBRATE HEATER=extruder TARGET=245`
2. [ ] **PID tune bed:** `PID_CALIBRATE HEATER=heater_bed TARGET=100`
3. [ ] **Calibrate extruder rotation_distance:** Extrude 100mm, measure, recalculate
4. [ ] **Tune sensorless homing thresholds:** Adjust `driver_SGTHRS` for X and Y
5. [ ] **Calibrate sexbolt Z endstop:** Paper test, adjust `position_endstop`
6. [ ] **Calibrate Fly Eddy probe:**
   - `LDC_CALIBRATE_DRIVE_CURRENT CHIP=fly_eddy_probe`
   - `PROBE_EDDY_CURRENT_CALIBRATE CHIP=fly_eddy_probe`
7. [ ] **Run Z-tilt adjust:** `Z_TILT_ADJUST`
8. [ ] **Generate bed mesh:** `BED_MESH_CALIBRATE METHOD=scan SCAN_MODE=rapid`
9. [ ] **Input shaper:** `SHAPER_CALIBRATE`
10. [ ] **Pressure advance:** Tower test or Marlin-style PA test
11. [ ] **Flow calibration:** Single-wall cube method
12. [ ] **Verify filament cut coordinates:** Adjust X225/Y255/X247 for your cutter position

---

## 14. Reference Links

### Local Resources

| Resource | Path |
|---|---|
| Kalico source | `/home/sandra/source/kalico/` |
| BTT Octopus V1.0 repo | `/home/sandra/source/BIGTREETECH-OCTOPUS-V1.0/` |
| Mellow SHT36 docs | `/home/sandra/source/mellow-klipper-docs/` |
| Mini 12864 reference | `/home/sandra/source/MINI-12864/` |
| Non-planar research | `/home/sandra/source/non-planar/` |
| Voron Trident non-planar | `/home/sandra/source/voron-trident-non-planar/` |
| Trident assembly manual | `/home/sandra/source/Assembly_Manual_Trident-1.pdf` |
| Kit BOM | `/home/sandra/source/CopyMaster3D Trident 250mm DIY Kit BOM-V4.pdf` |

### Research in this repo

| Topic | File |
|---|---|
| Aggregated firmware guide | [research/AGGREGATED-FIRMWARE-GUIDE.md](research/AGGREGATED-FIRMWARE-GUIDE.md) |
| Octopus flash guide | [research/btt-octopus-v1-flash.md](research/btt-octopus-v1-flash.md) |
| CAN bus setup | [research/can-bus-setup-overview.md](research/can-bus-setup-overview.md) |
| SHT36 V3 Max flash | [research/mellow-fly-sht36-v3-max-flash.md](research/mellow-fly-sht36-v3-max-flash.md) |
| Z homing research | [research/z-homing-sensorless-research.md](research/z-homing-sensorless-research.md) |

### External Documentation

| Topic | Link |
|---|---|
| Kalico GitHub | https://github.com/KalicoCrew/kalico |
| Klipper docs | https://www.klipper3d.org/ |
| Klipper TMC drivers | https://www.klipper3d.org/TMC_Drivers.html |
| Klipper Eddy Probe | https://www.klipper3d.org/Eddy_Probe.html |
| Voron Trident docs | https://docs.vorondesign.com/ |
| Voron Trident Octopus wiring | https://docs.vorondesign.com/build/electrical/v1_octopus_wiring.html |
| Voron Trident Octopus config | https://github.com/VoronDesign/Voron-Trident/blob/main/Firmware/Octopus/Trident_Octopus_Config.cfg |
| BTT Octopus V1.0 GitHub | https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-V1.0 |
| BTT Octopus wiki | https://bttwiki.com/Octopus.html |
| Mellow FLY SHT36 V3 docs | https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/ |
| Mellow SHT36 V3 config | https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/cfg/ |
| Mellow SHT36 V3 CAN firmware | https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/firmware/can |
| Mellow Eddy usage | https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/1612/ |
| Mellow Eddy troubleshooting | https://mellow.klipper.cn/en/docs/DebugDoc/faq/eddy/ |
| Mellow SHT36 V3 GitHub cfg | https://github.com/Mellow-3D/klipper-docs/blob/master/docs/board/fly_sht36_v3/cfg.md |
| Voron Mini 12864 guide | https://docs.vorondesign.com/build/electrical/mini12864_klipper_guide.html |
| Katapult (CanBoot) repo | https://github.com/Arksine/katapult |
| Extruder calibration calculator | https://www.service-uplink.de/esteps_cal/calculator.php |
| RX Projects V3 toolhead | https://makerworld.com/en/models/1781340-rx-projects-v3-the-coolest-voron-toolhead#profileId-2209684 |

---

### TODO / Open Items

- [ ] Confirm BTT CPAP ECO wiring diagram and PWM logic voltage (3.3V vs 5V)
- [ ] Verify filament sensor pin on SHT36 (remap from EBBCan:PB3)
- [ ] Verify filament unload button pin on SHT36 (remap from EBBCan:PB4)
- [ ] Confirm sexbolt endstop pin on Octopus (assumed PG10)
- [ ] Verify Octopus TMC2209 UART pins match your wiring
- [ ] Confirm DIAG pin assignments for sensorless X/Y (PG6, PG9)
- [ ] Measure actual Z motor positions for z_tilt section
- [ ] Verify leadscrew pitch (TR8x8 = rotation_distance 8, TR8x2 = rotation_distance 2)
- [ ] Calibrate filament cut XY coordinates for your cutter arm
- [ ] Research Kalico non-planar kinematics module configuration
