# Continue Setup on Linux

## Current Status
- ✅ Octopus V1.1 flashed (USB-to-CAN bridge, 1M CAN speed)
- ❌ SHT36 V3 Max needs flashing
- ❌ CAN interface not configured
- ❌ No printer.cfg yet

---

## 1. Clone Kalico

```bash
cd ~
git clone https://github.com/KalicoCrew/kalico
cd kalico
```

---

## 2. Compile & Flash SHT36 V3 Max

### Compile
```bash
cd ~/kalico
make clean
make menuconfig
```

**Settings:**
```
[*] Enable extra low-level configuration options
    Micro-controller Architecture: Raspberry Pi RP2040
    Bootloader offset: 16KiB bootloader
    Communication interface: CAN bus
    CAN RX gpio: gpio1
    CAN TX gpio: gpio0
    CAN bus speed: 1000000
```

```bash
make
```

### Flash via USB Boot
1. Disconnect 24V from SHT36
2. Hold BOOT button
3. Connect USB to Linux PC
4. Release BOOT button
5. Mount appears as RPI-RP2

```bash
# Find the mount point
lsblk
# Or it may auto-mount to /media/username/RPI-RP2

# Copy firmware
cp out/klipper.uf2 /media/$USER/RPI-RP2/
# Board auto-reboots when done
```

6. Disconnect USB from SHT36

---

## 3. Set Up CAN Interface

### Install can-utils
```bash
sudo apt update
sudo apt install can-utils
```

### Create CAN interface config
```bash
sudo nano /etc/network/interfaces.d/can0
```

Add:
```
allow-hotplug can0
iface can0 can static
    bitrate 1000000
    up ip link set can0 txqueuelen 1024
```

### Bring up CAN
```bash
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1024
```

Or reboot and it comes up automatically.

---

## 4. Connect Hardware

1. Connect Octopus to Linux PC via USB
2. Power on 24V PSU (this powers Octopus + SHT36 via CAN cable)

---

## 5. Find CAN UUIDs

```bash
cd ~/kalico
python3 scripts/canbus_query.py can0
```

You should see **two UUIDs** - one for Octopus, one for SHT36.

Example output:
```
Found canbus_uuid=abc123def456, Application: Klipper
Found canbus_uuid=789xyz012abc, Application: Klipper
```

Save both UUIDs - you'll need them for printer.cfg.

**If only one shows:** Check CAN wiring, termination resistors (120Ω on both ends).

---

## 6. Install Kalico (Klipper Host)

```bash
cd ~/kalico
./scripts/install-debian.sh
```

Follow prompts. This installs:
- Kalico (klippy service)
- Moonraker (API)
- Web interface

---

## 7. Create printer.cfg

```bash
nano ~/printer_data/config/printer.cfg
```

**Minimal test config:**

```ini
[mcu]
canbus_uuid: OCTOPUS_UUID_HERE

[mcu sht36]
canbus_uuid: SHT36_UUID_HERE

[printer]
kinematics: corexy
max_velocity: 300
max_accel: 3000
max_z_velocity: 15
max_z_accel: 45

#####################################################################
#  STEPPERS - Adjust pins/directions after testing
#####################################################################

[stepper_x]
step_pin: PF13
dir_pin: PF12
enable_pin: !PF14
microsteps: 16
rotation_distance: 40
endstop_pin: tmc2209_stepper_x:virtual_endstop
position_min: 0
position_endstop: 250
position_max: 250
homing_speed: 40
homing_retract_dist: 0

[tmc2209 stepper_x]
uart_pin: PC4
diag_pin: ^PG6
run_current: 0.8
stealthchop_threshold: 0
driver_SGTHRS: 90

[stepper_y]
step_pin: PG0
dir_pin: PG1
enable_pin: !PF15
microsteps: 16
rotation_distance: 40
endstop_pin: tmc2209_stepper_y:virtual_endstop
position_min: 0
position_endstop: 250
position_max: 250
homing_speed: 40
homing_retract_dist: 0

[tmc2209 stepper_y]
uart_pin: PD11
diag_pin: ^PG9
run_current: 0.8
stealthchop_threshold: 0
driver_SGTHRS: 90

[stepper_z]
step_pin: PF11
dir_pin: PG3
enable_pin: !PG5
microsteps: 16
rotation_distance: 4  # Check leadscrew pitch
endstop_pin: PG10  # Sexbolt - verify pin
position_max: 250
position_min: -5
homing_speed: 8

[tmc2209 stepper_z]
uart_pin: PC6
run_current: 0.8
stealthchop_threshold: 0

[stepper_z1]
step_pin: PG4
dir_pin: PC1
enable_pin: !PA2
microsteps: 16
rotation_distance: 4

[tmc2209 stepper_z1]
uart_pin: PC7
run_current: 0.8
stealthchop_threshold: 0

[stepper_z2]
step_pin: PF9
dir_pin: PF10
enable_pin: !PG2
microsteps: 16
rotation_distance: 4

[tmc2209 stepper_z2]
uart_pin: PF2
run_current: 0.8
stealthchop_threshold: 0

#####################################################################
#  EXTRUDER (on SHT36)
#####################################################################

[extruder]
step_pin: sht36:gpio7
dir_pin: sht36:gpio6
enable_pin: !sht36:gpio14
microsteps: 16
rotation_distance: 4.69  # Bambu H2S - calibrate later
nozzle_diameter: 0.4
filament_diameter: 1.75
heater_pin: sht36:gpio23
sensor_pin: sht36:gpio27
sensor_type: ATC Semitec 104NT-4-R025H42G  # Check Bambu thermistor type
control: pid
pid_Kp: 20
pid_Ki: 1
pid_Kd: 100
min_temp: 0
max_temp: 300

[tmc2209 extruder]
uart_pin: sht36:gpio15
run_current: 0.6
stealthchop_threshold: 0

#####################################################################
#  BED HEATER
#####################################################################

[heater_bed]
heater_pin: PA1  # Octopus HE0 or dedicated bed - verify
sensor_pin: PF3  # TB on Octopus
sensor_type: Generic 3950
control: pid
pid_Kp: 50
pid_Ki: 3
pid_Kd: 300
min_temp: 0
max_temp: 120

#####################################################################
#  FANS
#####################################################################

[fan]
pin: sht36:gpio21  # Part cooling

[heater_fan hotend_fan]
pin: sht36:gpio13
heater: extruder
heater_temp: 50.0

#####################################################################
#  TEMPERATURE SENSORS
#####################################################################

[temperature_sensor Octopus]
sensor_type: temperature_mcu

[temperature_sensor SHT36]
sensor_type: temperature_mcu
sensor_mcu: sht36

#####################################################################
#  DISPLAY - Mini 12864 v1.0
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
#  BOARD PINS (Octopus V1.1)
#####################################################################

[board_pins]
aliases:
    EXP1_1=PE8, EXP1_2=PE7,
    EXP1_3=PE9, EXP1_4=PE10,
    EXP1_5=PE12, EXP1_6=PE13,
    EXP1_7=PE14, EXP1_8=PE15,
    EXP1_9=<GND>, EXP1_10=<5V>,
    EXP2_1=PA6, EXP2_2=PA5,
    EXP2_3=PB1, EXP2_4=PA4,
    EXP2_5=PB2, EXP2_6=PA7,
    EXP2_7=PC15, EXP2_8=<RST>,
    EXP2_9=<GND>, EXP2_10=<5V>
```

---

## 8. Test

### Start Klipper
```bash
sudo systemctl start klipper
sudo systemctl status klipper
```

### Check logs
```bash
tail -f ~/printer_data/logs/klippy.log
```

### Once connected, test via console:
```
STEPPER_BUZZ STEPPER=stepper_x
STEPPER_BUZZ STEPPER=stepper_y
STEPPER_BUZZ STEPPER=stepper_z
STEPPER_BUZZ STEPPER=extruder
```

### Test bed heater:
```
SET_HEATER_TEMPERATURE HEATER=heater_bed TARGET=40
```

---

## Troubleshooting

### No CAN UUIDs found
- Check `ip link show can0` - should show UP
- Check wiring: CANH, CANL, GND between Octopus and SHT36
- Check 120Ω termination on both ends (measure ~60Ω between CANH and CANL)
- Both boards must have same CAN speed (1000000)

### Klipper won't connect
- Verify UUIDs in printer.cfg match output of canbus_query.py
- Check `dmesg | tail` for USB/CAN errors

### Display not working
- Display needs Klipper running and connected
- Check EXP1/EXP2 cable orientation

---

## Reference Files in This Repo
- `FLASH-PLAN.md` - Firmware menuconfig settings
- `SHT36-V3-MAX-PINOUT.md` - SHT36 pin reference
- `VORON-TRIDENT-BUILD-GUIDE.md` - Full build documentation
