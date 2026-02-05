# Voron Trident 250mm — CAN Bus Build

Kalico (Klipper fork) config and firmware for a Voron Trident 250mm with CAN bus toolhead.

## What Works

| Component | Status | Notes |
|-----------|--------|-------|
| Octopus V1.1 (MCU) | Working | USB-to-CAN bridge, CAN UUID `0f0788d658b4` |
| CAN bus interface | Working | `can0` at 1Mbit, auto-starts on boot |
| X stepper (B motor) | Working | MOTOR_0, 0.8A, physical endstop, direction verified |
| Y stepper (A motor) | Working | MOTOR_1, 0.8A, physical endstop, direction verified |
| Z0 stepper | Working | MOTOR_2, 0.4A, direction verified |
| Z1 stepper | Working | MOTOR_3, 0.4A, direction verified |
| Z2 stepper | Working | MOTOR_4, 0.4A, direction verified |
| Moonraker + Mainsail | Working | Web UI accessible |
| SHT36 V3 Max (toolhead) | Not yet | Needs Katapult flash + CAN wiring |
| Extruder (Bambu H2S) | Not yet | On SHT36, commented out in config |
| Bed heater | Not yet | Commented out, untested |
| Fans (part/hotend) | Not yet | On SHT36, commented out |
| Electronics fans | Disabled | Too loud, pending quieter replacements |
| Probe (Fly Eddy) | Not yet | On SHT36, commented out |
| Endstop homing | Not yet | X/Y physical endstops wired, Z sexbolt wired, not tested |
| Display (Mini 12864) | Not yet | Not configured |

## Octopus V1.1 Motor Port Mapping

```
MOTOR_0 (PF13/PF12/PF14)  →  stepper_x  (B motor, left)
MOTOR_1 (PG0/PG1/PF15)    →  stepper_y  (A motor, right)
MOTOR_2 (PF11/PG3/PG5)    →  stepper_z  (Z0, front left)
MOTOR_3 (PG4/PC1/PA0)     →  stepper_z1 (Z1, rear center)
MOTOR_4 (PF9/PF10/PG2)    →  stepper_z2 (Z2, front right)
```

## Octopus V1.1 TMC2209 UART Pins

```
MOTOR_0  →  PC4   (stepper_x)
MOTOR_1  →  PD11  (stepper_y)
MOTOR_2  →  PC6   (stepper_z)
MOTOR_3  →  PC7   (stepper_z1)
MOTOR_4  →  PF2   (stepper_z2)
```

## Octopus V1.1 Endstop / Sensor Pins

```
DIAG_0 / PG6   →  stepper_x endstop (physical switch)
DIAG_1 / PG9   →  stepper_y endstop (physical switch)
DIAG_2 / PG10  →  stepper_z endstop (sexbolt)
```

## Octopus V1.1 Other Pins

```
PA3   →  Bed heater (HE1 / BED_OUT)
PF3   →  Bed thermistor (TB)
PE5   →  Electronics fan 1 (FAN0) — disabled, too loud
PD12  →  Electronics fan 2 (FAN1) — disabled, too loud
PD13  →  Electronics fan 3 (FAN2) — disabled, too loud
PD14  →  Fan header 4 (spare)
PD15  →  Fan header 5 (spare)
PA8   →  CPAP part cooling (FAN_EXTRA — not currently used)
```

## SHT36 V3 Max Pins (pending CAN connection)

```
gpio7   →  Extruder step
gpio6   →  Extruder dir
gpio14  →  Extruder enable
gpio15  →  Extruder TMC2209 UART
gpio23  →  Hotend heater
gpio27  →  Hotend thermistor
gpio21  →  Part cooling fan
gpio13  →  Hotend fan (5V jumper)
gpio0   →  CAN TX
gpio1   →  CAN RX
```

## Host Setup

Currently running on a **Pop!_OS Linux PC** (temporary, will move to Raspberry Pi 5).

- **Klipper service:** `klipper-2` (installed via KIAUH)
- **Live config:** `~/printer_2_data/config/printer.cfg`
- **Logs:** `~/printer_2_data/logs/klippy.log`
- **Restart:** `sudo systemctl restart klipper-2`
- **CAN config:** `/etc/network/interfaces.d/can0`
- **Kalico source:** `~/klipper` (symlink to `~/source/kalico`)
- **Klippy venv:** `~/klippy-env`

## Repo Structure

```
config/printer.cfg          — Working printer config (copy to ~/printer_2_data/config/)
firmware/
  octopus-v1.1-can-bridge.bin  — Octopus USB-to-CAN bridge firmware
  sht36-v3-max-can.bin         — SHT36 Klipper CAN firmware
  sht36-v3-max-katapult.uf2    — SHT36 Katapult bootloader
  build-firmware.sh            — Docker-based firmware builder
  README.md                    — Flashing instructions
CLAUDE.md                   — Setup checklist and next steps
FLASH-PLAN.md               — Firmware menuconfig settings
SHT36-V3-MAX-PINOUT.md      — SHT36 pin reference
VORON-TRIDENT-BUILD-GUIDE.md — Full build documentation
```

## Next Steps

1. Flash Katapult bootloader to SHT36 via USB boot mode
2. Wire CAN bus (CANH, CANL, GND) between Octopus and SHT36
3. Flash Klipper firmware to SHT36 via CAN
4. Uncomment SHT36 sections in printer.cfg (extruder, fans, probe)
5. Test bed heater
6. Test endstop homing (X, Y, Z)
7. Z tilt calibration
8. PID tune hotend and bed
9. Migrate to Raspberry Pi 5
