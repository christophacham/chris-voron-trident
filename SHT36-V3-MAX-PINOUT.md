# Mellow Fly SHT36 V3 Max - Pinout & Wiring

## Board Overview

- **MCU:** Raspberry Pi RP2040 (Dual-core ARM Cortex-M0+ @ 133MHz)
- **Input Voltage:** 12-24V DC
- **Connector:** XT30 (2+2), 15A continuous / 30A peak
- **Driver:** TMC2209 onboard (up to 1.6A)
- **Accelerometer:** LIS2DW (onboard)
- **Probe:** LDC1612 (Fly Eddy, onboard)
- **PT100/PT1000:** MAX31865 chip (onboard)

---

## Pin Assignments (Klipper GPIO)

### Extruder Stepper
| Function | GPIO |
|----------|------|
| Step | gpio7 |
| Direction | gpio6 |
| Enable | gpio14 |
| TMC2209 UART | gpio15 |

### Heater & Thermistors
| Function | GPIO | Notes |
|----------|------|-------|
| Heater | gpio23 | 5A @ 24V max |
| Thermistor (TH0) | gpio27 | 100K thermistor or PT1000 |
| PT100 CS (MAX31865) | gpio17 | For PT100/PT1000 via SPI |

### Fans
| Function | GPIO | Notes |
|----------|------|-------|
| Fan 0 (hotend) | gpio13 | Voltage selectable via jumper |
| Fan 1 (part cooling) | gpio21 | Voltage selectable via jumper |

**Fan voltage jumper options:** 5V, 12V, or VIN (24V)

### Probe & Endstops
| Function | GPIO | Notes |
|----------|------|-------|
| Probe input | gpio22 | Standard probe signal |
| Servo/BLTouch | gpio24 | PWM output |
| Endstop 0 | gpio20 | Active low |
| Endstop 1 | gpio16 | Filament sensor / X endstop |

### Accelerometer (LIS2DW) - SPI
| Function | GPIO |
|----------|------|
| CS | gpio12 |
| SCLK | gpio2 |
| MOSI | gpio3 |
| MISO | gpio4 |

### RGB LEDs
| Function | GPIO |
|----------|------|
| Neopixel data | gpio26 |

### Fly Eddy Probe (LDC1612) - I2C
| Function | Notes |
|----------|-------|
| I2C Address | 0x2B (onboard) |
| SDA/SCL | Directly wired internally |

---

## Jumper Settings

### CAN Termination (120Ω)
- **Single toolboard:** Install CAN jumper
- **Multiple toolboards:** Only on last board in chain

### PT1000 Jumper
- **PT1000 sensor:** Install jumper
- **Standard thermistor:** Remove jumper

### IO1 Diode Bypass
- **Simple switch:** Install jumper
- **Electronic probe (inductive/optical):** Remove jumper

### Fan Voltage
Set jumper for each fan:
- 5V
- 12V
- VIN (24V)

---

## Connector Layout (Physical)

```
    [USB-C]              [CAN Terminal]
       |                      |
  +---------TOP OF BOARD----------+
  |                               |
  |  [Fan0]  [Fan1]   [RGB]       |
  |                               |
  |  [Heater]  [TH0]  [PT100]     |
  |                               |
  |  [Probe]  [Endstop]           |
  |                               |
  |     [TMC2209 Driver]          |
  |                               |
  |  [Motor]                      |
  |                               |
  +---------BOTTOM-----------------+
       |
   [XT30 Power+CAN]
```

*(For exact positions, see official diagram links below)*

---

## Klipper Config Snippet

```ini
[mcu sht36]
canbus_uuid: YOUR_UUID_HERE  # Get via: ~/klippy-env/bin/python ~/klipper/scripts/canbus_query.py can0

[extruder]
step_pin: sht36:gpio7
dir_pin: sht36:gpio6
enable_pin: !sht36:gpio14
heater_pin: sht36:gpio23
sensor_pin: sht36:gpio27

[tmc2209 extruder]
uart_pin: sht36:gpio15

[fan]  # Part cooling
pin: sht36:gpio21

[heater_fan hotend_fan]
pin: sht36:gpio13

[probe]
pin: sht36:gpio22

[adxl345]  # Actually LIS2DW but compatible
cs_pin: sht36:gpio12
spi_bus: spi0a

[neopixel toolhead]
pin: sht36:gpio26
```

---

## Current Limits

| Output | 24V Max | 12V Max |
|--------|---------|---------|
| Heater | 5A | 2.5A |
| Each Fan | 1A | 0.5A |

---

## Documentation Links

- [Official FLY Docs](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/)
- [Wiring Diagrams](https://mellow.klipper.cn/en/docs/ProductDoc/ToolBoard/fly-sht36/sht36_v3/wiring/) *(right-click → save image for full resolution)*
- [Pin Names Reference](https://teamgloomy.github.io/fly_sht36_max_v3_pins.html)
- [General Info](https://teamgloomy.github.io/fly_sht36_max_v3_general.html)
- [Schematic & CAD Files](https://mellow-3d.github.io/fly-sht36_files.html)
