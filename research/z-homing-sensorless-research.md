# Z-Axis Homing Research: Sensorless vs Sexbolt vs Fly Eddy

> Research compiled 2026-01-29. Sources: Gemini, Qwen, DeepSeek consensus analysis.

---

## Question

Can we do sensorless homing on Z with our Voron Trident hardware?

**Hardware context:**
- BTT Octopus V1.0 with TMC2209 drivers
- 3 independent Z motors on leadscrews (TR8x8 or TR8x2)
- Mellow Fly SHT36 V3 Max with onboard Fly Eddy (LDC1612, I2C)
- Bambu Lab H2S/H2D hotend (expensive nozzle)
- Sexbolt Z endstop already installed
- Running Kalico (Klipper fork)

---

## 1. TMC2209 StallGuard on Z Leadscrews

**Verdict: DANGEROUS. Do not attempt.**

- Leadscrews provide massive mechanical advantage (high torque). By the time StallGuard triggers a motor stall, the nozzle has already been driven into the bed with destructive force.
- With 3 independent Z motors, a stall on one motor while the others keep pushing will **twist the gantry frame**, potentially permanently damaging linear rails or leadscrews.
- StallGuard requires a minimum motor speed to generate reliable back-EMF for stall detection. Z leadscrews move too slowly during homing, making detection inconsistent.
- Leadscrew lubrication variance further degrades StallGuard reliability.

**Source:** [Klipper Discourse - Sensorless Z Safety](https://klipper.discourse.group/t/using-tmc2209-sensorless-homing-as-a-safety-fallback-for-z/10498)

---

## 2. Fly Eddy (LDC1612) as Virtual Z Endstop

**Verdict: Technically possible, but thermally risky.**

### How it works
- Klipper/Kalico supports `endstop_pin: probe:z_virtual_endstop` with `[probe_eddy_current]`
- The eddy current sensor detects the conductive steel bed surface **without nozzle contact**
- The nozzle never touches anything during homing

### Configuration
```ini
[stepper_z]
endstop_pin: probe:z_virtual_endstop
homing_retract_dist: 0

[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
i2c_address: 43
i2c_mcu: SHT36
i2c_bus: i2c1e
z_offset: 0.8
```

### Critical risk: Thermal drift
- LDC1612 readings drift significantly with temperature changes in the sensor coil and surrounding metal
- If you home Z cold then heat up the bed/chamber, the z_offset shifts
- If the drift goes in the wrong direction, the nozzle crashes into the bed
- **Beacon and Cartographer** have mature onboard thermal compensation; the Fly Eddy lacks this and relies on software calibration which must be done carefully
- Mellow's own docs warn about repeated z_offset adjustments and thermal sensitivity

### Calibration commands (if attempting eddy homing)
```
LDC_CALIBRATE_DRIVE_CURRENT CHIP=fly_eddy_probe
PROBE_EDDY_CURRENT_CALIBRATE CHIP=fly_eddy_probe
```

**Sources:**
- [Mellow Eddy Troubleshooting](https://mellow.klipper.cn/en/docs/DebugDoc/faq/eddy/)
- [Klipper Eddy Probe Docs](https://www.klipper3d.org/Eddy_Probe.html)

---

## 3. Sexbolt Endstop

**Verdict: SAFEST option for Z homing with Bambu H2S.**

### Why
- Physically measures the **actual nozzle tip position** relative to the bed
- Immune to thermal drift — the measurement is purely mechanical
- If you swap nozzles or the hotend expands with heat, the zero point remains accurate
- Sub-0.01mm repeatability with a well-designed sexbolt

### Trade-offs
- Slower than eddy probe homing (must move to probe location, trigger, retract)
- Minimal mechanical wear on nozzle tip over time (negligible)

---

## 4. Nozzle Protection (Bambu H2S)

The Bambu H2S nozzle is expensive and not easily replaced. Contact prevention is critical.

- **Sexbolt** measures nozzle tip directly — safest
- **Eddy probe** measures bed surface distance from the sensor, NOT from the nozzle — thermal expansion of the hotend is invisible to the sensor
- Always use `[safe_z_home]` to force the toolhead to a known position before Z homing

---

## 5. Kalico vs Upstream Klipper

- Both support `[probe_eddy_current]` and `endstop_pin: probe:z_virtual_endstop`
- No fundamental difference for Z homing specifically
- Kalico may offer earlier access to rapid scanning modes for bed mesh
- Both face the same thermal drift challenge with eddy current sensors

---

## 6. Recommendation: Hybrid Approach

| Task | Method | Why |
|---|---|---|
| Z homing | **Sexbolt** | Safe, thermal-stable, measures nozzle tip directly |
| Bed mesh | **Fly Eddy (LDC1612)** | Fast scanning, high-res meshes |
| Z-tilt adjust | **Fly Eddy (LDC1612)** | Speed advantage for 3-point leveling |
| X/Y homing | **Sensorless (TMC2209)** | Reliable on belt-driven axes |

### Config strategy
```ini
# Z homing via sexbolt
[stepper_z]
endstop_pin: PG10  # sexbolt pin (verify for your wiring)
position_endstop: 0.5  # calibrate

# Fly Eddy for mesh and tilt ONLY
[probe_eddy_current fly_eddy_probe]
sensor_type: ldc1612
i2c_address: 43
i2c_mcu: SHT36
i2c_bus: i2c1e
z_offset: 0.8
x_offset: 0
y_offset: 0

[z_tilt]
# Uses fly_eddy_probe for fast 3-point leveling

[bed_mesh]
# Uses fly_eddy_probe for rapid scanning
```

### If you later want to experiment with Eddy homing
1. Thermal soak: Heat bed and hotend to printing temps, wait for stabilization before homing
2. Drift calibration: Measure z_offset at multiple temperatures, build a lookup table
3. Redundant check: After eddy homing, verify with sexbolt and abort if discrepancy exceeds threshold

**Conservative recommendation: Keep the sexbolt for Z homing.**
