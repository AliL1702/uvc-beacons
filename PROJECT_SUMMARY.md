# UVC 275nm Optical Wireless Communication Project

Short-range, line-of-sight **Optical Wireless Communication (OWC)** link using a 275–280nm UVC LED as the transmitter and a SparkFun AS7331 UV spectral sensor as the receiver. UVC light is strongly absorbed by the atmosphere and blocked by standard window glass, making this channel highly localized, secure, and low-interference.

## Important
There is a document in this repo that explains all changes needed and parts purchased for this project.

## Hardware

| Component | Details |
|-----------|---------|
| UV LED | NewEnergy LST1-01G08-UV01-01 (280nm, ~2.28W) |
| LED driver | Mean Well LDD-350LW (350mA CC, PWM dim via DIM pin) |
| UV sensor | SparkFun AS7331 (UVA/UVB/UVC spectral sensor, I2C) |
| TX board | Arduino Nano |
| RX board | XIAO ESP32-C3 |

### Wiring — Transmitter (Arduino Nano)
- D9 -> LDD-350LW WHITE wire (DIM)
- GND -> LDD black wire + 12V supply negative (common ground)
- 10k pulldown from D9 to GND (safety)

### Wiring — Receiver (XIAO ESP32-C3)
- GPIO6 -> AS7331 SDA
- GPIO7 -> AS7331 SCL
- 3V3 -> AS7331 VCC
- GND -> AS7331 GND

### LDD-350LW Wires
- RED -> +12V supply
- BLACK -> Ground (common)
- YELLOW -> LED+ (anode)
- BLUE -> LED- (cathode)
- WHITE -> DIM control (D9 on Nano, or 5V for always-on)

## Sketches

### 1. `TX_arduino/` — Data Link Transmitter (Arduino Nano)
Sends text messages over the UVC LED using **Manchester-encoded OOK** (on-off keying). The Nano drives the LDD-350LW DIM pin on D9 to modulate the LED. Configurable bit rate (1–200 bps, default 20). Supports single message transmission, rate sweep tests (5/10/20/30/50 bps), and periodic ping mode. Frame format: 2-byte preamble (0xAA), start delimiter (0x7E), length, payload, XOR checksum.

- **Board:** Arduino Nano
- **Key pin:** D9 -> LDD DIM (white wire)

### 2. `RX_XIAO/` — Data Link Receiver (XIAO ESP32-C3)
Receives and decodes the Manchester-encoded UVC optical signal using the AS7331 sensor. Samples UVC irradiance at ~94 Hz (GAIN_64, TIME_8MS), thresholds it to binary, and decodes Manchester transitions with bit-level start-byte (0x7E) hunting — no byte-alignment needed. Validates frames with XOR checksum and reports decoded text, payload bit rate, and timing. Also has a live UV monitor mode and stats.

- **Board:** XIAO ESP32-C3
- **I2C pins:** SDA=GPIO6, SCL=GPIO7

### 3. `275nm_led_test/` — LED Bench Test (XIAO ESP32-C3)
Stand-alone bench test for measuring the UVC LED output. DIM pin tied to 5V (always-on when 12V is applied). Supports dark calibration (LED off), single averaged measurement (30 samples with std dev), and continuous live readout. Reports UVA/UVB/UVC in uW/cm2 with saturation warnings.

- **Board:** XIAO ESP32-C3
- **Sensor settings:** GAIN_2, TIME_64MS

### 4. `uv_sensor_led_arduinon_nano/` — LED Bench Test (Arduino Nano)
Full-featured bench test on the Nano. Adds PWM control of the LED via D9 (ramp test at 10/25/50/75/100%, hold at arbitrary duty cycle, auto-shutoff after 30s safety timer). Dark calibration, averaged measurements, continuous readout, saturation checking.

- **Board:** Arduino Nano
- **Key pin:** D9 -> LDD DIM (PWM)

### 5. `uv_sensor_test.ino/` — Sensor Calibration Tool (XIAO ESP32-C3)
Advanced calibration sketch. Runs dark calibration across 5 gain/time presets, plus a sun calibration routine that measures UVC contamination (cross-talk from UVA/UVB channels). Computes an alpha coefficient so that real UVC = measured UVC - alpha*(UVA+UVB). Intended for characterizing sensor accuracy before using it in the data link.

- **Board:** XIAO ESP32-C3
- **Presets:** 5 gain/time combos from LED-close to max-sensitivity

## File Map

```
uvc-275nm-project/
  TX_arduino/TX_arduino.ino                          # TX: data link transmitter
  RX_XIAO/RX_XIAO.ino                               # RX: data link receiver
  275nm_led_test/275nm_led_test.ino                   # bench test (XIAO)
  uv_sensor_led_arduinon_nano/uv_sensor_led_arduinon_nano.ino  # bench test (Nano)
  uv_sensor_test.ino/uv_sensor_test.ino.ino          # sensor calibration tool
  PROJECT_SUMMARY.md                                  # this file
```

## Sensor Characterization & Critical Findings

### Saturation Ceiling
High-gain presets (3 and 4) saturate in bright environments. The library returns flatlined values (e.g. exactly 21.2449 uW/cm2 on Preset 3, 10.6214 uW/cm2 on Preset 4). Never use Presets 2–4 in direct sunlight — drop to Preset 0 or 1.

### Out-of-Band Solar Leakage
In direct sunlight, the UVC channel reports ~50 uW/cm2 even though Earth's ozone blocks 100% of solar UVC. This is spectral leakage: the massive outdoor UVA signal (~7,500 uW/cm2) bleeds through the on-chip UVC filter at roughly 0.7%.

### Indoor Low-Signal Noise
Indoor UVC/UVB readings near a closed window can appear higher than outdoor shade readings. Window glass blocks true UV, so at max gain the sensor amplifies its own electronic noise floor or picks up spectral emissions from artificial lights.

## Environmental Baselines

| Environment | Use Presets | Expected UVA (uW/cm2) | UVC Channel Behavior |
|---|---|---|---|
| Enclosed/Dark | 3 or 4 | 0.0000 | Clean zero after calibration |
| Indoors (Window) | 2 or 3 | ~15.0 | True UV blocked by glass; ignore UVC artifacts |
| Outdoors (Shade) | 1 or 2 | ~240.0 | High scattered UVA; low UVB/UVC |
| Direct Sunlight | 0 or 1 | ~7,500+ | High UVA; false ~50 uW/cm2 UVC leakage |

## Engineering Task Status

| Task | Status | Implementation |
|---|---|---|
| Signal modulation (Manchester OOK) | **Done** | `TX_arduino/` encodes, `RX_XIAO/` decodes. Configurable 1–200 bps. |
| Differential filtering (solar leakage subtraction) | **Done** | `uv_sensor_test.ino/` computes alpha via sun calibration: True UVC = Raw UVC - alpha * (UVA + UVB) |
| Auto-Gain Control (AGC) | **Partial** | `uv_sensor_test.ino/` detects saturation and suggests a preset, but does not auto-switch yet. |
| Physical UVC pass filter | **Not started** | Hardware task — source a solar-blind bandpass filter to place over the AS7331 to reject UVA mechanically. |

## Dependencies

- [SparkFun AS7331 Arduino Library](https://github.com/sparkfun/SparkFun_AS7331_Arduino_Library) (installed at `~/Arduino/libraries/`)


