/*
 * UVC LED Bench Test — Arduino Nano version
 * LED: NewEnergy LST1-01G08-UV01-01 (280nm, ~2.28W)
 * Driver: Mean Well LDD-350LW (350mA CC, PWM dim via DIM pin)
 * Sensor: SparkFun AS7331
 *
 * Wiring:
 *   D9   -> LDD-350LW WHITE wire (DIM)  -- LOW=off, HIGH=on, PWM=dim
 *   A4   -> AS7331 SDA
 *   A5   -> AS7331 SCL
 *   3V3  -> AS7331 VCC  (DO NOT USE 5V !!!)
 *   GND  -> AS7331 GND  + LDD-350LW black wire (-Vin), all common
 *
 *   LDD-350LW outputs:
 *     YELLOW -> LED+ (anode)
 *     BLUE   -> LED- (cathode)
 *
 * Serial commands (115200 baud):
 *   d   — dark calibration (LED forced off, 20 readings)
 *   r   — full-power LED test (5 sec, 30 readings)
 *   p   — PWM ramp 10/25/50/75/100%
 *   m   — single measurement at current setting
 *   c   — continuous live readout
 *   1-9 — set LED at 10..90% for 10 sec
 *   0   — set LED at 100% for 10 sec
 *   s   — STOP (force LED off + exit continuous)
 *   q   — query state
 *   ?   — help
 */

#include <Wire.h>
#include "SparkFun_AS7331.h"

// -------------------- CONFIG --------------------
#define LED_PIN          9        // D9 — Nano PWM-capable pin
#define MAX_ON_TIME_MS   30000UL  // hard auto-shutoff after 30 sec
#define WARMUP_MS        2000     // LED stabilization time
#define N_READINGS       30

#define AS7331_GAIN      GAIN_2
#define AS7331_TIME      TIME_64MS
#define AS7331_CCLK      CCLK_1_024_MHZ
#define UVC_FSR          5312.0
#define SATURATION_LIMIT 0.95

// -------------------- GLOBALS --------------------
SfeAS7331ArdI2C uvSensor;

float darkUVA = 0, darkUVB = 0, darkUVC = 0;
bool  darkCalibrated = false;

unsigned long ledOnSinceMs = 0;
bool ledIsOn = false;
uint8_t currentDuty = 0;
bool continuousMode = false;

// -------------------- LED CONTROL --------------------

void ledOff() {
  analogWrite(LED_PIN, 0);
  digitalWrite(LED_PIN, LOW);
  ledIsOn = false;
  currentDuty = 0;
  ledOnSinceMs = 0;
}

void ledSetDuty(uint8_t duty) {
  analogWrite(LED_PIN, duty);
  currentDuty = duty;
  if (duty == 0) {
    ledIsOn = false;
    ledOnSinceMs = 0;
  } else {
    if (!ledIsOn) ledOnSinceMs = millis();
    ledIsOn = true;
  }
}

void ledSetPercent(uint8_t pct) {
  if (pct > 100) pct = 100;
  uint8_t duty = (uint16_t)pct * 255 / 100;
  ledSetDuty(duty);
}

void enforceMaxOnTime() {
  if (ledIsOn && (millis() - ledOnSinceMs > MAX_ON_TIME_MS)) {
    ledOff();
    Serial.println(F("[SAFETY] LED auto-shutoff: max ON time reached"));
  }
}

// -------------------- SENSOR --------------------

bool readOne(float &uva, float &uvb, float &uvc) {
  if (uvSensor.setStartState(true) != 0) return false;
  delay(uvSensor.getConversionTimeMillis() + 2);
  if (uvSensor.readAllUV() != 0) return false;
  uva = uvSensor.getUVA();
  uvb = uvSensor.getUVB();
  uvc = uvSensor.getUVC();
  return true;
}

bool readAveraged(int n, float &uva, float &uvb, float &uvc, float &uvc_std) {
  double sa = 0, sb = 0, sc = 0, sc2 = 0;
  int got = 0;
  for (int i = 0; i < n; i++) {
    float a, b, c;
    if (!readOne(a, b, c)) continue;
    sa += a; sb += b; sc += c; sc2 += (double)c * c;
    got++;
    delay(20);
  }
  if (got == 0) return false;
  uva = sa / got;
  uvb = sb / got;
  uvc = sc / got;
  double meanc = sc / got;
  double var = (sc2 / got) - (meanc * meanc);
  uvc_std = (var > 0) ? sqrt(var) : 0;
  return true;
}

bool checkSaturation(float uvc, const char *label) {
  if (uvc > UVC_FSR * SATURATION_LIMIT) {
    Serial.print(F("[WARN] "));
    Serial.print(label);
    Serial.print(F(" UVC="));
    Serial.print(uvc);
    Serial.println(F(" near FSR — INCREASE DISTANCE OR REDUCE PWM"));
    return true;
  }
  return false;
}

// -------------------- ROUTINES --------------------

void doDarkCalibration() {
  Serial.println(F("\n=== DARK CALIBRATION ==="));
  Serial.println(F("Forcing LED OFF, waiting 3 sec..."));
  ledOff();
  delay(3000);
  Serial.println(F("Reading 20 dark samples..."));
  float a, b, c, std;
  if (!readAveraged(20, a, b, c, std)) {
    Serial.println(F("[ERR] Sensor read failed"));
    return;
  }
  darkUVA = a; darkUVB = b; darkUVC = c;
  darkCalibrated = true;
  Serial.print(F("Dark offsets: UVA="));
  Serial.print(darkUVA, 4); Serial.print(F("  UVB="));
  Serial.print(darkUVB, 4); Serial.print(F("  UVC="));
  Serial.println(darkUVC, 4);
  Serial.println(F("=== Done ===\n"));
}

void runSinglePowerTest() {
  if (!darkCalibrated) {
    Serial.println(F("[ERR] Run 'd' first."));
    return;
  }
  Serial.println(F("\n=== FULL-POWER TEST ==="));
  Serial.println(F("[1/4] LED OFF baseline..."));
  ledOff();
  delay(500);
  float a0, b0, c0, std0;
  readAveraged(10, a0, b0, c0, std0);
  Serial.print(F("  Baseline: UVA="));
  Serial.print(a0 - darkUVA, 3); Serial.print(F("  UVB="));
  Serial.print(b0 - darkUVB, 3); Serial.print(F("  UVC="));
  Serial.println(c0 - darkUVC, 3);

  Serial.print(F("[2/4] LED ON 100%, warming up "));
  Serial.print(WARMUP_MS / 1000); Serial.println(F("s..."));
  ledSetPercent(100);
  delay(WARMUP_MS);

  Serial.println(F("[3/4] Reading 30 samples..."));
  float a, b, c, std;
  if (!readAveraged(N_READINGS, a, b, c, std)) {
    Serial.println(F("[ERR] read failed"));
    ledOff();
    return;
  }

  Serial.println(F("[4/4] LED OFF"));
  ledOff();

  float uva = a - darkUVA;
  float uvb = b - darkUVB;
  float uvc = c - darkUVC;

  Serial.println(F("\n--- RESULT ---"));
  Serial.print(F("UVA: ")); Serial.print(uva, 2); Serial.println(F(" uW/cm2"));
  Serial.print(F("UVB: ")); Serial.print(uvb, 2); Serial.println(F(" uW/cm2"));
  Serial.print(F("UVC: ")); Serial.print(uvc, 2);
  Serial.print(F(" uW/cm2  (+/-")); Serial.print(std, 2);
  Serial.println(F(" 1sigma)"));
  if (uvc > 0) {
    Serial.print(F("Noise: "));
    Serial.print(100.0 * std / uvc, 2);
    Serial.println(F("%"));
  }
  checkSaturation(c, "Test");
  Serial.println(F("=============\n"));
}

void runPwmRamp() {
  if (!darkCalibrated) {
    Serial.println(F("[ERR] Run 'd' first."));
    return;
  }
  const uint8_t steps[] = {10, 25, 50, 75, 100};
  Serial.println(F("\n=== PWM RAMP ==="));
  Serial.println(F("Duty%  UVA      UVB      UVC      sigma_UVC"));
  for (uint8_t i = 0; i < 5; i++) {
    ledSetPercent(steps[i]);
    delay(WARMUP_MS);
    float a, b, c, std;
    if (!readAveraged(15, a, b, c, std)) continue;
    Serial.print(F(" "));
    Serial.print(steps[i]);
    Serial.print(F("\t"));
    Serial.print(a - darkUVA, 2);
    Serial.print(F("\t"));
    Serial.print(b - darkUVB, 2);
    Serial.print(F("\t"));
    Serial.print(c - darkUVC, 2);
    Serial.print(F("\t"));
    Serial.println(std, 2);
    if (checkSaturation(c, "Ramp step")) {
      Serial.println(F("Saturation — aborting."));
      break;
    }
  }
  ledOff();
  Serial.println(F("=== Ramp done ===\n"));
}

void runMeasurement() {
  if (!darkCalibrated) {
    Serial.println(F("[WARN] No dark calibration — uncorrected."));
  }
  Serial.println(F("\n=== MEASUREMENT ==="));
  Serial.print(F("Current LED duty: "));
  Serial.print((uint16_t)currentDuty * 100 / 255);
  Serial.println(F("%"));
  Serial.println(F("Reading 30 samples..."));
  float a, b, c, std;
  if (!readAveraged(30, a, b, c, std)) {
    Serial.println(F("[ERR] read failed"));
    return;
  }
  float uva = a - darkUVA;
  float uvb = b - darkUVB;
  float uvc = c - darkUVC;
  Serial.print(F("UVA: ")); Serial.println(uva, 2);
  Serial.print(F("UVB: ")); Serial.println(uvb, 2);
  Serial.print(F("UVC: ")); Serial.print(uvc, 2);
  Serial.print(F(" (+/-")); Serial.print(std, 2);
  Serial.println(F(")"));
  Serial.println(F("===\n"));
}

void holdAtPercent(uint8_t pct) {
  if (!darkCalibrated) {
    Serial.println(F("[ERR] Run 'd' first."));
    return;
  }
  Serial.print(F("\nLED at "));
  Serial.print(pct);
  Serial.println(F("% for 10s..."));
  ledSetPercent(pct);
  delay(WARMUP_MS);
  unsigned long endMs = millis() + 10000;
  while ((long)(millis() - endMs) < 0) {
    enforceMaxOnTime();
    if (!ledIsOn) break;
    float a, b, c;
    if (readOne(a, b, c)) {
      Serial.print(F("UVA="));
      Serial.print(a - darkUVA, 2);
      Serial.print(F("  UVB="));
      Serial.print(b - darkUVB, 2);
      Serial.print(F("  UVC="));
      Serial.println(c - darkUVC, 2);
      checkSaturation(c, "Hold");
    }
    if (Serial.available()) {
      char ch = Serial.read();
      if (ch == 's' || ch == 'S') {
        Serial.println(F("[STOP] User abort"));
        break;
      }
    }
    delay(500);
  }
  ledOff();
  Serial.println(F("Hold ended.\n"));
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  d  — dark calibration"));
  Serial.println(F("  r  — full-power test"));
  Serial.println(F("  p  — PWM ramp 10/25/50/75/100%"));
  Serial.println(F("  m  — measurement at current setting"));
  Serial.println(F("  c  — continuous readout"));
  Serial.println(F("  1-9 — hold at 10..90%"));
  Serial.println(F("  0  — hold at 100%"));
  Serial.println(F("  s  — STOP"));
  Serial.println(F("  q  — query state"));
  Serial.println(F("  ?  — help\n"));
}

void printState() {
  Serial.print(F("LED: "));
  if (ledIsOn) {
    Serial.print(F("ON  duty="));
    Serial.print((uint16_t)currentDuty * 100 / 255);
    Serial.print(F("%  on for "));
    Serial.print((millis() - ledOnSinceMs) / 1000);
    Serial.println(F("s"));
  } else {
    Serial.println(F("OFF"));
  }
  Serial.print(F("Dark cal: "));
  Serial.println(darkCalibrated ? "yes" : "no");
}

// -------------------- SETUP --------------------

void setup() {
  // Force LED off immediately
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n=== UVC LED Test — Arduino Nano ==="));
  Serial.println(F("LED: LST1-01G08-UV01-01 (280nm)"));
  Serial.println(F("Driver: LDD-350LW @ 350mA"));
  Serial.println(F("Sensor: AS7331"));
  Serial.println(F("[SAFETY] Box closed, glasses on, never look at LED."));

  Wire.begin();
  Wire.setClock(100000);

  if (uvSensor.begin() == false) {
    Serial.println(F("[ERR] AS7331 not found. Check wiring (SDA=A4, SCL=A5, VCC=3V3)."));
    while (1) delay(1000);
  }
  Serial.println(F("AS7331 found."));

  uvSensor.setGain(AS7331_GAIN);
  uvSensor.setConversionTime(AS7331_TIME);
  uvSensor.setCClk(AS7331_CCLK);

  if (uvSensor.prepareMeasurement(MEAS_MODE_CMD) == false) {
    Serial.println(F("[ERR] prepareMeasurement failed"));
    while (1) delay(1000);
  }

  Serial.print(F("Preset: GAIN_2 / 64ms — UVC FSR = "));
  Serial.print(UVC_FSR);
  Serial.println(F(" uW/cm2"));

  printHelp();
}

// -------------------- LOOP --------------------

void loop() {
  enforceMaxOnTime();

  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'd': case 'D': doDarkCalibration(); break;
      case 'r': case 'R': runSinglePowerTest(); break;
      case 'p': case 'P': runPwmRamp(); break;
      case 'm': case 'M': runMeasurement(); break;
      case 'c': case 'C':
        Serial.println(F("\n=== CONTINUOUS (s to stop) ==="));
        Serial.println(F("UVA      UVB      UVC"));
        continuousMode = true;
        break;
      case 's': case 'S':
        ledOff();
        if (continuousMode) {
          continuousMode = false;
          Serial.println(F("--- continuous stopped ---"));
        } else {
          Serial.println(F("[STOP] LED forced off."));
        }
        break;
      case 'q': case 'Q': printState(); break;
      case '?':           printHelp(); break;
      case '0':           holdAtPercent(100); break;
      case '1':           holdAtPercent(10);  break;
      case '2':           holdAtPercent(20);  break;
      case '3':           holdAtPercent(30);  break;
      case '4':           holdAtPercent(40);  break;
      case '5':           holdAtPercent(50);  break;
      case '6':           holdAtPercent(60);  break;
      case '7':           holdAtPercent(70);  break;
      case '8':           holdAtPercent(80);  break;
      case '9':           holdAtPercent(90);  break;
      case '\n': case '\r': case ' ': break;
      default:
        Serial.print(F("Unknown: '"));
        Serial.print(ch);
        Serial.println(F("'  press '?' for help"));
        break;
    }
  }

  if (continuousMode) {
    float a, b, c;
    if (readOne(a, b, c)) {
      Serial.print(a - darkUVA, 2);
      Serial.print(F("\t"));
      Serial.print(b - darkUVB, 2);
      Serial.print(F("\t"));
      Serial.println(c - darkUVC, 2);
    }
    delay(200);
  }
  delay(20);
}