/*
 * UVC LED Bench Test — DIM tied to XIAO 5V (always on with 12V applied)
 *
 * Wiring:
 *   GPIO 6  -> AS7331 SDA
 *   GPIO 7  -> AS7331 SCL
 *   3V3     -> AS7331 VCC
 *   GND     -> AS7331 GND  AND  LDD -Vin (black)  AND  12V supply -
 *
 *   LDD-350LW wires:
 *     RED    -> +12V supply
 *     BLACK  -> Ground (common with XIAO GND)
 *     YELLOW -> LED + (anode on starboard)
 *     BLUE   -> LED - (cathode on starboard)
 *     WHITE  -> XIAO 5V pin (DIM held high = LED enabled)
 *
 * To turn LED OFF:  unplug 12V supply
 * To turn LED ON:   plug in 12V supply
 *
 * Serial commands:
 *   d  — dark calibration (you must unplug 12V first!)
 *   m  — single measurement (30 readings, average)
 *   c  — continuous live readings
 *   s  — stop continuous mode
 *   ?  — help
 */

#include <Wire.h>
#include "SparkFun_AS7331.h"

#define I2C_SDA          6
#define I2C_SCL          7

#define AS7331_GAIN      GAIN_2
#define AS7331_TIME      TIME_64MS
#define AS7331_CCLK      CCLK_1_024_MHZ
#define UVC_FSR          5312.0
#define SATURATION_LIMIT 0.95

SfeAS7331ArdI2C uvSensor;
float darkUVA = 0, darkUVB = 0, darkUVC = 0;
bool  darkCalibrated = false;
bool  continuousMode = false;

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

void doDarkCalibration() {
  Serial.println(F("\n=== DARK CALIBRATION ==="));
  Serial.println(F("!!! UNPLUG 12V SUPPLY NOW (LED must be OFF) !!!"));
  Serial.println(F("Waiting 5 seconds for you to unplug..."));
  delay(5000);
  Serial.println(F("Reading 20 dark samples..."));
  float a, b, c, std;
  if (!readAveraged(20, a, b, c, std)) {
    Serial.println(F("[ERR] Sensor read failed"));
    return;
  }
  darkUVA = a; darkUVB = b; darkUVC = c;
  darkCalibrated = true;
  Serial.print(F("Dark offsets [µW/cm²]: UVA="));
  Serial.print(darkUVA, 4); Serial.print(F("  UVB="));
  Serial.print(darkUVB, 4); Serial.print(F("  UVC="));
  Serial.println(darkUVC, 4);
  Serial.println(F("Now plug 12V back in to turn LED on."));
  Serial.println(F("=== Done ===\n"));
}

void runMeasurement() {
  if (!darkCalibrated) {
    Serial.println(F("[WARN] No dark calibration yet — readings are uncorrected."));
  }
  Serial.println(F("\n=== MEASUREMENT ==="));
  Serial.println(F("Make sure 12V is APPLIED → LED should be ON."));
  Serial.println(F("Reading 30 samples..."));
  float a, b, c, std;
  if (!readAveraged(30, a, b, c, std)) {
    Serial.println(F("[ERR] Sensor read failed"));
    return;
  }
  float uva = a - darkUVA;
  float uvb = b - darkUVB;
  float uvc = c - darkUVC;
  Serial.println(F("\n--- RESULT ---"));
  Serial.print(F("UVA: ")); Serial.print(uva, 2); Serial.println(F(" µW/cm²"));
  Serial.print(F("UVB: ")); Serial.print(uvb, 2); Serial.println(F(" µW/cm²"));
  Serial.print(F("UVC: ")); Serial.print(uvc, 2);
  Serial.print(F(" µW/cm² (±")); Serial.print(std, 2);
  Serial.println(F(" µW/cm² 1σ)"));
  if (uvc > 0) {
    Serial.print(F("UVC noise: "));
    Serial.print(100.0 * std / uvc, 2); Serial.println(F("%"));
  }
  if (c > UVC_FSR * SATURATION_LIMIT) {
    Serial.println(F("[WARN] UVC near saturation — increase distance!"));
  }
  Serial.println(F("===============\n"));
}

void printHelp() {
  Serial.println(F("\nCommands:"));
  Serial.println(F("  d  — dark calibration (unplug 12V first!)"));
  Serial.println(F("  m  — single measurement (30-sample average)"));
  Serial.println(F("  c  — continuous live readings"));
  Serial.println(F("  s  — stop continuous mode"));
  Serial.println(F("  ?  — this help\n"));
  Serial.println(F("LED on/off: plug or unplug 12V supply"));
  Serial.println(F("(white DIM wire is tied to 5V = LED enabled)\n"));
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("\n\n=== UVC LED Test (DIM tied to 5V) ==="));
  Serial.println(F("LED: NewEnergy LST1-01G08-UV01-01 (280nm)"));
  Serial.println(F("Driver: Mean Well LDD-350LW @ 350mA"));
  Serial.println(F("Sensor: SparkFun AS7331"));
  Serial.println(F(""));
  Serial.println(F("[SAFETY] Box closed, glasses on, never look at LED."));
  Serial.println(F(""));

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  if (uvSensor.begin() == false) {
    Serial.println(F("[ERR] AS7331 not found"));
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
  Serial.println(F(" µW/cm²"));

  printHelp();
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'd': case 'D': doDarkCalibration(); break;
      case 'm': case 'M': runMeasurement(); break;
      case 'c': case 'C':
        Serial.println(F("\n=== CONTINUOUS MODE ==="));
        Serial.println(F("Press 's' to stop. Live readings:\n"));
        Serial.println(F("    UVA          UVB          UVC"));
        continuousMode = true;
        break;
      case 's': case 'S':
        if (continuousMode) {
          continuousMode = false;
          Serial.println(F("\n--- continuous stopped ---\n"));
        }
        break;
      case '?': printHelp(); break;
      case '\n': case '\r': case ' ': break;
      default:
        Serial.print(F("jUnknown: '"));
        Serial.print(ch); Serial.println(F("' — press '?' for help"));
        break;
    }
  }

  if (continuousMode) {
    float a, b, c;
    if (readOne(a, b, c)) {
      Serial.printf("%8.2f   %8.2f   %8.2f µW/cm²\n",
                    a - darkUVA, b - darkUVB, c - darkUVC);
    }
    delay(200);
  }
  delay(20);
}