/*
  AS7331 - UVC-Focused Calibration for XIAO ESP32-C3
  ---------------------------------------------------
  Goal: detect UVC reliably. Treat UVA/UVB as nuisance signals.

  Calibration:
    1. Dark calibration  (sensor in drawer)
    2. Sun calibration   (sensor outside in direct sun)
       Real UVC is zero outdoors -> any UVC reading is contamination.
       Model: UVC_contam = alpha * (UVA + UVB)

  In normal use:  real_UVC = measured_UVC - dark_offset - alpha*(UVA+UVB)

  Serial commands:
    0-4 : switch preset
    r   : re-run dark calibration
    c   : run sun calibration (outdoors, direct sun)
    x   : toggle UVC contamination subtraction
    p   : print all calibration values
*/

#include <Arduino.h>
#include <SparkFun_AS7331.h>
#include <Wire.h>

#define PIN_SDA           6
#define PIN_SCL           7
#define CAL_SAMPLES       20
#define SUN_SAMPLES       30
#define LOOP_DELAY_MS     1000

struct Preset {
  const char*           label;
  as7331_gain_t         gain;
  as7331_conv_time_t    time;
};

Preset presets[] = {
  { "LED-close  (GAIN_2  / 64ms)",   GAIN_2,    TIME_64MS   },
  { "UVB-lamp   (GAIN_16 / 128ms)",  GAIN_16,   TIME_128MS  },
  { "Sunlight   (GAIN_256/ 256ms)",  GAIN_256,  TIME_256MS  },
  { "Dim-indoor (GAIN_2048/512ms)",  GAIN_2048, TIME_512MS  },
  { "Max-sens   (GAIN_2048/1024ms)", GAIN_2048, TIME_1024MS },
};
const int NUM_PRESETS = sizeof(presets) / sizeof(presets[0]);

float presetFSR_UVA[NUM_PRESETS] = { 10880.0, 1512.0, 340.0, 21.25, 10.63 };
float presetFSR_UVB[NUM_PRESETS] = { 12096.0, 1512.0, 378.0, 23.63, 11.81 };
float presetFSR_UVC[NUM_PRESETS] = {  5312.0,  664.0, 166.0, 10.38,  5.19 };

struct DarkOffset {
  float uva, uvb, uvc;
  bool  valid;
};
DarkOffset darkOffsets[NUM_PRESETS];

// UVC contamination coefficient: UVC_contam = alpha * (UVA + UVB)
float alpha_UVC = 0.007f;  // initial guess: 0.7% from earlier observations
bool  alphaValid = false;
bool  contamSubEnabled = true;

int currentPreset = 0;
SfeAS7331ArdI2C myUVSensor;

// ---------- Helpers ----------

void applyPreset(int idx) {
  myUVSensor.setOperationMode(DEVICE_MODE_CFG);
  myUVSensor.setCClk(CCLK_1_024_MHZ);
  myUVSensor.setConversionTime(presets[idx].time);
  myUVSensor.setGain(presets[idx].gain);
  myUVSensor.setOperationMode(DEVICE_MODE_MEAS);
  currentPreset = idx;
}

bool takeOne(float& uva, float& uvb, float& uvc) {
  if (ksfTkErrOk != myUVSensor.setStartState(true)) return false;
  delay(5 + myUVSensor.getConversionTimeMillis());
  if (ksfTkErrOk != myUVSensor.readAllUV())         return false;
  uva = myUVSensor.getUVA();
  uvb = myUVSensor.getUVB();
  uvc = myUVSensor.getUVC();
  return true;
}

bool sampleDarkCorrected(int n, float& meanA, float& meanB, float& meanC) {
  float a, b, c;
  takeOne(a, b, c);  // discard first

  double sumA = 0, sumB = 0, sumC = 0;
  int good = 0;
  for (int s = 0; s < n; s++) {
    if (takeOne(a, b, c)) {
      if (darkOffsets[currentPreset].valid) {
        a -= darkOffsets[currentPreset].uva;
        b -= darkOffsets[currentPreset].uvb;
        c -= darkOffsets[currentPreset].uvc;
        if (a < 0) a = 0;
        if (b < 0) b = 0;
        if (c < 0) c = 0;
      }
      sumA += a; sumB += b; sumC += c;
      good++;
      Serial.print(".");
    }
  }
  Serial.println();

  if (good == 0) return false;
  meanA = sumA / good;
  meanB = sumB / good;
  meanC = sumC / good;
  return true;
}

void waitForKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) delay(100);
  while (Serial.available()) Serial.read();
}

// ---------- Calibration routines ----------

void runDarkCalibration() {
  Serial.println();
  Serial.println("====================================================");
  Serial.println("DARK CALIBRATION");
  Serial.println("Cover the sensor (drawer, foil). Press any key...");
  Serial.println("====================================================");
  waitForKey();

  Serial.println("Calibrating - keep covered until DONE.");
  Serial.println();

  for (int i = 0; i < NUM_PRESETS; i++) {
    applyPreset(i);
    Serial.print("  P"); Serial.print(i);
    Serial.print(" ["); Serial.print(presets[i].label); Serial.print("] ... ");

    float a, b, c;
    takeOne(a, b, c);

    double sumA = 0, sumB = 0, sumC = 0;
    int good = 0;
    for (int s = 0; s < CAL_SAMPLES; s++) {
      if (takeOne(a, b, c)) {
        sumA += a; sumB += b; sumC += c;
        good++;
      }
    }
    if (good == 0) {
      Serial.println("FAILED");
      darkOffsets[i].valid = false;
      continue;
    }
    darkOffsets[i].uva = sumA / good;
    darkOffsets[i].uvb = sumB / good;
    darkOffsets[i].uvc = sumC / good;
    darkOffsets[i].valid = true;

    Serial.print("A="); Serial.print(darkOffsets[i].uva, 4);
    Serial.print(" B="); Serial.print(darkOffsets[i].uvb, 4);
    Serial.print(" C="); Serial.println(darkOffsets[i].uvc, 4);
  }

  Serial.println();
  Serial.println("DONE.");
  Serial.println();
}

// Sun calibration: sensor outdoors in direct sun.
// Real UVC = 0 (atmospheric ozone), so any UVC reading is contamination.
// Fit alpha such that: measured_UVC = alpha * (UVA + UVB)
void runSunCalibration() {
  Serial.println();
  Serial.println("====================================================");
  Serial.println("SUN CALIBRATION");
  Serial.println("");
  Serial.println("Take the sensor OUTSIDE in DIRECT SUNLIGHT.");
  Serial.println("Point it at the sky (not the ground or a wall).");
  Serial.println("Real UVC outdoors is zero, so any UVC reading");
  Serial.println("is contamination from the UVA/UVB channels.");
  Serial.println("");
  Serial.println("Use a preset that does NOT saturate in direct sun.");
  Serial.println("Recommended: P1 (UVB-lamp).");
  Serial.println("");
  Serial.println("Press any key when positioned in direct sun...");
  Serial.println("====================================================");
  waitForKey();

  Serial.print("Sampling");
  float a, b, c;
  if (!sampleDarkCorrected(SUN_SAMPLES, a, b, c)) {
    Serial.println("FAILED - sampling error.");
    return;
  }
  Serial.print("  Sun:  UVA="); Serial.print(a, 3);
  Serial.print(" UVB="); Serial.print(b, 3);
  Serial.print(" UVC="); Serial.println(c, 3);

  // Sanity checks
  if (a < 100.0f) {
    Serial.println("WARNING: low UVA. Are you in direct sun?");
    Serial.println("Calibration aborted.");
    return;
  }

  // Check FSR - if any channel near saturation, alpha will be wrong
  if (a > 0.9f * presetFSR_UVA[currentPreset] ||
      b > 0.9f * presetFSR_UVB[currentPreset] ||
      c > 0.9f * presetFSR_UVC[currentPreset]) {
    Serial.println("WARNING: at least one channel is near saturation.");
    Serial.println("Switch to a less sensitive preset and try again.");
    return;
  }

  float total = a + b;
  if (total < 1.0f) {
    Serial.println("FAILED - UVA+UVB too low.");
    return;
  }

  float new_alpha = c / total;

  Serial.println();
  Serial.print("  UVA + UVB = "); Serial.print(total, 3); Serial.println(" uW/cm^2");
  Serial.print("  Apparent UVC (all contamination) = "); Serial.print(c, 4);
  Serial.println(" uW/cm^2");
  Serial.print("  Computed alpha (UVC/(UVA+UVB)) = ");
  Serial.print(new_alpha * 100, 4); Serial.println("%");

  if (new_alpha < 0.0f || new_alpha > 0.05f) {
    Serial.println("REJECTED - alpha outside expected 0-5% range.");
    return;
  }

  alpha_UVC = new_alpha;
  alphaValid = true;
  Serial.println("Coefficient UPDATED.");
  Serial.println();
}

// ---------- Display ----------

void printOffsets() {
  Serial.println();
  Serial.println("Dark offsets (uW/cm^2):");
  for (int i = 0; i < NUM_PRESETS; i++) {
    Serial.print("  P"); Serial.print(i); Serial.print(": ");
    if (!darkOffsets[i].valid) { Serial.println("<INVALID>"); continue; }
    Serial.print("A="); Serial.print(darkOffsets[i].uva, 4);
    Serial.print(" B="); Serial.print(darkOffsets[i].uvb, 4);
    Serial.print(" C="); Serial.println(darkOffsets[i].uvc, 4);
  }
  Serial.println();
  Serial.print("UVC contamination subtraction: ");
  Serial.println(contamSubEnabled ? "ENABLED" : "DISABLED");
  Serial.print("  alpha = ");
  Serial.print(alpha_UVC * 100, 4); Serial.print("%");
  Serial.println(alphaValid ? "  (calibrated)" : "  (default - run 'c' to calibrate)");
  Serial.println("  formula: UVC_corrected = UVC_measured - alpha * (UVA + UVB)");
  Serial.println();
}

void printPresetMenu() {
  Serial.println();
  Serial.println("Available presets:");
  for (int i = 0; i < NUM_PRESETS; i++) {
    Serial.print("  "); Serial.print(i); Serial.print(": ");
    Serial.println(presets[i].label);
  }
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  0-4 : switch preset");
  Serial.println("  r   : re-run dark calibration");
  Serial.println("  c   : run sun calibration (outside, direct sun)");
  Serial.println("  x   : toggle UVC contamination subtraction");
  Serial.println("  p   : print calibration values");
  Serial.println();
}

void handleSerialCommand() {
  if (!Serial.available()) return;
  char ch = Serial.read();
  while (Serial.available()) Serial.read();

  if (ch >= '0' && ch < '0' + NUM_PRESETS) {
    int idx = ch - '0';
    applyPreset(idx);
    Serial.print(">> Preset ");
    Serial.print(idx); Serial.print(" [");
    Serial.print(presets[idx].label); Serial.println("]");
  } else if (ch == 'r' || ch == 'R') {
    runDarkCalibration();
    applyPreset(currentPreset);
  } else if (ch == 'c' || ch == 'C') {
    runSunCalibration();
  } else if (ch == 'x' || ch == 'X') {
    contamSubEnabled = !contamSubEnabled;
    Serial.print(">> Contamination subtraction ");
    Serial.println(contamSubEnabled ? "ENABLED" : "DISABLED");
  } else if (ch == 'p' || ch == 'P') {
    printOffsets();
  }
}

// ---------- Arduino entry points ----------

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) delay(50);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  if (!myUVSensor.begin()) {
    Serial.println("AS7331 not found.");
    while (1) delay(1000);
  }
  if (!myUVSensor.prepareMeasurement(MEAS_MODE_CMD)) {
    Serial.println("prepareMeasurement failed.");
    while (1) delay(1000);
  }

  Serial.println();
  Serial.println("AS7331 UVC-Focused Calibration");
  Serial.println("-------------------------------");

  for (int i = 0; i < NUM_PRESETS; i++) darkOffsets[i].valid = false;

  runDarkCalibration();
  printPresetMenu();
  applyPreset(1);  // Default to UVB-lamp preset (good for sun calibration)
  Serial.println("Next: take sensor outside in direct sun and press 'c'.");
  Serial.println();
}

void loop() {
  handleSerialCommand();

  float uva, uvb, uvc;
  if (!takeOne(uva, uvb, uvc)) {
    Serial.println("Measurement error.");
    delay(LOOP_DELAY_MS);
    return;
  }

  // Step 1: dark-offset subtraction
  float corrA = uva, corrB = uvb, corrC = uvc;
  bool darkApplied = false;
  if (darkOffsets[currentPreset].valid) {
    corrA = uva - darkOffsets[currentPreset].uva;
    corrB = uvb - darkOffsets[currentPreset].uvb;
    corrC = uvc - darkOffsets[currentPreset].uvc;
    if (corrA < 0) corrA = 0;
    if (corrB < 0) corrB = 0;
    if (corrC < 0) corrC = 0;
    darkApplied = true;
  }

  // Step 2: UVC contamination subtraction
  // UVC_real = UVC_measured - alpha * (UVA + UVB)
  bool contamApplied = false;
  if (contamSubEnabled && darkApplied) {
    float predicted_contam = alpha_UVC * (corrA + corrB);
    corrC -= predicted_contam;
    if (corrC < 0) corrC = 0;
    contamApplied = true;
  }

  // Saturation check
  bool satA = corrA > 0.95f * presetFSR_UVA[currentPreset];
  bool satB = corrB > 0.95f * presetFSR_UVB[currentPreset];
  bool satC = corrC > 0.95f * presetFSR_UVC[currentPreset];
  bool saturated = satA || satB || satC;
  int suggestedPreset = currentPreset;
  if (saturated && currentPreset > 0) suggestedPreset = currentPreset - 1;

  const char* tag;
  if      (contamApplied) tag = "CLEAN";
  else if (darkApplied)   tag = "DARK";
  else                    tag = "RAW";

  Serial.print("[P"); Serial.print(currentPreset); Serial.print("] ");
  Serial.print(tag);
  Serial.print("  UVA="); Serial.print(corrA, 3);
  Serial.print("  UVB="); Serial.print(corrB, 3);
  Serial.print("  UVC="); Serial.print(corrC, 4);
  Serial.print(" uW/cm^2   (raw C: "); Serial.print(uvc, 3); Serial.print(")");

  if (saturated) {
    Serial.print("  [!] SAT on");
    if (satA) Serial.print(" A");
    if (satB) Serial.print(" B");
    if (satC) Serial.print(" C");
    if (suggestedPreset != currentPreset) {
      Serial.print(" -> P"); Serial.print(suggestedPreset);
    }
  }
  Serial.println();

  delay(LOOP_DELAY_MS);
}