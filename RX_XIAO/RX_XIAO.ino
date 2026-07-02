/*
 * UV-C Data Link — RECEIVER (XIAO ESP32-C3) — DECODER v4
 *
 * Sensor: GAIN_64 / TIME_8MS, ~94 samples/sec.
 * Decoder: Manchester via transition timing, with bit-level start byte
 * detection (looks for 0x7E pattern in the bit stream rather than relying
 * on byte alignment from the first transition).
 *
 * Frame format (must match transmitter):
 *   PREAMBLE:  0xAA 0xAA  (alternating, for sync)
 *   START:     0x7E
 *   LENGTH:    N
 *   PAYLOAD:   N bytes
 *   CHECKSUM:  XOR of payload
 *
 * Wiring:
 *   GPIO 6 -> AS7331 SDA, GPIO 7 -> SCL, 3V3 -> VCC, GND -> GND
 *
 * Serial commands:
 *   r        — reset stats and decoder
 *   m        — toggle live monitor
 *   t        — show stats
 *   b<bps>   — set expected bit rate (must match TX!)
 *   v        — toggle verbose decode (shows bits/bytes as they arrive)
 *   ?        — help
 */

#include <Wire.h>
#include "SparkFun_AS7331.h"

#define I2C_SDA 6
#define I2C_SCL 7

SfeAS7331ArdI2C uvSensor;

// --- decoder config ---
uint16_t bitRate = 5;
unsigned long chipUs = 100000;
float threshold = 4.0;

// --- monitor / stats ---
bool monitorMode = false;
bool verboseDecode = false;
float runMin = 999999.0, runMax = 0;
double runSum = 0;
uint32_t sampleCount = 0;

// --- decoder state ---
unsigned long lastTransitionUs = 0;
unsigned long frameStartMs = 0;
uint8_t lastLevel = 0;
bool synced = false;
uint8_t bitBuf = 0;
uint8_t bitCount = 0;

enum FrameState {
  FS_HUNT_START,
  FS_GOT_LEN,
  FS_PAYLOAD,
  FS_CHECKSUM
};
FrameState fState = FS_HUNT_START;
uint8_t frLen = 0;
uint8_t frRecv = 0;
uint8_t frBuf[256];
uint8_t frChecksum = 0;

void setBitRate(uint16_t bps) {
  if (bps < 1) bps = 1;
  bitRate = bps;
  chipUs = 500000UL / bps;
  Serial.print(F("RX bit rate: "));
  Serial.print(bps);
  Serial.print(F(" bps (chip = "));
  Serial.print(chipUs / 1000);
  Serial.println(F(" ms)"));
}

void resetDecoder() {
  synced = false;
  bitBuf = 0;
  bitCount = 0;
  fState = FS_HUNT_START;
  lastTransitionUs = 0;
}

void resetStats() {
  runMin = 999999.0;
  runMax = 0;
  runSum = 0;
  sampleCount = 0;
  resetDecoder();
  Serial.println(F("[RX] Reset."));
}

bool readUVC(float &uvc) {
  if (uvSensor.setStartState(true) != 0) return false;
  delay(uvSensor.getConversionTimeMillis() + 2);
  if (uvSensor.readAllUV() != 0) return false;
  uvc = uvSensor.getUVC();
  return true;
}

void onByte(uint8_t b) {
  if (verboseDecode) {
    Serial.print(F("  byte: 0x"));
    if (b < 0x10) Serial.print('0');
    Serial.print(b, HEX);
    Serial.print(F(" ('"));
    Serial.write((b >= 32 && b < 127) ? (char)b : '.');
    Serial.println(F("')"));
  }
  switch (fState) {
    case FS_HUNT_START:
      // Should never reach here — bit-level hunt handles this state
      break;
    case FS_GOT_LEN:
      frLen = b;
      frRecv = 0;
      frChecksum = 0;
      if (frLen == 0 || frLen > 200) {
        Serial.print(F("[RX] Bad len: "));
        Serial.println(frLen);
        fState = FS_HUNT_START;
      } else {
        fState = FS_PAYLOAD;
        if (verboseDecode) {
          Serial.print(F("  -> length="));
          Serial.println(frLen);
        }
      }
      break;
    case FS_PAYLOAD:
      frBuf[frRecv++] = b;
      frChecksum ^= b;
      if (frRecv >= frLen) fState = FS_CHECKSUM;
      break;
    case FS_CHECKSUM:
      if (b == frChecksum) {
        unsigned long elapsed = millis() - frameStartMs;
        Serial.print(F("[RX] OK: \""));
        for (uint8_t i = 0; i < frLen; i++) {
          char c = frBuf[i];
          Serial.write((c >= 32 && c < 127) ? c : '.');
        }
        Serial.print(F("\"  ("));
        Serial.print(frLen);
        Serial.print(F(" bytes in "));
        Serial.print(elapsed);
        Serial.print(F(" ms, "));
        // Effective payload bit rate (excluding preamble+start+len+checksum overhead):
        // Just the payload in those ms.
        Serial.print((float)(frLen * 8 * 1000) / elapsed, 2);
        Serial.println(F(" bps payload)"));
      } else {
        Serial.print(F("[RX] CRC FAIL: got 0x"));
        Serial.print(b, HEX);
        Serial.print(F(" expected 0x"));
        Serial.println(frChecksum, HEX);
      }
      fState = FS_HUNT_START;
      break;
  }
}

void onBit(uint8_t bit) {
  if (verboseDecode) Serial.print(bit);

  // Always shift new bit into the rolling 8-bit window
  bitBuf = ((bitBuf << 1) | (bit & 1)) & 0xFF;

  if (fState == FS_HUNT_START) {
    // Look for the start byte (0x7E) bit-by-bit, no byte alignment yet.
    // Once we see it, we know we're aligned and can switch to byte mode.
    if (bitBuf == 0x7E) {
      if (verboseDecode) Serial.println(F("\n  -> got START (bit-aligned)"));
      fState = FS_GOT_LEN;
      bitCount = 0;
      bitBuf = 0;
      frameStartMs = millis();
    }
  } else {
    // We're locked. Count 8 bits per byte.
    bitCount++;
    if (bitCount >= 8) {
      onByte(bitBuf);
      bitCount = 0;
      bitBuf = 0;
    }
  }
}

// Manchester decode based on transition timing.
void onTransition(uint8_t newLevel, unsigned long now) {
  if (lastTransitionUs == 0) {
    lastTransitionUs = now;
    lastLevel = newLevel;
    synced = false;
    return;
  }
  unsigned long delta = now - lastTransitionUs;
  lastTransitionUs = now;

  bool isLong = (delta > (chipUs * 3 / 2));

  if (!synced) {
    if (!isLong) synced = true;
    lastLevel = newLevel;
    return;
  }

  static bool nextShortIsBitMid = false;

  if (isLong) {
    uint8_t bit = (newLevel == 1) ? 0 : 1;
    onBit(bit);
    nextShortIsBitMid = false;
  } else {
    if (nextShortIsBitMid) {
      uint8_t bit = (newLevel == 1) ? 0 : 1;
      onBit(bit);
      nextShortIsBitMid = false;
    } else {
      nextShortIsBitMid = true;
    }
  }

  lastLevel = newLevel;
}

void printHelp() {
  Serial.println(F("\nRX commands:"));
  Serial.println(F("  r       — reset"));
  Serial.println(F("  m       — toggle live monitor"));
  Serial.println(F("  t       — show stats"));
  Serial.println(F("  b<bps>  — set bit rate (match TX!)"));
  Serial.println(F("  v       — toggle verbose decode"));
  Serial.println(F("  ?       — help"));
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println(F("\n=== UV-C RX (XIAO) — DECODER v4 ==="));

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (uvSensor.begin() == false) {
    Serial.println(F("[ERR] AS7331 not found"));
    while (1) delay(1000);
  }
  Serial.println(F("AS7331 found."));

  uvSensor.setGain(GAIN_64);
  uvSensor.setConversionTime(TIME_8MS);
  uvSensor.setCClk(CCLK_1_024_MHZ);

  if (uvSensor.prepareMeasurement(MEAS_MODE_CMD) == false) {
    Serial.println(F("[ERR] prepareMeasurement failed"));
    while (1) delay(1000);
  }

  setBitRate(5);
  Serial.print(F("Threshold: "));
  Serial.print(threshold);
  Serial.println(F(" uW/cm2"));
  Serial.println(F("Listening..."));
  printHelp();
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'r' || ch == 'R') {
      resetStats();
    } else if (ch == 'm' || ch == 'M') {
      monitorMode = !monitorMode;
      Serial.print(F("Monitor: "));
      Serial.println(monitorMode ? "ON" : "OFF");
    } else if (ch == 't' || ch == 'T') {
      Serial.print(F("samples="));
      Serial.print(sampleCount);
      if (sampleCount > 0) {
        Serial.print(F("  avg="));
        Serial.print(runSum / sampleCount, 2);
      }
      Serial.print(F("  min="));
      Serial.print(runMin, 2);
      Serial.print(F("  max="));
      Serial.println(runMax, 2);
    } else if (ch == 'b' || ch == 'B') {
      uint16_t v = Serial.parseInt();
      if (v > 0) {
        setBitRate(v);
        resetDecoder();
      }
    } else if (ch == 'v' || ch == 'V') {
      verboseDecode = !verboseDecode;
      Serial.print(F("Verbose: "));
      Serial.println(verboseDecode ? "ON" : "OFF");
    } else if (ch == '?') {
      printHelp();
    }
  }

  float uvc;
  if (!readUVC(uvc)) return;

  if (uvc < runMin) runMin = uvc;
  if (uvc > runMax) runMax = uvc;
  runSum += uvc;
  sampleCount++;

  if (monitorMode) {
    Serial.print(F("UVC="));
    Serial.println(uvc, 2);
  }

  uint8_t level = (uvc > threshold) ? 1 : 0;
  unsigned long now = micros();
  if (level != lastLevel) {
    onTransition(level, now);
  }
}