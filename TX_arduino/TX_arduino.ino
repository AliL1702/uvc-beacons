  /*
  * UV-C Data Link — TRANSMITTER (Arduino Nano)
  *
  * Sends a text message via the UV-C LED using Manchester-encoded OOK.
  * Bit rate configurable. Uses the existing LED + LDD-350LW + DIM pin setup.
  *
  * Frame format:
  *   [PREAMBLE: 0xAA 0xAA  (alternating)]
  *   [START:    0x7E       (delimiter)]
  *   [LENGTH:   N          (payload bytes)]
  *   [PAYLOAD:  N bytes]
  *   [CHECKSUM: XOR of payload]
  *
  * Manchester encoding:
  *   bit 0 = LOW then HIGH  (rising edge in middle)
  *   bit 1 = HIGH then LOW  (falling edge in middle)
  *
  * Wiring:
  *   D9   -> LDD-350LW WHITE wire (DIM)
  *   GND  -> LDD black wire AND 12V supply negative (common ground!)
  *   10kΩ pulldown from D9 to GND  (safety)
  *
  * Serial commands (115200 baud):
  *   t<message>   — transmit a message at current bit rate (e.g. "tHELLO")
  *   b<rate>      — set bit rate in bps (e.g. "b20")
  *   r            — run rate sweep test (sends pattern at 5, 10, 20, 30, 50 bps)
  *   p<bps>       — send periodic test pattern at bps for 5 seconds
  *   ?            — help
  */

  #define LED_PIN 9
  #define DEFAULT_BPS 20
  #define MAX_MSG_LEN 80

  uint16_t bitRate = DEFAULT_BPS;
  unsigned long chipUs;  // microseconds per Manchester chip (half a bit)

  void setBitRate(uint16_t bps) {
    if (bps < 1) bps = 1;
    if (bps > 200) bps = 200;
    bitRate = bps;
    chipUs = 500000UL / bps;  // half-bit period in microseconds
    Serial.print(F("Bit rate set to "));
    Serial.print(bps);
    Serial.print(F(" bps (chip period "));
    Serial.print(chipUs);
    Serial.println(F(" us)"));
  }

  inline void txHigh() { digitalWrite(LED_PIN, HIGH); }
  inline void txLow()  { digitalWrite(LED_PIN, LOW);  }

  // Safe delay — handles values > 16383 us correctly by splitting into ms + us.
  // delayMicroseconds() is unreliable for values larger than ~16383.
  void delayChip() {
    unsigned long us = chipUs;
    unsigned long ms = us / 1000;
    unsigned int rem = us % 1000;
    if (ms > 0) delay(ms);
    if (rem > 0) delayMicroseconds(rem);
  }

  // Manchester: bit=0 -> LOW,HIGH ; bit=1 -> HIGH,LOW
  void sendBit(uint8_t b) {
    if (b) {
      txHigh();
      delayChip();
      txLow();
      delayChip();
    } else {
      txLow();
      delayChip();
      txHigh();
      delayChip();
    }
  }

  void sendByte(uint8_t v) {
    // MSB first
    for (int i = 7; i >= 0; i--) {
      sendBit((v >> i) & 1);
    }
  }

  void sendFrame(const uint8_t *payload, uint8_t len) {
    // Preamble — give receiver time to lock
    sendByte(0xAA);
    sendByte(0xAA);
    // Start delimiter
    sendByte(0x7E);
    // Length
    sendByte(len);
    // Payload + checksum
    uint8_t cksum = 0;
    for (uint8_t i = 0; i < len; i++) {
      sendByte(payload[i]);
      cksum ^= payload[i];
    }
    sendByte(cksum);
    // Trailing low to settle
    txLow();
  }

  void transmitMessage(const char *msg) {
    uint8_t len = strlen(msg);
    if (len > MAX_MSG_LEN) len = MAX_MSG_LEN;
    Serial.print(F("Transmitting \""));
    Serial.print(msg);
    Serial.print(F("\" ("));
    Serial.print(len);
    Serial.print(F(" bytes) at "));
    Serial.print(bitRate);
    Serial.println(F(" bps..."));
    unsigned long startMs = millis();
    sendFrame((const uint8_t*)msg, len);
    unsigned long elapsed = millis() - startMs;
    Serial.print(F("Done in "));
    Serial.print(elapsed);
    Serial.println(F(" ms"));
  }

  void rateSweep() {
    const uint16_t rates[] = {5, 10, 20, 30, 50};
    const char *testMsg = "TEST123";
    for (uint8_t i = 0; i < 5; i++) {
      setBitRate(rates[i]);
      delay(2000);  // gap between transmissions
      transmitMessage(testMsg);
    }
    setBitRate(DEFAULT_BPS);
    Serial.println(F("Sweep complete."));
  }

  void periodicTest(uint16_t bps) {
    setBitRate(bps);
    unsigned long endMs = millis() + 5000;
    while ((long)(millis() - endMs) < 0) {
      transmitMessage("PING");
      delay(500);
    }
  }

  void printHelp() {
    Serial.println(F("\nUV-C TX commands:"));
    Serial.println(F("  t<msg>   — transmit message"));
    Serial.println(F("  b<bps>   — set bit rate"));
    Serial.println(F("  r        — rate sweep test (5/10/20/30/50 bps)"));
    Serial.println(F("  p<bps>   — periodic ping for 5 sec at given bps"));
    Serial.println(F("  ?        — help"));
  }

  void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("\n=== UV-C TX (Nano) ==="));
    setBitRate(DEFAULT_BPS);
    printHelp();
  }

  void loop() {
    if (!Serial.available()) return;
    char ch = Serial.read();
    if (ch == 't' || ch == 'T') {
      char buf[MAX_MSG_LEN + 1];
      uint8_t i = 0;
      unsigned long endMs = millis() + 1000;  // 1 sec timeout for line
      while (i < MAX_MSG_LEN && (long)(millis() - endMs) < 0) {
        if (Serial.available()) {
          char c = Serial.read();
          if (c == '\n' || c == '\r') break;
          buf[i++] = c;
          endMs = millis() + 200;
        }
      }
      buf[i] = 0;
      if (i > 0) transmitMessage(buf);
    } else if (ch == 'b' || ch == 'B') {
      uint16_t v = Serial.parseInt();
      if (v > 0) setBitRate(v);
    } else if (ch == 'p' || ch == 'P') {
      uint16_t v = Serial.parseInt();
      if (v > 0) periodicTest(v);
    } else if (ch == 'r' || ch == 'R') {
      rateSweep();
    } else if (ch == '?') {
      printHelp();
    }
    // ignore whitespace silently
  }