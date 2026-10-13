/*
 * PIC16F72 high-voltage ICSP programmer engine
 * Port of Tahmid Mahbub's proven prog72.py (RP2040) to Arduino Uno/Nano.
 *   https://github.com/SyedTahmidMahbub/q-dpico_picprog72
 * Algorithm per Microchip DS39588A (PIC16F72 Flash Programming Specification).
 *
 * ---------------------------------------------------------------------------
 * WIRING (PIC16F72 PDIP-28) — SOIC pins in brackets
 *
 *   VPP switch (Q1 NPN pre-driver + Q2 PNP high-side):
 *     D9 --1k-- Q1 base (NPN: BC547/2N2222)  10k from Q1 base to GND
 *     Q1 emitter -- GND                      Q1 collector --4.7k-- Q2 base
 *     Q2 (PNP: BC557/2N3906) emitter -- boost +13V
 *     10k between Q2 base and emitter
 *     Q2 collector --100R-- PIC pin 1 (MCLR/VPP)
 *     10k from PIC pin 1 to GND (MCLR pulldown when VPP is off)
 *       D9 HIGH = VPP on  (~13V on MCLR)
 *       D9 LOW  = VPP off (MCLR at GND, device held in reset)
 *     A single PNP cannot be switched directly from D9: with its emitter
 *     at 13V, a 0/5V pin can never pull the base up high enough to turn
 *     it off. Q1 does the level shifting. (The original prog72.py hardware
 *     used a PC817 optocoupler for the same reason. PC817 works here too,
 *     same polarity: D9--330R--LED side, collector to +13V, emitter to
 *     MCLR, 10k MCLR to GND.)
 *
 *   Arduino D10 -- PIC pin 28 RB7/PGD  [SOIC 25]
 *   Arduino D11 -- PIC pin 27 RB6/PGC  [SOIC 24]
 *   Arduino 5V  -- PIC pin 20 VDD      MUST measure 4.75-5.25V at the pin
 *   Arduino GND -- PIC pin 8 + pin 19 (VSS), and boost converter GND
 *
 *   Boost converter: input from Arduino 5V, output set to 13.2V
 *   (the prog72.py target; spec window at MCLR is 12.75-13.25V — the
 *   100R and 10k eat ~0.3V). Set and verify with a meter BEFORE
 *   connecting MCLR. If VDD at pin 20 sags below 4.75V on USB power,
 *   power the Arduino from VIN (7-9V) instead.
 *
 * ---------------------------------------------------------------------------
 * SERIAL PROTOCOL — 115200 baud, 8N1, ASCII commands terminated by '\n'
 *   ?        -> P72        handshake
 *   E        -> OK         enter programming mode (PC resets to 0x0000)
 *   X        -> OK         exit programming mode (VPP off)
 *   I        -> Dxxxx      read device-ID word at 0x2006 (4 hex digits)
 *   A        -> OK         chip erase (blocks ~30 ms)
 *   C        -> OK         load configuration (PC = 0x2000)
 *   N        -> OK         increment address
 *   Pxxxx    -> OK         program word xxxx at PC, then increment PC
 *   R        -> Dxxxx      read word at PC (no increment)
 *   Gxxxx    -> xxxx...    read xxxx consecutive words from PC (increments)
 *   anything else -> ER
 */

#define VPP_PIN    9
#define PGD_PIN    10
#define PGC_PIN    11
#define LED_PIN    13        // onboard LED lights while VPP is active

// DS39588A Table 2-2 command codes
#define CMD_LOAD_CONFIG 0x00
#define CMD_LOAD_DATA   0x02
#define CMD_READ_DATA   0x04
#define CMD_INC_ADDR    0x06
#define CMD_BEGIN_PROG  0x08
#define CMD_BULK_ERASE  0x09
#define CMD_END_PROG    0x0E

// DS39588A Table 5-1 (all minimums; digitalWrite is far slower, so easily met)
#define TPROG_MS   3         // tprog 1-3 ms (value proven in prog72.py)
#define TERASE_MS  30        // tera 30 ms, self-timed

#define VPP_ON  HIGH         // Q1 NPN pre-driver + Q2 PNP high-side switch
#define VPP_OFF LOW

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Write output levels BEFORE switching to OUTPUT, so D9 comes up in
  // its idle state (LOW = VPP off) with no glitch. While D9 is Hi-Z at
  // boot, R2 (10k, Q1 base to GND) already holds VPP off.
  digitalWrite(VPP_PIN, VPP_OFF);
  digitalWrite(PGD_PIN, LOW);
  digitalWrite(PGC_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  pinMode(VPP_PIN, OUTPUT);
  pinMode(PGD_PIN, OUTPUT);
  pinMode(PGC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
}

// --- low-level ICSP bit-banging -------------------------------------------

void icspCommand(uint8_t cmd) {
  cmd &= 0x3F;
  for (uint8_t i = 0; i < 6; i++) {        // 6 bits, LSB first,
    digitalWrite(PGC_PIN, HIGH);           // latched on falling edge of PGC
    digitalWrite(PGD_PIN, (cmd >> i) & 1);
    delayMicroseconds(1);                  // tset1 >= 100 ns
    digitalWrite(PGC_PIN, LOW);
    delayMicroseconds(1);                  // thld1 >= 100 ns
  }
  delayMicroseconds(2);                    // tdly1 >= 1 us
}

void icspWriteData(uint16_t data) {
  uint16_t x = (data & 0x3FFF) << 1;       // start bit 0, 14 data bits, stop bit 0
  pinMode(PGD_PIN, OUTPUT);
  for (uint8_t i = 0; i < 16; i++) {       // LSB first
    digitalWrite(PGC_PIN, HIGH);
    digitalWrite(PGD_PIN, x & 1);
    delayMicroseconds(1);
    digitalWrite(PGC_PIN, LOW);
    delayMicroseconds(1);
    x >>= 1;
  }
  delayMicroseconds(2);                    // tdly1 >= 1 us
}

uint16_t icspReadData() {
  uint16_t x = 0;
  pinMode(PGD_PIN, INPUT);                 // PIC drives RB7 during the read
  for (uint8_t i = 0; i < 16; i++) {
    digitalWrite(PGC_PIN, HIGH);
    delayMicroseconds(1);                  // tdly3 <= 200 ns until data valid
    digitalWrite(PGC_PIN, LOW);
    uint8_t b = digitalRead(PGD_PIN);
    delayMicroseconds(1);
    x = (x >> 1) | ((uint16_t)b << 15);
  }
  pinMode(PGD_PIN, OUTPUT);
  digitalWrite(PGD_PIN, LOW);
  delayMicroseconds(2);
  return (x >> 1) & 0x3FFF;                // drop stop bit, mask out start bit
}

// --- mid-level operations ---------------------------------------------------

void vppOn()  { digitalWrite(VPP_PIN, VPP_ON);  digitalWrite(LED_PIN, HIGH); }
void vppOff() { digitalWrite(VPP_PIN, VPP_OFF); digitalWrite(LED_PIN, LOW); }

void enterProgMode() {
  // RB6/RB7 must be held low while MCLR rises from VIL to VPP (DS39588A 2.3)
  digitalWrite(PGD_PIN, LOW);
  digitalWrite(PGC_PIN, LOW);
  vppOff();
  delay(100);                  // MCLR discharges to VIL through 10k pulldown
  vppOn();
  delay(10);                   // settle in programming mode; PC = 0x0000
}

void exitProgMode() {
  vppOff();                    // MCLR below VIL: device leaves programming mode
  digitalWrite(PGD_PIN, LOW);
  digitalWrite(PGC_PIN, LOW);
  delay(10);
}

void loadConfig() {            // PC = 0x2000; the data word is discarded
  icspCommand(CMD_LOAD_CONFIG);
  icspWriteData(0);
}

void programWord(uint16_t data) {
  icspCommand(CMD_LOAD_DATA);
  icspWriteData(data);
  icspCommand(CMD_BEGIN_PROG);
  delay(TPROG_MS);
  icspCommand(CMD_END_PROG);
  icspCommand(CMD_INC_ADDR);
}

void chipErase() {
  icspCommand(CMD_BULK_ERASE);
  delay(TERASE_MS);            // self-timed
}

uint16_t readWord() {
  icspCommand(CMD_READ_DATA);
  return icspReadData();
}

uint16_t readDeviceId() {      // mirrors prog72 find_target()
  enterProgMode();
  loadConfig();                // PC = 0x2000
  for (uint8_t i = 0; i < 6; i++) {   // -> 0x2006
    icspCommand(CMD_INC_ADDR);
  }
  return readWord();
}

// --- serial command interpreter ----------------------------------------------

void printHex4(uint16_t v) {
  if (v < 0x1000) Serial.print('0');
  if (v < 0x0100) Serial.print('0');
  if (v < 0x0010) Serial.print('0');
  Serial.print(v, HEX);
}

bool isHexWord(const char *s) {   // exactly 4 hex digits
  if (s[4] != '\0') return false;
  for (uint8_t i = 0; i < 4; i++) {
    char c = s[i];
    bool ok = (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    if (!ok) return false;
  }
  return true;
}

void loop() {
  static char buf[8];
  if (!Serial.available()) return;
  memset(buf, 0, sizeof(buf));            // no stale bytes from previous commands
  size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  while (n > 0 && (buf[n - 1] == '\r' || buf[n - 1] == ' ')) n--;
  buf[n] = '\0';
  if (n == 0) return;

  switch (buf[0]) {
    case '?':
      Serial.println(F("P72"));
      break;

    case 'E':
      enterProgMode();
      Serial.println(F("OK"));
      break;

    case 'X':
      exitProgMode();
      Serial.println(F("OK"));
      break;

    case 'I': {
      uint16_t id = readDeviceId();
      Serial.print('D');
      printHex4(id);
      Serial.println();
      break;
    }

    case 'A':
      chipErase();
      Serial.println(F("OK"));
      break;

    case 'C':
      loadConfig();
      Serial.println(F("OK"));
      break;

    case 'N':
      icspCommand(CMD_INC_ADDR);
      Serial.println(F("OK"));
      break;

    case 'R': {
      uint16_t w = readWord();
      Serial.print('D');
      printHex4(w);
      Serial.println();
      break;
    }

    case 'P': {
      if (!isHexWord(buf + 1)) { Serial.println(F("ER")); break; }
      programWord((uint16_t)strtoul(buf + 1, NULL, 16));
      Serial.println(F("OK"));
      break;
    }

    case 'G': {
      if (!isHexWord(buf + 1)) { Serial.println(F("ER")); break; }
      uint16_t cnt = (uint16_t)strtoul(buf + 1, NULL, 16);
      if (cnt == 0 || cnt > 0x0800) { Serial.println(F("ER")); break; }
      for (uint16_t i = 0; i < cnt; i++) {
        printHex4(readWord());
        icspCommand(CMD_INC_ADDR);
      }
      Serial.println();
      break;
    }

    default:
      Serial.println(F("ER"));
  }
}
