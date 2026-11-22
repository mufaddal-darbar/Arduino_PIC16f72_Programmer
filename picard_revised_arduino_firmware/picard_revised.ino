/*
 * PIC16F72 high-voltage ICSP programmer engine
 * PicArd Revised Edition — for Nelson Bittencourt's PicArd shield circuit
 * (Rev 3: onboard boost converter, Q1/Q2 VPP switch, Q3 VDD switch).
 *
 * ICSP routines, command codes and serial protocol are unchanged from the
 * working pic16f72_icsp sketch; only the pin map, the VDD switch and the
 * boost-converter regulator are new.
 *
 * Algorithm per Microchip DS39588A (PIC16F72 Flash Programming Specification).
 *
 * ---------------------------------------------------------------------------
 * WIRING (PicArd Rev 3, as built)
 *
 *   Arduino 5V  -- +5V rail: C3 470uF, C4 100nF, R10 56R, Q3 emitter
 *   Arduino GND -- common ground
 *   Arduino D5  -- ICSP_DATA  -- PIC pin 28 RB7/PGD   (ZIF pin 40)
 *   Arduino D6  -- ICSP_CLOCK -- PIC pin 27 RB6/PGC   (ZIF pin 39)
 *   Arduino D8  -- 4.7k R8 -- Q3 base   (BC557 PNP high-side VDD switch)
 *                  Q3 collector = VDD_DEVICE, R9 10k from collector to GND
 *                  D8 LOW  = VDD on     (PNP with emitter on +5V)
 *                  D8 HIGH = VDD off
 *   Arduino D9  -- 4.7k R4 -- Q1 base   (BC547 NPN)
 *                  R5 100k from +13V rail to Q2 base, Q1 collector
 *                  through R6 10k to Q2 base, Q2 BC557 emitter on +13V,
 *                  Q2 collector = VPP_DEVICE, R7 10k to GND
 *                  D9 HIGH = VPP on (13V on MCLR), D9 LOW = VPP off
 *   Arduino D10 -- 1k R1 -- Q4 base (BC547), C2 1nF base-emitter
 *                  Q4 low-side switch of the boost converter
 *   Arduino A0  -- PWM_FB: 4k7 R2 from VPP rail, 2.61k R3 to GND,
 *                  D1 1N5231 (5.1V) clamps A0
 *
 *   Boost stage: +5V --R10 56R-- L1 2.2mH -- Q4 collector,
 *                L1/Q4 node -- D2 1N5817 --> VPP rail -- C1 47uF
 *
 *   The boost is PWM'd at 20 kHz (Timer1 fast PWM, OC1B) and regulated by
 *   a duty ramp driven from A0. It is only run while in programming mode,
 *   and reaches ~13V before VDD and VPP are applied to the PIC.
 *
 * ---------------------------------------------------------------------------
 * CALIBRATION (only needed if VPP is not 12.75-13.25V on MCLR)
 *   Default target is ADC 950 (13.0V with R2=4k7, R3=2.61k).
 *   python picard_host.py --port COMx --boost
 *       enters programming mode, keeps VPP on, prints the A0 reading.
 *       Measure VPP on the PIC's MCLR pin with a meter, enter it when
 *       asked, and the script prints the corrected target.
 *   python picard_host.py --port COMx --set-target N
 *       stores target N (in EEPROM, survives power cycles).
 *
 * ---------------------------------------------------------------------------
 * SERIAL PROTOCOL — 115200 baud, 8N1, ASCII commands terminated by '\n'
 *   ?        -> P72        handshake
 *   E        -> OK         enter programming mode (PC resets to 0x0000)
 *   X        -> OK         exit programming mode (VPP, VDD and boost off)
 *   I        -> Dxxxx      read device-ID word at 0x2006 (4 hex digits)
 *   A        -> OK         chip erase (blocks ~30 ms)
 *   C        -> OK         load configuration (PC = 0x2000)
 *   N        -> OK         increment address
 *   Pxxxx    -> OK         program word xxxx at PC, then increment PC
 *   R        -> Dxxxx      read word at PC (no increment)
 *   Gxxxx    -> xxxx...    read xxxx consecutive words from PC (increments)
 *   V        -> Dxxxx      enter programming mode, report A0 feedback reading
 *   Vxxxx    -> OK         set boost target (ADC counts) and save to EEPROM
 *   W        -> Dxxxx      report boost target
 *   S        -> Dxxxx yyyy report live A0 reading and PWM duty (0-800)
 *   T        -> Dxxxx yyyy zzzz
 *                          A0 lowest, A0 highest and peak PWM duty since
 *                          programming mode was entered (VPP droop check)
 *   Qxxxx    -> Dxxxx yyyy zzzz tttt
 *                          worst-case load test: program xxxx all-zero words
 *                          at the real 13 ms cadence, then report A0 low,
 *                          A0 high, peak duty and elapsed ms. Writes 0x0000
 *                          into the chip - erase afterwards.
 *   B        -> Dxxxx yyyy mmmm nnnn
 *                          boost self-test (VDD/VPP stay off): runs 1.2 s with
 *                          no load, reports final ADC, duty, and the min/max
 *                          ADC over the last 400 ms (oscillation check)
 *   EV       <-            VPP regulation failed (boost did not reach target)
 *   anything else -> ER
 *
 * If no command arrives for 120 s while in programming mode, VPP/VDD/boost
 * are dropped automatically (protects the chip if the host goes away).
 */

#include <EEPROM.h>

#define PGD_PIN    5
#define PGC_PIN    6
#define VDD_PIN    8
#define VPP_PIN    9
#define PWM_PIN    10        // OC1B
#define FB_PIN     A0
#define LED_PIN    13        // onboard LED lights while VPP is active

#define VDD_ON   LOW         // Q3 BC557 PNP, emitter on +5V
#define VDD_OFF  HIGH
#define VPP_ON   HIGH        // Q1 BC547 turns Q2 BC557 on
#define VPP_OFF  LOW

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

// --- boost regulator tuning -------------------------------------------------
// 16 MHz / (799 + 1) = 20 kHz. Equilibrium duty at 13V is ~215 counts.
#define PWM_TOP       799
#define DUTY_MAX      560    // hard duty ceiling (~70%); this stage needs ~450
                             // at 13V, so 50% left the loop with no headroom
#define ADC_DEADBAND  3      // +-3 LSB (~0.05V) around target
#define CTRL_DIV      4      // duty step every 4 ADC samples (~416 us)
#define SETTLE_TOL    12     // |adc - target| accepted for programming mode
                             // (~0.18V; a looser window let a run start with
                             // VPP already above the 13.25V datasheet limit)
#define SETTLE_MS     800    // boost ramp timeout
#define MCLR_MS       100    // MCLR discharge before VPP (DS39588A 2.3)

#define EE_MAGIC_ADDR 0
#define EE_TARGET_ADDR 1
#define EE_MAGIC      0x72
#define TARGET_DEFAULT 950   // 13.0V with R2=4k7, R3=2.61k
#define IDLE_TIMEOUT_MS 120000UL

// --- regulator state (shared with the ADC ISR) ------------------------------
volatile bool     boostOn = false;
volatile uint16_t adcTarget = TARGET_DEFAULT;
volatile uint16_t adcAvg = 0;
volatile uint16_t dutyNow = 0;
volatile uint8_t  avgIdx = 0;
volatile uint16_t avgBuf[4];
volatile uint32_t avgSum = 0;
volatile uint8_t  ctrlDiv = 0;

volatile bool progMode = false;
volatile uint16_t adcMin = 1023; // extremes seen since programming mode began,
volatile uint16_t adcMax = 0;    // reported by 'T' — shows VPP sag during the
volatile uint16_t dutyPeak = 0;  // programming pulses
uint32_t boostOffAt = 0;         // boost self-test hold time (0 = not held)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Idle levels before the pins become outputs, so nothing glitches on at
  // boot: VPP off, VDD off, clock/data low, boost switch off.
  digitalWrite(VPP_PIN, VPP_OFF);
  digitalWrite(PGD_PIN, LOW);
  digitalWrite(PGC_PIN, LOW);
  digitalWrite(PWM_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  pinMode(VPP_PIN, OUTPUT);
  pinMode(PGD_PIN, OUTPUT);
  pinMode(PGC_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Q3 base floats while the pin is Hi-Z; the internal pull-up holds it at
  // 5V (= VDD off) until we take the pin as an output.
  digitalWrite(VDD_PIN, VDD_OFF);
  pinMode(VDD_PIN, INPUT_PULLUP);
  pinMode(VDD_PIN, OUTPUT);
  digitalWrite(VDD_PIN, VDD_OFF);

  // Timer1 fast PWM, mode 14: f = 16MHz / (1 + ICR1)
  TCCR1A = _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = PWM_TOP;
  OCR1B = 0;
  pinMode(PWM_PIN, OUTPUT);

  // ADC free-running on A0, AVcc reference, /128 -> ~9.6 kSPS
  ADMUX = _BV(REFS0);
  ADCSRB = 0;
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) |
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  ADCSRA |= _BV(ADSC);

  if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC) {
    uint16_t t = EEPROM.read(EE_TARGET_ADDR) |
                 ((uint16_t)EEPROM.read(EE_TARGET_ADDR + 1) << 8);
    if (t >= 300 && t <= 1023) adcTarget = t;
  }
}

// --- boost regulator --------------------------------------------------------

ISR(ADC_vect) {
  uint16_t v = ADC;                      // reading ADCx clears ADIF

  avgSum -= avgBuf[avgIdx];
  avgBuf[avgIdx] = v;
  avgSum += v;
  avgIdx = (avgIdx + 1) & 3;

  if (++ctrlDiv < CTRL_DIV) return;
  ctrlDiv = 0;

  uint16_t a = (uint16_t)(avgSum >> 2);
  adcAvg = a;

  if (!boostOn) {
    dutyNow = 0;
  } else if (a >= 1020) {
    // Feedback is pinned at the top of the range (zener clamp): back off
    // hard. The loop cannot see how far above target the rail really is.
    dutyNow = 0;
  } else if (a + ADC_DEADBAND < adcTarget) {
    if (dutyNow < DUTY_MAX) dutyNow++;
  } else if (a > adcTarget + ADC_DEADBAND) {
    uint16_t over = a - adcTarget;
    dutyNow = (dutyNow > over) ? dutyNow - over : 0;
  }

  if (progMode) {                        // track the rail while the chip is
    if (a < adcMin) adcMin = a;          // being programmed, so a droop during
    if (a > adcMax) adcMax = a;          // the programming pulses is visible
    if (dutyNow > dutyPeak) dutyPeak = dutyNow;
  }
  OCR1B = dutyNow;
}

void boostStart() {
  noInterrupts();
  dutyNow = 0;
  OCR1B = 0;                             // start the ramp from zero
  avgSum = 0;
  avgIdx = 0;
  for (uint8_t i = 0; i < 4; i++) avgBuf[i] = 0;
  adcAvg = 0;                            // no stale reading in boostSettle()
  boostOn = true;
  interrupts();
}

void boostStop() {
  noInterrupts();
  boostOn = false;
  dutyNow = 0;
  OCR1B = 0;
  interrupts();
}

uint16_t adcSnapshot() {                 // 16-bit read under a stable ISR state
  uint16_t v;
  noInterrupts();
  v = adcAvg;
  interrupts();
  return v;
}

uint16_t dutySnapshot() {
  uint16_t v;
  noInterrupts();
  v = dutyNow;
  interrupts();
  return v;
}

// Wait until the feedback reading sits inside the settle window.
bool boostSettle() {
  uint32_t t0 = millis();
  uint8_t good = 0;
  while (millis() - t0 < SETTLE_MS) {
    int16_t err = (int16_t)adcSnapshot() - (int16_t)adcTarget;
    if (err >= -SETTLE_TOL && err <= SETTLE_TOL) {
      if (++good >= 8) return true;      // ~8 ms stable
    } else {
      good = 0;
    }
    delay(1);
  }
  return false;
}

void setTarget(uint16_t t) {
  noInterrupts();
  adcTarget = t;
  interrupts();
  EEPROM.update(EE_MAGIC_ADDR, EE_MAGIC);
  EEPROM.update(EE_TARGET_ADDR, t & 0xFF);
  EEPROM.update(EE_TARGET_ADDR + 1, t >> 8);
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
void vddOn()  { digitalWrite(VDD_PIN, VDD_ON); }
void vddOff() { digitalWrite(VDD_PIN, VDD_OFF); }

bool enterProgMode() {
  // RB6/RB7 must be held low while MCLR rises from VIL to VPP (DS39588A 2.3)
  digitalWrite(PGD_PIN, LOW);
  digitalWrite(PGC_PIN, LOW);
  vppOff();
  vddOff();
  progMode = false;
  delay(MCLR_MS);              // MCLR discharges to VIL through R7 10k

  if (!boostOn) boostStart();  // boost first: the VPP rail must be settled
  if (!boostSettle()) {        // before it is switched onto MCLR
    boostStop();
    vddOff();
    return false;
  }

  vddOn();
  delay(10);                   // VDD stable before VPP rises
  vppOn();
  delay(10);                   // settle in programming mode; PC = 0x0000
  noInterrupts();              // start the extremes window with VPP applied
  adcMin = 1023;
  adcMax = 0;
  dutyPeak = 0;
  progMode = true;
  interrupts();
  return true;
}

void exitProgMode() {
  vppOff();                    // MCLR below VIL: device leaves programming mode
  delay(5);
  vddOff();
  boostStop();
  digitalWrite(PGD_PIN, LOW);
  digitalWrite(PGC_PIN, LOW);
  progMode = false;
  delay(5);
}

bool ensureProgMode() {
  if (progMode) return true;
  return enterProgMode();
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

uint16_t readDeviceId() {      // programming mode must already be active
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
  static uint32_t lastCmd = 0;

  if (progMode && millis() - lastCmd > IDLE_TIMEOUT_MS) {
    exitProgMode();
  }
  if (boostOffAt && !progMode && (int32_t)(millis() - boostOffAt) >= 0) {
    boostStop();
    boostOffAt = 0;
  }
  if (!Serial.available()) return;
  lastCmd = millis();

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
      Serial.println(enterProgMode() ? F("OK") : F("EV"));
      break;

    case 'X':
      exitProgMode();
      Serial.println(F("OK"));
      break;

    case 'I': {
      if (!ensureProgMode()) { Serial.println(F("EV")); break; }
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

    case 'S':
      Serial.print('D');
      printHex4(adcSnapshot());
      Serial.print(' ');
      printHex4(dutySnapshot());
      Serial.println();
      break;

    case 'T': {                     // VPP extremes during programming mode
      uint16_t mn, mx, dp;
      noInterrupts();
      mn = adcMin; mx = adcMax; dp = dutyPeak;
      interrupts();
      Serial.print('D');
      printHex4(mn);
      Serial.print(' ');
      printHex4(mx);
      Serial.print(' ');
      printHex4(dp);
      Serial.println();
      break;
    }

    case 'Q': {                     // worst-case VPP load test
      if (!isHexWord(buf + 1)) { Serial.println(F("ER")); break; }
      uint16_t n = (uint16_t)strtoul(buf + 1, NULL, 16);
      if (n == 0 || n > 0x0400) { Serial.println(F("ER")); break; }
      if (!ensureProgMode()) { Serial.println(F("EV")); break; }
      noInterrupts();
      adcMin = 1023; adcMax = 0; dutyPeak = 0;
      interrupts();
      uint32_t t0 = millis();
      for (uint16_t i = 0; i < n; i++) {
        programWord(0x0000);        // all 14 cells charging: worst-case current
        delay(10);                  // matches the real programming cadence
      }
      uint32_t ms = millis() - t0;
      noInterrupts();
      uint16_t mn = adcMin, mx = adcMax, dp = dutyPeak;
      interrupts();
      Serial.print('D');
      printHex4(mn);
      Serial.print(' ');
      printHex4(mx);
      Serial.print(' ');
      printHex4(dp);
      Serial.print(' ');
      printHex4((uint16_t)(ms > 0xFFFF ? 0xFFFF : ms));
      Serial.println();
      break;
    }

    case 'B': {                     // boost self-test: no VDD, no VPP applied
      bool wasOn = boostOn;
      if (!wasOn) boostStart();
      uint32_t t0 = millis();
      uint16_t mn = 1023, mx = 0;
      while (millis() - t0 < 1200) {
        if (millis() - t0 >= 800) {       // ignore the ramp, watch for oscillation
          uint16_t a = adcSnapshot();
          if (a < mn) mn = a;
          if (a > mx) mx = a;
        }
        delay(1);
      }
      uint16_t a = adcSnapshot();
      uint16_t d = dutySnapshot();
      if (!wasOn) boostStop();
      Serial.print('D');
      printHex4(a);
      Serial.print(' ');
      printHex4(d);
      Serial.print(' ');
      printHex4(mn);
      Serial.print(' ');
      printHex4(mx);
      Serial.println();
      // Keep switching for 10 s so the rail can be measured with a meter.
      if (!progMode) boostOffAt = millis() + 10000;
      break;
    }

    case 'V': {
      if (buf[1] == '\0') {                 // report the A0 feedback reading
        if (!ensureProgMode()) { Serial.println(F("EV")); break; }
        Serial.print('D');
        printHex4(adcSnapshot());
        Serial.println();
        break;
      }
      if (!isHexWord(buf + 1)) { Serial.println(F("ER")); break; }
      uint16_t t = (uint16_t)strtoul(buf + 1, NULL, 16);
      if (t < 300 || t > 1023) { Serial.println(F("ER")); break; }
      setTarget(t);
      Serial.println(F("OK"));
      break;
    }

    case 'W':
      Serial.print('D');
      printHex4(adcTarget);
      Serial.println();
      break;

    default:
      Serial.println(F("ER"));
  }
}
