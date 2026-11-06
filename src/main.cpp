// Quanta Test - checks every part of the "Quanta 4 Channel Driver_BK FIXED4" board (V1.0).
//
// What it tests:
//   I2C pull-ups, both I2C buses, DAC A (U9) and DAC B (U8), the monitor ADC (U5), all 8 outputs
//   (OUT1..OUT8 = jack 1 tip, jack 1 ring, ... jack 4 ring) at 0/2/5/8/10 V with the load current
//   of each LED, the 15 V supply, the battery clock (U16) and its CR2032, the status LED (U11),
//   the SW1 and BOOT buttons, Wi-Fi and internet time.
//
// How to use: upload, open the Serial Monitor (115200), and read the results. Type a letter + Enter
// for more tests (h = menu). Without a computer: after the start-up tests the status LED shows
// green = all passed, amber = something to look at, blinking red = a failure. Each short press of
// SW1 then lights the next output at full brightness (OUT1, OUT2 ... all on, off).
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include "config.h"
#include "core.h"
#include "hal.h"

#define TEST_VERSION "QT-1.0"

// ----------------------------------------------------------------------------- results
enum Res : uint8_t { R_NOTRUN = 0, R_PASS, R_FAIL, R_CHECK };
enum {
  T_PULL, T_BUS1, T_BUS2, T_DACA, T_DACB, T_ADC,
  T_OUT1, T_OUT2, T_OUT3, T_OUT4, T_OUT5, T_OUT6, T_OUT7, T_OUT8,
  T_SUPPLY, T_RTC, T_BATT, T_LED, T_SW1, T_BOOT, T_WIFI, T_NTP, T_N
};
static const char *tName[T_N] = {
  "I2C pull-up resistors", "Bus 1 chips (IO21/IO22)", "Bus 2 chip (IO16/IO17)",
  "DAC A  U9  (OUT1-4)", "DAC B  U8  (OUT5-8)", "Monitor ADC  U5",
  "OUT1  jack 1 tip", "OUT2  jack 1 ring", "OUT3  jack 2 tip", "OUT4  jack 2 ring",
  "OUT5  jack 3 tip", "OUT6  jack 3 ring", "OUT7  jack 4 tip", "OUT8  jack 4 ring",
  "15 V supply / op-amps", "Battery clock  U16", "Clock battery  CR2032", "Status LED  U11",
  "Button SW1  (IO27)", "BOOT button  (IO0)", "Wi-Fi", "Internet time"
};
static Res  res[T_N];
static char note[T_N][120];

static const char *resTxt(Res r) {
  switch (r) { case R_PASS: return "PASS "; case R_FAIL: return "FAIL "; case R_CHECK: return "CHECK"; default: return "  -  "; }
}
static void setR(int i, Res r, const char *fmt, ...) {
  res[i] = r;
  va_list ap; va_start(ap, fmt); vsnprintf(note[i], sizeof(note[i]), fmt, ap); va_end(ap);
  Serial.printf("  [%s] %-26s %s\n", resTxt(r), tName[i], note[i]);
}

// ----------------------------------------------------------------------------- hardware
static hal::Mcp4728   dacA(ADDR_DAC_A), dacB(ADDR_DAC_B, &Wire1);
static hal::MonitorAdc adc;
static hal::Ds3231    rtc;
static const uint8_t  dacMap[8] = DAC_MAP;
static uint16_t       code[8];                  // DAC code per output OUT1..OUT8
static bool           bootOsf = false;          // clock had stopped when this test started

static void pixel(uint8_t r, uint8_t g, uint8_t b) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  rgbLedWrite(PIN_STATUS_LED, r, g, b);
#else
  neopixelWrite(PIN_STATUS_LED, r, g, b);
#endif
}

static bool dacOkFor(uint8_t c) { return c < 4 ? dacA.ok : dacB.ok; }

static void writeOutputs() {
  uint16_t a[4] = {0, 0, 0, 0}, b[4] = {0, 0, 0, 0};
  for (uint8_t c = 0; c < 8; c++) {
    uint8_t k = dacMap[c] & 7;
    if (k < 4) a[k] = code[c]; else b[k - 4] = code[c];
  }
  if (dacA.ok) dacA.write(a);
  if (dacB.ok) dacB.write(b);
}
static void allOff() { memset(code, 0, sizeof(code)); writeOutputs(); }
static uint16_t mvToCode(uint32_t mv) {
  float c = mv * (float)qc::DAC_MAX / (qc::OUT_FS_V * 1000.0f) + 0.5f;
  return c > qc::DAC_MAX ? qc::DAC_MAX : (uint16_t)c;
}

// ----------------------------------------------------------------------------- serial input
static String lineBuf;
static bool pollLine(String &out) {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { out = lineBuf; lineBuf = ""; out.trim(); return true; }
    if (lineBuf.length() < 100) lineBuf += ch;
  }
  return false;
}
// Waits for a typed answer. Returns "" if nobody answered in time.
static String ask(const char *prompt, uint32_t ms) {
  Serial.print(prompt);
  uint32_t t0 = millis(); String s;
  while (millis() - t0 < ms) { if (pollLine(s)) { Serial.println(); return s.length() ? s : String(" "); } delay(5); }
  Serial.println("  (no answer)");
  return String("");
}
static bool yes(const String &s) { return s.length() && (s[0] == 'y' || s[0] == 'Y'); }

static void fmtUtc(uint32_t e, char *out, size_t n) {
  time_t t = e; struct tm u; gmtime_r(&t, &u);
  strftime(out, n, "%Y-%m-%d %H:%M:%S UTC", &u);
}

// ============================================================================= tests
// 1. Pull-ups: with the ESP32's own pull-ups off, all four I2C lines must sit high on the board's 10k resistors.
static void testPullups() {
  Wire.end(); Wire1.end();
  const uint8_t pins[4] = {PIN_SDA, PIN_SCL, PIN_SDA2, PIN_SCL2};
  const char *nm[4] = {"SDA1(IO21) ", "SCL1(IO22) ", "SDA2(IO16) ", "SCL2(IO17) "};
  String bad;
  for (int i = 0; i < 4; i++) { pinMode(pins[i], INPUT); delay(3); if (digitalRead(pins[i]) != HIGH) bad += nm[i]; }
  if (bad.length()) setR(T_PULL, R_FAIL, "held low: %s- missing 10k pull-up or a short", bad.c_str());
  else setR(T_PULL, R_PASS, "all four lines sit high on the 10k resistors");
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  Wire1.begin(PIN_SDA2, PIN_SCL2, I2C_HZ);
}

// 2. Buses: bus 1 must have the ADC (0x48), a DAC (0x60-0x67) and the clock (0x68); bus 2 a DAC.
static uint8_t dacAddr1 = 0, dacAddr2 = 0;
static void testBuses() {
  String l1, l2, miss, extra; bool a48 = false, a68 = false; dacAddr1 = dacAddr2 = 0; char b[8];
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      snprintf(b, sizeof(b), "0x%02X ", a); l1 += b;
      if (a == ADDR_ADC) a48 = true;
      else if (a == ADDR_RTC) a68 = true;
      else if (a >= 0x60 && a <= 0x67 && !dacAddr1) dacAddr1 = a;
      else extra += b;
    }
  }
  for (uint8_t a = 1; a < 127; a++) {
    Wire1.beginTransmission(a);
    if (Wire1.endTransmission() == 0) {
      snprintf(b, sizeof(b), "0x%02X ", a); l2 += b;
      if (a >= 0x60 && a <= 0x67 && !dacAddr2) dacAddr2 = a;
    }
  }
  Serial.printf("  bus 1 answers: %s\n  bus 2 answers: %s\n", l1.length() ? l1.c_str() : "nothing", l2.length() ? l2.c_str() : "nothing");
  if (!a48) miss += "ADC U5 (0x48) ";
  if (!dacAddr1) miss += "DAC U9 (0x60-67) ";
  if (!a68) miss += "clock U16 (0x68) ";
  if (miss.length()) setR(T_BUS1, R_FAIL, "missing: %s", miss.c_str());
  else if (extra.length()) setR(T_BUS1, R_CHECK, "all 3 chips found, plus unexpected: %s", extra.c_str());
  else setR(T_BUS1, R_PASS, "ADC 0x48, DAC 0x%02X, clock 0x68", dacAddr1);
  if (dacAddr2) setR(T_BUS2, R_PASS, "DAC at 0x%02X", dacAddr2);
  else setR(T_BUS2, R_FAIL, "DAC U8 not answering (check IO16/IO17, its pull-ups, U8 power)");
}

// 3. DACs: write a pattern, read it back, set to 0.
static void testDac(hal::Mcp4728 &d, uint8_t addr, TwoWire *bus, int ti, const char *chip) {
  if (!addr) { d.ok = false; setR(ti, R_FAIL, "%s not found on its bus", chip); return; }
  d = hal::Mcp4728(addr, bus);
  if (!d.begin()) { setR(ti, R_FAIL, "%s at 0x%02X stopped answering", chip, addr); return; }
  const uint16_t pat[4] = {0x123, 0x456, 0x789, 0xABC}, zero[4] = {0, 0, 0, 0};
  uint16_t cur[4] = {0}, eep[4] = {0}; bool ir[4] = {false};
  bool ok = d.write(pat) && d.read(cur, eep, ir);
  for (int c = 0; c < 4; c++) if (cur[c] != pat[c]) ok = false;
  d.write(zero);
  if (!ok) { setR(ti, R_FAIL, "wrote 123 456 789 ABC, read back %03X %03X %03X %03X", cur[0], cur[1], cur[2], cur[3]); d.ok = false; return; }
  bool pz = true; for (int c = 0; c < 4; c++) if (eep[c] != 0 || !ir[c]) pz = false;
  setR(ti, R_PASS, "0x%02X: write + read-back OK%s", addr, pz ? ", powers up at 0 V" : " (power-up value not 0 V yet; the Quanta firmware sets it once)");
}

// 4. Monitor ADC: with all outputs at 0 V, all 8 channels should read near 0.
static void testAdc() {
  allOff(); delay(150);
  if (!adc.begin()) { setR(T_ADC, R_FAIL, "ADS7828 not answering at 0x48"); return; }
  int mx = 0, mxc = 0; bool rd = true;
  for (uint8_t j = 0; j < 8; j++) { int v = adc.readAmpMv(j); if (v < 0) rd = false; if (v > mx) { mx = v; mxc = j; } }
  if (!rd) setR(T_ADC, R_FAIL, "found, but readings fail");
  else if (mx > 400) setR(T_ADC, R_CHECK, "all outputs at 0 V, but Monitor%d reads %.2f V", mxc + 1, mx / 1000.0f);
  else setR(T_ADC, R_PASS, "8 channels read; all near 0 V with outputs off (max %d mV)", mx);
}

// 5. Outputs. The monitor reads the op-amp side of the 1k series resistor, so
//    load current (mA) = (op-amp volts - jack volts) / 1k. An LED pulls current, so the op-amp side reads higher.
static const uint16_t LV[5] = {0, 2000, 5000, 8000, 10000};
static int ampAt10[8];
static void testOutput(uint8_t c) {
  int ti = T_OUT1 + c;
  ampAt10[c] = -1;
  if (!dacOkFor(c)) { setR(ti, R_FAIL, "skipped: its DAC (%s) isn't working", c < 4 ? "U9" : "U8"); return; }
  Serial.printf("\n  OUT%u = jack %u %s\n    set V   op-amp V   load mA   other monitors that moved\n", c + 1, c / 2 + 1, c & 1 ? "ring" : "tip");
  int amp[5]; float ma[5]; int worst = 0, worstCh = -1;
  for (int k = 0; k < 5; k++) {
    memset(code, 0, sizeof(code)); code[c] = mvToCode(LV[k]); writeOutputs(); delay(150);
    int m[8]; for (uint8_t j = 0; j < 8; j++) m[j] = adc.ok ? adc.readAmpMv(j) : -1;
    amp[k] = m[c]; ma[k] = (m[c] - (int)LV[k]) / 1000.0f;
    String others; char b[24];
    for (uint8_t j = 0; j < 8; j++) if (j != c && m[j] > 300) {
      snprintf(b, sizeof(b), "OUT%u %.2fV ", j + 1, m[j] / 1000.0f); others += b;
      if (m[j] > worst) { worst = m[j]; worstCh = j; }
    }
    if (adc.ok) Serial.printf("    %5.2f    %6.2f    %6.2f    %s\n", LV[k] / 1000.0f, amp[k] / 1000.0f, ma[k], others.c_str());
    else Serial.printf("    %5.2f    (no monitor chip)\n", LV[k] / 1000.0f);
  }
  allOff();
  if (!adc.ok) { setR(ti, R_CHECK, "no monitor chip: check the LED by eye (type v)"); return; }
  ampAt10[c] = amp[4];
  if (amp[0] > 300) { setR(ti, R_FAIL, "reads %.2f V with the output at 0 V", amp[0] / 1000.0f); return; }
  if (worst > 1000 && amp[3] < 1000) {
    setR(ti, R_FAIL, "nothing on OUT%u's monitor, but OUT%d's moved: outputs swapped (DAC_MAP/MON_MAP in config.h)", c + 1, worstCh + 1); return;
  }
  if (worst > 1000) { setR(ti, R_FAIL, "OUT%d moved too (%.1f V): tip and ring shorted together?", worstCh + 1, worst / 1000.0f); return; }
  for (int k = 1; k < 4; k++) {                 // 2, 5, 8 V must be reached (LED loads only push the op-amp side higher)
    int need = (int)(LV[k] * 0.95f) - 150;
    if (amp[k] < need) { setR(ti, R_FAIL, "too low: set %.1f V, op-amp only %.2f V", LV[k] / 1000.0f, amp[k] / 1000.0f); return; }
  }
  bool sat = amp[4] > 13000 || amp[3] > 13000;
  if (sat) { setR(ti, R_CHECK, "op-amp at its limit (%.1f V): the LED pulls %.1f mA, so the jack can't reach 10 V. Fine for an LED test load.", amp[4] / 1000.0f, ma[4]); return; }
  if (ma[2] < 0.2f && ma[4] < 0.2f) { setR(ti, R_CHECK, "output works 0-10 V, but no load current: is the LED in and the plug pushed fully home?"); return; }
  setR(ti, R_PASS, "0-10 V OK; LED draws %.1f mA at 5 V, %.1f mA at 10 V", ma[2], ma[4]);
}

static void testAllOutputs() {
  for (uint8_t c = 0; c < 8; c++) testOutput(c);
  Serial.println();
  int best = -1; for (uint8_t c = 0; c < 8; c++) if (ampAt10[c] > best) best = ampAt10[c];
  if (best < 0) setR(T_SUPPLY, R_NOTRUN, "outputs couldn't be measured");
  else if (best < 4000) setR(T_SUPPLY, R_FAIL, "no output gets above %.1f V: is the 15 V adapter plugged in? (USB/programmer power alone can't drive the outputs)", best / 1000.0f);
  else setR(T_SUPPLY, R_PASS, "outputs reach %.1f V, so 15 V, 5 V and the op-amps work", best / 1000.0f);
}

// 6. Battery clock and its coin cell.
static void testRtc(bool atBoot) {
  if (!rtc.begin()) {
    setR(T_RTC, R_FAIL, "DS3231 not answering at 0x68");
    if (atBoot) setR(T_BATT, R_NOTRUN, "clock chip missing");
    return;
  }
  if (atBoot) bootOsf = rtc.lostPower;
  uint8_t ctl = 0xFF, s1 = 0, s2 = 0;
  hal::Ds3231::rd8(ADDR_RTC, 0x0E, ctl);
  hal::Ds3231::rd8(ADDR_RTC, 0x00, s1); delay(1100); hal::Ds3231::rd8(ADDR_RTC, 0x00, s2);
  bool ticking = s1 != s2;
  int16_t tq = rtc.tempQuarterC(); float tc = tq == INT16_MIN ? -99 : tq / 4.0f;
  uint32_t now = rtc.read(); char ts[32] = "not set";
  if (now) fmtUtc(now, ts, sizeof(ts));
  if (!ticking) setR(T_RTC, R_FAIL, "seconds are not counting (oscillator stopped)");
  else if (ctl != 0x1C) setR(T_RTC, R_FAIL, "control register reads 0x%02X, expected 0x1C", ctl);
  else if (tc < 0 || tc > 70) setR(T_RTC, R_CHECK, "ticking, but temperature reads %.1f C", tc);
  else if (!now) setR(T_RTC, R_CHECK, "ticking, %.1f C, time not set yet (type t to set it from the internet)", tc);
  else setR(T_RTC, R_PASS, "ticking, %.1f C, %s", tc, ts);

  if (!atBoot) return;
  Preferences q; bool wasSet = false;
  if (q.begin("qtest", true)) { wasSet = q.isKey("set") && q.getBool("set", false); q.end(); }
  esp_reset_reason_t rr = esp_reset_reason();
  bool cold = rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT;
  if (bootOsf && wasSet) setR(T_BATT, R_FAIL, "the clock stopped while unplugged: CR2032 flat, missing or in backwards, or BT1 not connected to VBAT");
  else if (bootOsf) setR(T_BATT, R_CHECK, "clock had never been set (new chip). Type t to set it, unplug ALL power 1 min, plug back in");
  else if (now && cold) setR(T_BATT, R_PASS, "the clock kept time while the board was unplugged");
  else if (now) setR(T_BATT, R_CHECK, "time is kept; to prove the battery, unplug ALL power for 1 min and plug back in");
  else setR(T_BATT, R_CHECK, "type t to set the time, then unplug ALL power 1 min and plug back in");
}

// 7. Wi-Fi and internet time (sets the battery clock).
static bool getCreds(String &s, String &p) {
  Preferences q;
  if (q.begin("qtest", true)) { if (q.isKey("ssid")) { s = q.getString("ssid", ""); p = q.getString("pass", ""); } q.end(); }
  if (!s.length() && q.begin("quanta", true)) {           // Wi-Fi saved by the Quanta firmware, if any
    if (q.isKey("ssid")) s = q.getString("ssid", "");
    if (q.isKey("pass")) p = q.getString("pass", "");
    q.end();
  }
  return s.length() > 0;
}
static void enterCreds() {
  String s = ask("  Wi-Fi name (SSID): ", 120000); s.trim();
  if (!s.length()) return;
  String p = ask("  Wi-Fi password (Enter for none): ", 120000); p.trim();
  Preferences q; q.begin("qtest", false); q.putString("ssid", s); q.putString("pass", p); q.end();
  Serial.printf("  saved \"%s\"\n", s.c_str());
}
static void testWifiTime() {
  String s, p;
  if (!getCreds(s, p)) {
    setR(T_WIFI, R_NOTRUN, "no Wi-Fi saved: type w to enter one");
    setR(T_NTP, R_NOTRUN, "needs Wi-Fi");
    return;
  }
  Serial.printf("  joining \"%s\" ...\n", s.c_str());
  WiFi.mode(WIFI_STA); WiFi.begin(s.c_str(), p.c_str());
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(200);
  if (WiFi.status() != WL_CONNECTED) {
    setR(T_WIFI, R_FAIL, "could not join \"%s\" (wrong password, or 5 GHz-only network?)", s.c_str());
    setR(T_NTP, R_NOTRUN, "needs Wi-Fi");
    return;
  }
  setR(T_WIFI, R_PASS, "joined \"%s\", signal %d dBm, address %s", s.c_str(), WiFi.RSSI(), WiFi.localIP().toString().c_str());
  configTime(0, 0, NTP_SERVER1, NTP_SERVER2, NTP_SERVER3);
  t0 = millis();
  while ((uint32_t)time(nullptr) < 1700000000UL && millis() - t0 < 20000) delay(200);
  uint32_t net = (uint32_t)time(nullptr);
  if (net < 1700000000UL) { setR(T_NTP, R_FAIL, "no answer from the time servers (internet down, or port 123 blocked)"); return; }
  char ts[32]; fmtUtc(net, ts, sizeof(ts));
  if (!rtc.ok) { setR(T_NTP, R_CHECK, "internet time %s, but no battery clock to set", ts); return; }
  uint32_t r = rtc.read();
  bool wrote = rtc.write(net);
  if (wrote) { Preferences q; q.begin("qtest", false); q.putBool("set", true); q.end(); }
  if (!wrote) setR(T_NTP, R_FAIL, "internet time %s, but writing the clock failed", ts);
  else if (r) setR(T_NTP, R_PASS, "%s; battery clock was off by %ld s, now set", ts, (long)((int64_t)r - (int64_t)net));
  else setR(T_NTP, R_PASS, "%s; battery clock had no time, now set", ts);
}

// 8. Status LED (needs someone to look).
static void testLed() {
  Serial.println("  Watch the status LED: red, green, blue, white (repeats twice)");
  for (int k = 0; k < 2; k++) {
    pixel(60, 0, 0); delay(700); pixel(0, 60, 0); delay(700); pixel(0, 0, 60); delay(700); pixel(50, 50, 50); delay(700); pixel(0, 0, 0); delay(300);
  }
  String a = ask("  Did you see red, green, blue and white in that order? (y/n): ", 30000);
  if (!a.length()) setR(T_LED, R_CHECK, "colours shown, nobody confirmed");
  else if (yes(a)) setR(T_LED, R_PASS, "red, green, blue, white seen");
  else setR(T_LED, R_FAIL, "wrong or no colours: check U11 5 V, IO26 to DIN, orientation");
}

// 9. Buttons (needs someone to press).
static void testButton(uint8_t pin, int ti, const char *name) {
  if (digitalRead(pin) == LOW) { setR(ti, R_FAIL, "reads pressed without being touched (stuck or shorted)"); return; }
  Serial.printf("  Press and release %s now (15 s)...\n", name);
  pixel(0, 0, 40);
  uint32_t t0 = millis(); bool down = false, up = false;
  while (millis() - t0 < 15000 && !up) {
    if (!down && digitalRead(pin) == LOW) { down = true; Serial.println("  pressed"); }
    if (down && digitalRead(pin) == HIGH) up = true;
    delay(10);
  }
  pixel(0, 0, 0);
  if (up) setR(ti, R_PASS, "press and release detected");
  else if (down) setR(ti, R_FAIL, "pressed, but never released");
  else setR(ti, R_CHECK, "no press detected in 15 s");
}

// ----------------------------------------------------------------------------- visual helpers
static void visualWalk() {
  Serial.println("  Each LED fades up and down in turn. Watch the test board.");
  for (uint8_t c = 0; c < 8; c++) {
    if (!dacOkFor(c)) { Serial.printf("  OUT%u skipped (DAC missing)\n", c + 1); continue; }
    Serial.printf("  OUT%u (jack %u %s)\n", c + 1, c / 2 + 1, c & 1 ? "ring" : "tip");
    for (int s = 0; s <= 50; s++) { memset(code, 0, sizeof(code)); code[c] = mvToCode(s * 200); writeOutputs(); delay(20); }
    delay(500);
    for (int s = 50; s >= 0; s--) { code[c] = mvToCode(s * 200); writeOutputs(); delay(20); }
  }
  Serial.println("  All 8 together");
  for (int s = 0; s <= 50; s++) { for (uint8_t c = 0; c < 8; c++) code[c] = mvToCode(s * 200); writeOutputs(); delay(20); }
  delay(1500);
  allOff();
}

static void setCmd(const String &line) {     // "p 3 50", "p all 100", "p off"
  String a = line.substring(1); a.trim();
  if (a.startsWith("off") || !a.length()) { allOff(); Serial.println("  all outputs off"); return; }
  int sp = a.indexOf(' ');
  String w = sp > 0 ? a.substring(0, sp) : a; int pct = sp > 0 ? a.substring(sp + 1).toInt() : 100;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  uint16_t cd = mvToCode(pct * 100);
  if (w == "all") { for (uint8_t c = 0; c < 8; c++) code[c] = cd; }
  else { int n = w.toInt(); if (n < 1 || n > 8) { Serial.println("  use: p 3 50   p all 100   p off"); return; } code[n - 1] = cd; }
  writeOutputs();
  delay(150);
  for (uint8_t c = 0; c < 8; c++) {
    int m = adc.ok ? adc.readAmpMv(c) : -1;
    uint32_t setmv = (uint32_t)(code[c] * qc::OUT_FS_V * 1000.0f / qc::DAC_MAX + 0.5f);
    Serial.printf("  OUT%u  set %5.2f V   op-amp %6.2f V   load %5.2f mA\n", c + 1, setmv / 1000.0f, m / 1000.0f, m < 0 ? 0 : (m - (int)setmv) / 1000.0f);
  }
}

// ----------------------------------------------------------------------------- summary + menu
static void summary() {
  int p = 0, f = 0, k = 0;
  Serial.println("\n================ QUANTA TEST RESULTS ================");
  for (int i = 0; i < T_N; i++) {
    Serial.printf("  [%s] %-26s %s\n", resTxt(res[i]), tName[i], note[i]);
    if (res[i] == R_PASS) p++; else if (res[i] == R_FAIL) f++; else if (res[i] == R_CHECK) k++;
  }
  Serial.printf("-----------------------------------------------------\n  %d passed, %d failed, %d to check, %d not run\n", p, f, k, T_N - p - f - k);
  Serial.println(f ? "  >>> BOARD HAS FAULTS <<<" : k ? "  Board OK so far; look at the CHECK lines" : "  >>> ALL TESTS PASSED <<<");
  Serial.println("=====================================================");
}
static void menu() {
  Serial.println(
    "\nType a letter and press Enter:\n"
    "  a  run EVERY test (you'll be asked to look at the LED and press the buttons)\n"
    "  q  quick: automatic tests only\n"
    "  s  I2C pull-ups, bus scan, DACs, ADC\n"
    "  o  output test with numbers (all 8)\n"
    "  v  visual: each LED fades up and down in turn\n"
    "  p  set outputs by hand:  p 3 50  (OUT3 at 50%)   p all 100   p off\n"
    "  c  battery clock test\n"
    "  t  internet time (also sets the battery clock)\n"
    "  w  enter Wi-Fi name and password\n"
    "  l  status LED colours\n"
    "  b  buttons SW1 and BOOT\n"
    "  r  show the results again\n"
    "  h  this menu");
}
static void runHardware() {
  Serial.println("\n--- I2C ---"); testPullups(); testBuses();
  Serial.println("\n--- DACs ---");
  testDac(dacA, dacAddr1, &Wire, T_DACA, "DAC U9");
  testDac(dacB, dacAddr2, &Wire1, T_DACB, "DAC U8");
  Serial.println("\n--- monitor ADC ---"); testAdc();
}
static void runAuto(bool atBoot) {
  runHardware();
  Serial.println("\n--- outputs ---"); testAllOutputs();
  Serial.println("\n--- battery clock ---"); testRtc(atBoot);
  Serial.println("\n--- Wi-Fi and internet time ---"); testWifiTime();
}

// ----------------------------------------------------------------------------- result LED + stand-alone walk
static void resultLed() {
  bool f = false, k = false;
  for (int i = 0; i < T_N; i++) { if (res[i] == R_FAIL) f = true; if (res[i] == R_CHECK) k = true; }
  if (f) pixel((millis() / 400) & 1 ? 60 : 0, 0, 0);
  else if (k) pixel(50, 25, 0);
  else pixel(0, 50, 0);
}
static int walkStep = -1;                    // -1 = off, 0..7 = one output, 8 = all on
static void walkNext() {
  walkStep = walkStep >= 8 ? -1 : walkStep + 1;
  memset(code, 0, sizeof(code));
  if (walkStep >= 0 && walkStep < 8) { code[walkStep] = qc::DAC_MAX; Serial.printf("SW1: OUT%d on (jack %d %s)\n", walkStep + 1, walkStep / 2 + 1, walkStep & 1 ? "ring" : "tip"); }
  else if (walkStep == 8) { for (uint8_t c = 0; c < 8; c++) code[c] = qc::DAC_MAX; Serial.println("SW1: all outputs on"); }
  else Serial.println("SW1: all outputs off");
  writeOutputs();
}

// ============================================================================= setup / loop
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  pixel(0, 0, 0);
  delay(1500);                               // time to open the serial monitor
  Serial.printf("\n\n=====================================================\n"
                "  QUANTA TEST %s  -  Quanta 4 Channel Driver V1.0\n"
                "  ESP32 %s rev %d, %d MHz, flash %u MB, chip id %012llX\n"
                "  start reason: %d (1 = power on)\n"
                "=====================================================\n",
                TEST_VERSION, ESP.getChipModel(), ESP.getChipRevision(), ESP.getCpuFreqMHz(),
                (unsigned)(ESP.getFlashChipSize() / (1024 * 1024)), (unsigned long long)ESP.getEfuseMac(), (int)esp_reset_reason());
  runAuto(true);
  summary();
  Serial.println("\nThe LED and button tests need you: type a (everything) or l / b.");
  menu();
}

void loop() {
  String line;
  if (pollLine(line) && line.length()) {
    walkStep = -1;
    char c = tolower(line[0]);
    switch (c) {
      case 'a': runAuto(false); Serial.println("\n--- status LED ---"); testLed();
                Serial.println("\n--- buttons ---"); testButton(PIN_BUTTON, T_SW1, "SW1"); testButton(0, T_BOOT, "BOOT");
                summary(); break;
      case 'q': runAuto(false); summary(); break;
      case 's': runHardware(); break;
      case 'o': testAllOutputs(); break;
      case 'v': visualWalk(); break;
      case 'p': setCmd(line); break;
      case 'c': testRtc(false); break;
      case 't': testWifiTime(); break;
      case 'w': enterCreds(); testWifiTime(); break;
      case 'l': testLed(); break;
      case 'b': testButton(PIN_BUTTON, T_SW1, "SW1"); testButton(0, T_BOOT, "BOOT"); break;
      case 'r': summary(); break;
      default: menu(); break;
    }
    Serial.print("\n> ");
  }
  // stand-alone: each short press of SW1 lights the next output
  static uint32_t downAt = 0;
  bool down = digitalRead(PIN_BUTTON) == LOW;
  if (down && !downAt) downAt = millis();
  if (!down && downAt) { if (millis() - downAt > 30) walkNext(); downAt = 0; }
  static uint32_t lastLed = 0;
  if (millis() - lastLed > 50) { lastLed = millis(); resultLed(); }
  delay(5);
}
