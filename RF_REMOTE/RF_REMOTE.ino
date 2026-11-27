#include "driver/gpio.h"
#include "esp32-hal-rmt.h"
#include <Arduino.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include <Preferences.h>
#include <RCSwitch.h>
#include <SPI.h>
#include <U8g2lib.h>
#include <WebServer.h>
#include <WiFi.h>

// Donanım Pinleri (ESP32-C3 0.42" OLED + 433M V2.0 CC1101)
#define OLED_SDA 5
#define OLED_SCL 6
#define LED_PIN 8     // Dahili mavi LED (Active LOW)
#define BOOT_BUTTON 9 // Dahili BOOT butonu (Active LOW, INPUT_PULLUP)

#define CC1101_SCK 4
#define CC1101_MISO 3
#define CC1101_MOSI 7
#define CC1101_CSN 10
#define CC1101_GDO0 1

#define MAX_REMOTES 20
#define NVS_SCHEMA_VERSION 2
#define MAX_RAW_PULSES 140
#define MAX_ISR_PULSES 360
#define FRAME_QUEUE_LEN 4
#define MAX_WEB_LOGS 25
#define LISTEN_TIMEOUT_MS 60000UL

#define AP_SSID "CanXer_Telemetry"
#define AP_PASS "LecInterval66%"

struct RemoteItem {
  String name;
  uint32_t code;
  uint16_t bits;
  uint16_t pulse;
  uint8_t proto;
  uint16_t rawCount;
  uint16_t syncLow;
  int16_t lastRssi;
  uint16_t rawPulses[MAX_RAW_PULSES];
};

struct DecodedOok {
  bool valid;
  uint32_t code;
  uint16_t bits;
  uint16_t pulse;
  uint8_t proto; // 1: Trailing-Sync (PT2262/EV1527/Weko), 13: Leading-Sync
                 // (Came/Nice)
  uint16_t offset;
  uint16_t edgeCount;
  float score;
};

struct FrameSlot {
  uint16_t pulses[MAX_ISR_PULSES];
  uint16_t count;
  uint16_t syncLow;
};

struct OokAlign {
  uint32_t code;
  uint16_t bits;
};

U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
RCSwitch mySwitch;
WebServer server(80);
Preferences prefs;

RemoteItem remotes[MAX_REMOTES];
int remoteCount = 0;
int selectedIndex = 0;

bool cc1101Ok = false;
uint8_t cc1101Part = 0xFF;
uint8_t cc1101Ver = 0xFF;
uint8_t cc1101Agc2 = 0xFF;
uint8_t cc1101Marc = 0xFF;
float currentFreqMhz = 433.92f;

int currentRssi = -120;
int peakRssi = -120;
int displayPeakRssi = -120;
unsigned long displayPeakUntil = 0;
float noiseFloor = -95.0f;
bool noiseFloorInit = false;

// ISR 4-slot halka tamponu + gercek zamanli parazit dikici
volatile uint16_t isrPulses[MAX_ISR_PULSES];
volatile uint16_t isrCount = 0;
volatile uint32_t isrLastUs = 0;
volatile uint8_t isrLastLevel = 0;
volatile uint32_t isrEdgeTotal = 0;
volatile uint16_t isrMaxCountSeen = 0;
volatile uint32_t isrGlitchCount = 0;
volatile uint16_t isrMinDtSeen = 65535;
volatile uint16_t isrMaxDtSeen = 0;

volatile FrameSlot frameQueue[FRAME_QUEUE_LEN];
volatile uint8_t qHead = 0;
volatile uint8_t qTail = 0;

// Web arayuzu icin hafizada olay loglari (AJAX yok -> sifir ek isinma / sifir
// RF parazit)
String webLogs[MAX_WEB_LOGS];
int webLogCount = 0;

// Dinleme modu durumu
bool listenMode = false;
unsigned long listenModeUntil = 0;

// 2-cerceve dogrulama ve en temiz cerceve secim degiskenleri
uint32_t pendingCode = 0;
uint16_t pendingBits = 0;
uint16_t pendingPulse = 0;
uint8_t pendingProto = 1;
uint16_t pendingOffset = 0;
uint16_t pendingEdgeCount = 0;
uint16_t pendingRawCount = 0;
uint16_t pendingSyncLow = 0;
float pendingScore = 999.0f;
uint16_t pendingPulses[MAX_ISR_PULSES];
unsigned long pendingTime = 0;
bool nvsDumpedToSerial = false;
bool lastTxGdo0Ok = false;
bool lastTxRmtOk = false;
bool rmtTxActive = false;

uint16_t lastUnmatchedCount = 0;
uint16_t lastUnmatchedSync = 0;
unsigned long lastUnmatchedTime = 0;
unsigned long lastRawSavedTime = 0;

// Zamanlayıcı ve durum değişkenleri
unsigned long lastRssiRead = 0;
unsigned long lastOledDraw = 0;
unsigned long lastSerialLog = 0;
unsigned long lastRawSignalTime = 0;
unsigned long lastDecodedTime = 0;
unsigned long lastBurstTime = 0;
uint32_t lastBurstCode = 0;
unsigned long ledOffTime = 0;
unsigned long oledBusySkipUntil = 0;

String overlayLine1 = "";
String overlayLine2 = "";
unsigned long overlayUntil = 0;

bool lastBtnReading = HIGH;
bool stableBtnState = HIGH;
unsigned long lastBtnDebounce = 0;
unsigned long btnPressStartMs = 0;
bool btnLongHandled = false;

void addLog(const String &msg) {
  unsigned long sec = millis() / 1000UL;
  char ts[16];
  snprintf(ts, sizeof(ts), "[%02lu:%02lu] ", sec / 60UL, sec % 60UL);
  String entry = String(ts) + msg;

  Serial.println(entry);

  if (webLogCount < MAX_WEB_LOGS) {
    webLogs[webLogCount++] = entry;
  } else {
    for (int i = 0; i < MAX_WEB_LOGS - 1; i++) {
      webLogs[i] = webLogs[i + 1];
    }
    webLogs[MAX_WEB_LOGS - 1] = entry;
  }
}

volatile uint32_t lastRssiSampleMs = 0;

bool isIdleOscillation(const uint16_t *pulses, uint16_t count,
                       bool strictPure) {
  if (count < 16)
    return false;
  uint16_t totalPairs = count / 2;
  uint16_t oscPairs = 0;
  for (uint16_t i = 0; i + 1 < count; i += 2) {
    uint16_t hi = pulses[i];
    uint16_t lo = pulses[i + 1];
    if (hi >= 750 && hi <= 1050 && lo >= 60 && lo <= 160) {
      oscPairs++;
    }
  }
  if (!strictPure) {
    return (oscPairs * 10 >= totalPairs * 6);
  }
  if (oscPairs * 10 < totalPairs * 8) {
    return false;
  }
  // Kuyruk sonunda gercek uzaktan kumanda darbesi varsa koru (son 24 kenari
  // kontrol et)
  uint16_t tailStart = (count > 24) ? (uint16_t)((count - 24) & ~1U) : 0;
  uint16_t tailPairs = (count - tailStart) / 2;
  uint16_t tailOsc = 0;
  for (uint16_t i = tailStart; i + 1 < count; i += 2) {
    uint16_t hi = pulses[i];
    uint16_t lo = pulses[i + 1];
    if (hi >= 750 && hi <= 1050 && lo >= 60 && lo <= 160) {
      tailOsc++;
    }
  }
  return (tailPairs > 0) && (tailOsc * 10 >= tailPairs * 7);
}

void IRAM_ATTR pushFrameFromIsr(uint16_t count, uint16_t syncLow) {
  if (count < 20)
    return;
  if (count > MAX_ISR_PULSES)
    count = MAX_ISR_PULSES;
  uint8_t nextHead = (uint8_t)((qHead + 1) % FRAME_QUEUE_LEN);
  if (nextHead == qTail) {
    // Kuyruk doluysa en eski paketi dusur, yeni sinyali koru
    qTail = (uint8_t)((qTail + 1) % FRAME_QUEUE_LEN);
  }
  for (uint16_t i = 0; i < count; i++) {
    frameQueue[qHead].pulses[i] = isrPulses[i];
  }
  frameQueue[qHead].count = count;
  frameQueue[qHead].syncLow = syncLow;
  qHead = nextHead;
}

void IRAM_ATTR rfIsr() {
  uint32_t nowUs = micros();
  uint32_t dt = nowUs - isrLastUs;
  uint8_t level = (uint8_t)gpio_get_level((gpio_num_t)CC1101_GDO0);

  // Ayni seviyede mikrosaniyelik cok kisa sicramayi yut
  if (isrCount > 0 && level == isrLastLevel) {
    return;
  }

  isrLastUs = nowUs;
  isrLastLevel = level;
  isrEdgeTotal++;
  if (isrCount > isrMaxCountSeen)
    isrMaxCountSeen = isrCount;
  if (dt > 0 && dt < 65000UL) {
    if ((uint16_t)dt < isrMinDtSeen)
      isrMinDtSeen = (uint16_t)dt;
    if ((uint16_t)dt > isrMaxDtSeen)
      isrMaxDtSeen = (uint16_t)dt;
  }

  // Henuz aktif cerceve baslamadiysa: gecerli ilk HIGH darbesinin bitisinde
  // (level==0) baslat
  if (isrCount == 0) {
    if (level == 0 && dt >= 55 && dt <= 5000) {
      isrPulses[0] = (uint16_t)dt;
      isrCount = 1;
      if (isrCount > isrMaxCountSeen)
        isrMaxCountSeen = isrCount;
    }
    return;
  }

  // Normal Sync LOW boslugu (1500..60000 us) bitip yukselen kenar geldiginde
  // cerceveyi teslim et
  if (level == 1 && dt >= 1500 && dt <= 60000) {
    if (isrCount >= 20) {
      pushFrameFromIsr(isrCount, (uint16_t)dt);
    }
    isrCount = 0;
    return;
  }

  // Gecerli OOK darbe suresi (55..5000 us)
  if (dt >= 55 && dt <= 5000) {
    if (isrCount < MAX_ISR_PULSES) {
      isrPulses[isrCount++] = (uint16_t)dt;
      if (isrCount > isrMaxCountSeen)
        isrMaxCountSeen = isrCount;
    } else {
      pushFrameFromIsr(isrCount, 0);
      isrCount = 0;
    }
    return;
  }

  // Sinir disi darbe (<55 veya >5000 us): reset oncesi birikmis >=24 kenari
  // kurtar
  if (dt < 55) {
    isrGlitchCount++;
    if (isrCount >= 24) {
      pushFrameFromIsr(isrCount, 0);
    }
  } else {
    if (isrCount >= 24) {
      uint16_t sGap =
          (level == 1) ? ((dt > 65000UL) ? 65000U : (uint16_t)dt) : 0;
      pushFrameFromIsr(isrCount, sGap);
    }
  }
  isrCount = 0;
  if (level == 0 && dt >= 55 && dt <= 5000) {
    isrPulses[0] = (uint16_t)dt;
    isrCount = 1;
  }
}

void attachRfIsr() {
  pinMode(CC1101_GDO0, INPUT);
  isrCount = 0;
  qHead = 0;
  qTail = 0;
  isrGlitchCount = 0;
  isrMinDtSeen = 65535;
  isrMaxDtSeen = 0;
  isrLastUs = micros();
  isrLastLevel = (uint8_t)gpio_get_level((gpio_num_t)CC1101_GDO0);
  attachInterrupt(CC1101_GDO0, rfIsr, CHANGE);
}

void setLed(bool on) { digitalWrite(LED_PIN, on ? LOW : HIGH); }

void triggerLedBlink(unsigned long durationMs) {
  setLed(true);
  ledOffTime = millis() + durationMs;
}

void setStatusOverlay(const String &line1, const String &line2,
                      unsigned long durationMs) {
  overlayLine1 = line1;
  overlayLine2 = line2;
  overlayUntil = millis() + durationMs;
}

void delayUsSafe(uint32_t us) {
  if (us > 10000) {
    delay(us / 1000);
    us %= 1000;
  }
  if (us > 0) {
    delayMicroseconds(us);
  }
}

bool waitMarcState(uint8_t target, uint16_t timeoutUs) {
  uint32_t startUs = micros();
  while ((uint32_t)(micros() - startUs) < timeoutUs) {
    uint8_t st = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
    cc1101Marc = st;
    if (st == target)
      return true;
    delayMicroseconds(40);
  }
  cc1101Marc = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
  return (cc1101Marc == target);
}

bool enterRxMode() {
  gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  pinMode(CC1101_GDO0, INPUT);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 3000);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SFRX);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
  if (waitMarcState(0x0D, 4000)) {
    return true;
  }
  // Ilk deneme basarisizsa manuel kalibrasyon + SRX
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 2000);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);
  waitMarcState(0x01, 3000);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SRX);
  return waitMarcState(0x0D, 4000);
}

void enterIdleMode() {
  detachInterrupt(CC1101_GDO0);
  pinMode(CC1101_GDO0, INPUT);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 3000);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x2E);
  isrCount = 0;
  qHead = 0;
  qTail = 0;
}

static rmt_data_t rmtTxBuf[640];

static inline rmt_data_t makeRmtSym(uint16_t d0, uint8_t l0, uint16_t d1,
                                    uint8_t l1) {
  rmt_data_t s;
  s.val = 0;
  s.duration0 = (d0 > 32767) ? 32767 : d0;
  s.level0 = l0 ? 1 : 0;
  s.duration1 = (d1 > 32767) ? 32767 : d1;
  s.level1 = l1 ? 1 : 0;
  return s;
}

bool sendRmtFrameRepeated(const rmt_data_t *frameSyms, int frameLen,
                          int repeats) {
  if (!rmtTxActive || frameLen <= 0 || frameLen > 640 || repeats <= 0)
    return false;
  int maxChunkReps = 640 / frameLen;
  if (maxChunkReps < 1)
    maxChunkReps = 1;

  int remaining = repeats;
  while (remaining > 0) {
    int n = (remaining < maxChunkReps) ? remaining : maxChunkReps;
    for (int r = 0; r < n; r++) {
      memcpy(&rmtTxBuf[r * frameLen], frameSyms,
             (size_t)frameLen * sizeof(rmt_data_t));
    }
    if (!rmtWrite(CC1101_GDO0, rmtTxBuf, (size_t)(n * frameLen), 5000)) {
      return false;
    }
    remaining -= n;
  }
  return true;
}

bool enterTxMode() {
  rmtTxActive = false;
  lastTxRmtOk = false;
  pinMode(CC1101_GDO0, INPUT);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 3000);
  ELECHOUSE_cc1101.setDRate(
      50.0); // 400 kHz async TX ornekleme (2.5 us cozunurluk)

  // SIDLE durumunda (RF kapaliyken) GDO0 cikis hatti dogrulamasi yap
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x2E);
  pinMode(CC1101_GDO0, INPUT | OUTPUT);
  gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  delayMicroseconds(20);
  uint8_t p0 = (uint8_t)gpio_get_level((gpio_num_t)CC1101_GDO0);
  gpio_set_level((gpio_num_t)CC1101_GDO0, 1);
  delayMicroseconds(5);
  uint8_t p1 = (uint8_t)gpio_get_level((gpio_num_t)CC1101_GDO0);
  gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  lastTxGdo0Ok = (p0 == 0 && p1 == 1);

  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0, 0x0D);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL0, 0x32);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG2, 0x30);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND0, 0x11);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0, 0x00);
  uint8_t paTable[8] = {0x00, 0xC0, 0, 0, 0, 0, 0, 0};
  ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_PATABLE, paTable, 8);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);
  waitMarcState(0x01, 3000);

  // STX oncesinde GDO0 pinini donanim RMT TX'e bagla (RMT aktifken
  // pinMode/gpio_set_level cagirilmaz)
  rmtTxActive =
      rmtInit(CC1101_GDO0, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_2, 1000000);
  if (rmtTxActive) {
    rmtSetEOT(CC1101_GDO0, 0);
    rmt_data_t prime = makeRmtSym(20, 0, 20, 0);
    if (!rmtWrite(CC1101_GDO0, &prime, 1, 1000)) {
      rmtDeinit(CC1101_GDO0);
      rmtTxActive = false;
    }
  }
  if (!rmtTxActive) {
    gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  }
  lastTxRmtOk = rmtTxActive;

  ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
  if (!waitMarcState(0x13, 4000)) {
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    waitMarcState(0x01, 2000);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);
    waitMarcState(0x01, 3000);
    ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
    waitMarcState(0x13, 4000);
  }
  if (!rmtTxActive) {
    gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  }
  delayMicroseconds(1200);
  return (cc1101Marc == 0x13);
}

void setTxFreqOffset(uint8_t fsctrl0Val) {
  if (!rmtTxActive) {
    gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  }
  delay(3);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 2000);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0, fsctrl0Val);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
  waitMarcState(0x13, 3000);
  delayMicroseconds(500);
}

void exitAsyncTxMode() {
  if (!rmtTxActive) {
    gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
  }
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 2000);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0, 0x00);
  if (rmtTxActive) {
    rmtDeinit(CC1101_GDO0);
    rmtTxActive = false;
  }
  pinMode(CC1101_GDO0, INPUT);
}

String truncateText(const String &s, size_t maxLen) {
  if (s.length() <= maxLen)
    return s;
  return s.substring(0, maxLen - 2) + "..";
}

String htmlEscape(const String &s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '&')
      out += "&amp;";
    else if (c == '<')
      out += "&lt;";
    else if (c == '>')
      out += "&gt;";
    else if (c == '"')
      out += "&quot;";
    else if (c == '\'')
      out += "&#39;";
    else
      out += c;
  }
  return out;
}

String urlEncode(const String &s) {
  String out;
  const char hex[] = "0123456789ABCDEF";
  for (size_t i = 0; i < s.length(); i++) {
    uint8_t c = (uint8_t)s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += (char)c;
    } else if (c == ' ') {
      out += '+';
    } else {
      out += '%';
      out += hex[(c >> 4) & 0x0F];
      out += hex[c & 0x0F];
    }
  }
  return out;
}

uint16_t calcMedianU16(const uint16_t *arr, int n) {
  uint16_t tmp[32];
  for (int i = 0; i < n; i++)
    tmp[i] = arr[i];
  for (int i = 1; i < n; i++) {
    uint16_t key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  return tmp[n / 2];
}

bool isValidRemoteCode(uint32_t code, uint16_t bits, uint16_t pulse) {
  if (bits != 12 && bits != 24 && bits != 25)
    return false;
  if (pulse < 140 || pulse > 650)
    return false;

  uint32_t mask = (1UL << bits) - 1UL;
  code &= mask;
  if (code == 0 || code == mask)
    return false;

  int ones = 0;
  int transitions = 0;
  int runOne = 0;
  int maxRunOne = 0;
  for (uint16_t i = 0; i < bits; i++) {
    uint32_t b = (code >> i) & 1UL;
    if (b) {
      ones++;
      runOne++;
      if (runOne > maxRunOne)
        maxRunOne = runOne;
    } else {
      runOne = 0;
    }
    if (i + 1 < bits) {
      uint32_t nextB = (code >> (i + 1)) & 1UL;
      if (b != nextB)
        transitions++;
    }
  }
  if (ones < 3 || ones > (int)bits - 3)
    return false;
  if (transitions < 3)
    return false;
  if (maxRunOne >= 19)
    return false;
  return true;
}

DecodedOok evalOokWindow(const uint16_t *pulses, uint16_t count, int offset,
                         uint16_t bits, uint8_t protoId) {
  DecodedOok res = {false, 0, 0, 0, protoId, 0, 0, 999.0f};
  if (bits < 12 || bits > 32 || offset < 0)
    return res;
  if ((uint32_t)offset + (uint32_t)bits * 2UL > count)
    return res;

  uint16_t periods[32];
  uint16_t shorts[32];
  uint16_t longs[32];

  for (uint16_t i = 0; i < bits; i++) {
    uint16_t a = pulses[offset + 2 * i];
    uint16_t b = pulses[offset + 2 * i + 1];
    periods[i] = a + b;
    shorts[i] = (a < b) ? a : b;
    longs[i] = (a < b) ? b : a;
  }

  uint16_t tMed = calcMedianU16(periods, bits);
  uint16_t sMed = calcMedianU16(shorts, bits);
  uint16_t lMed = calcMedianU16(longs, bits);

  if (tMed < 640 || tMed > 2600 || sMed < 140 || sMed > 650) {
    return res;
  }
  float ratio = (float)lMed / (float)sMed;
  if (ratio < 1.65f || ratio > 4.20f) {
    return res;
  }

  float errSum = 0.0f;
  uint32_t code = 0;
  for (uint16_t i = 0; i < bits; i++) {
    if (periods[i] < (uint32_t)(tMed * 0.56f) ||
        periods[i] > (uint32_t)(tMed * 1.44f)) {
      return res;
    }
    if (shorts[i] < (uint32_t)(sMed * 0.36f) ||
        shorts[i] > (uint32_t)(sMed * 1.52f)) {
      return res;
    }
    if (longs[i] < (uint32_t)(lMed * 0.54f) ||
        longs[i] > (uint32_t)(lMed * 1.46f)) {
      return res;
    }

    errSum += fabsf((float)periods[i] - (float)tMed) / (float)tMed;
    uint16_t a = pulses[offset + 2 * i];
    uint16_t b = pulses[offset + 2 * i + 1];
    code = (code << 1) | ((a > b) ? 1UL : 0UL);
  }

  if (!isValidRemoteCode(code, bits, sMed)) {
    return res;
  }

  float score = errSum / (float)bits;
  if (bits == 24) {
    score -= 0.052f; // Standart 24-bit kumandalara en yuksek oncelik
  } else if (bits == 25 && offset == 0 && (count == 50 || count == 51)) {
    score -= 0.065f;
  } else if (bits == 12) {
    score -= 0.015f;
  }

  uint16_t edges = bits * 2;
  if ((uint32_t)offset + edges < count) {
    edges += 1; // Trailing stop darbesini koru
  }

  // OOK alici uzamasi (1:3 darbenin HIGH kismi sismisse) gercek 1T =
  // T_periyot/4 degerini koru
  uint16_t quarterT = (tMed + 2) / 4;
  uint16_t effectivePulse = sMed;
  if (sMed > quarterT + 35 && lMed >= quarterT * 2 + 40 && quarterT >= 140) {
    effectivePulse = quarterT;
  }

  res.valid = true;
  res.code = code;
  res.bits = bits;
  res.pulse = effectivePulse;
  res.proto = protoId;
  res.offset = (uint16_t)offset;
  res.edgeCount = edges;
  res.score = score;
  return res;
}

DecodedOok decodeOokFrame(const uint16_t *pulses, uint16_t count) {
  DecodedOok best = {false, 0, 0, 0, 1, 0, 0, 999.0f};
  if (count < 24)
    return best;

  int maxOffset = (count > 24) ? (int)(count - 24) : 0;

  for (int offset = 0; offset <= maxOffset; offset++) {
    uint16_t availBits = (count - (uint16_t)offset) / 2;
    if (availBits > 32)
      availBits = 32;
    if (availBits < 12)
      break;

    uint8_t protoId = (offset % 2 == 0) ? 1 : 13;
    const uint16_t candidates[3] = {24, 25, 12};

    for (int k = 0; k < 3; k++) {
      uint16_t targetBits = candidates[k];
      if (targetBits == 12 && (count < 24 || count > 29))
        continue;
      if (targetBits > availBits)
        continue;

      DecodedOok cand =
          evalOokWindow(pulses, count, offset, targetBits, protoId);
      if (cand.valid && (!best.valid || cand.score < best.score)) {
        best = cand;
      }
    }
  }
  return best;
}

void saveRemotes() {
  prefs.begin("garaj", false);
  prefs.clear();
  prefs.putInt("schema_v", NVS_SCHEMA_VERSION);
  prefs.putInt("count", remoteCount);
  prefs.putInt("sel", selectedIndex);
  for (int i = 0; i < remoteCount; i++) {
    String idx = String(i);
    prefs.putString(("n" + idx).c_str(), remotes[i].name);
    prefs.putULong(("c" + idx).c_str(), remotes[i].code);
    prefs.putUShort(("b" + idx).c_str(), remotes[i].bits);
    prefs.putUShort(("u" + idx).c_str(), remotes[i].pulse);
    prefs.putUChar(("p" + idx).c_str(), remotes[i].proto);
    prefs.putUShort(("rc" + idx).c_str(), remotes[i].rawCount);
    prefs.putUShort(("sl" + idx).c_str(), remotes[i].syncLow);
    prefs.putInt(("lr" + idx).c_str(), remotes[i].lastRssi);
    if (remotes[i].rawCount > 0) {
      prefs.putBytes(("rp" + idx).c_str(), remotes[i].rawPulses,
                     remotes[i].rawCount * sizeof(uint16_t));
    }
  }
  prefs.end();
}

bool codesMatchFlexible(uint32_t c1, uint16_t b1, uint32_t c2, uint16_t b2) {
  if (c1 == 0 || c2 == 0)
    return false;
  if (c1 == c2 && b1 == b2)
    return true;
  if ((b1 == 24 || b1 == 25) && (b2 == 24 || b2 == 25)) {
    uint32_t low1 = c1 & 0xFFFFFFUL;
    uint32_t low2 = c2 & 0xFFFFFFUL;
    uint32_t hi1 = (b1 == 25) ? (c1 >> 1) : low1;
    uint32_t hi2 = (b2 == 25) ? (c2 >> 1) : low2;
    if (low1 == low2 || hi1 == low2 || low1 == hi2 || hi1 == hi2)
      return true;
  }
  return false;
}

String formatPulsesCsv(const uint16_t *p, uint16_t cnt, uint16_t maxItems) {
  String s;
  uint16_t lim = (cnt < maxItems) ? cnt : maxItems;
  for (uint16_t i = 0; i < lim; i++) {
    if (i > 0)
      s += ",";
    s += String(p[i]);
  }
  if (cnt > lim)
    s += ",...";
  return s;
}

String formatCodeBinary(uint32_t code, uint16_t bits) {
  String s;
  if (bits == 0 || bits > 32)
    return "0";
  for (int b = (int)bits - 1; b >= 0; b--) {
    s += ((code >> b) & 1UL) ? '1' : '0';
    if (b > 0 && (b % 4) == 0)
      s += ' ';
  }
  return s;
}

void dumpRemotesSerial() {
  Serial.println("=== [NVS-DUMP] Kayitli Kumandalar ve Ham Darbeler ===");
  for (int i = 0; i < remoteCount; i++) {
    const RemoteItem &r = remotes[i];
    Serial.printf("#%d Ad='%s' Kod=%lu (0x%lX) %ub %uus P%u Sync=%u RSSI=%ddBm "
                  "HamSayi=%u Bin=[%s]\n",
                  i + 1, r.name.c_str(), (unsigned long)r.code,
                  (unsigned long)r.code, r.bits, r.pulse, r.proto, r.syncLow,
                  r.lastRssi, r.rawCount,
                  formatCodeBinary(r.code, r.bits).c_str());
    Serial.print("   RawPulses: ");
    Serial.println(formatPulsesCsv(r.rawPulses, r.rawCount, MAX_RAW_PULSES));
  }
  Serial.println("=== [NVS-DUMP SONU] ===");
}

void addDefaultGarageRemote() {
  if (remoteCount >= MAX_REMOTES)
    return;
  static const uint16_t rawSubFrame[53] = {
      746, 437, 777, 421,  778, 407,  778, 420, 780, 442,  745, 451,  774, 411,
      759, 421, 805, 417,  768, 434,  777, 417, 769, 433,  746, 441,  769, 426,
      776, 426, 168, 1020, 166, 1007, 769, 429, 168, 1012, 164, 1041, 744, 443,
      755, 422, 777, 443,  753, 422,  776, 439, 754, 423,  3480};
  RemoteItem &r = remotes[remoteCount];
  r.name = "Ev Garaji";
  r.code = 0;
  r.bits = 26;
  r.pulse = 420;
  r.proto = 1;
  r.rawCount = 53;
  r.syncLow = 3561;
  r.lastRssi = -50;
  memcpy(r.rawPulses, rawSubFrame, sizeof(rawSubFrame));
  selectedIndex = remoteCount;
  remoteCount++;
}

void loadRemotes() {
  prefs.begin("garaj", false);
  int schemaV = prefs.getInt("schema_v", 0);
  if (schemaV != NVS_SCHEMA_VERSION) {
    prefs.end();
    remoteCount = 0;
    selectedIndex = 0;
    addDefaultGarageRemote();
    saveRemotes();
    addLog("[KURULUM] Ev Garaji (raw.sub) varsayilan olarak yuklendi.");
    return;
  }

  int savedCount = prefs.getInt("count", 0);
  if (savedCount < 0)
    savedCount = 0;
  if (savedCount > MAX_REMOTES)
    savedCount = MAX_REMOTES;

  int validCount = 0;
  int purgedCount = 0;
  int savedSel = prefs.getInt("sel", 0);
  int mappedSel = 0;

  for (int i = 0; i < savedCount; i++) {
    String idx = String(i);
    RemoteItem item;
    item.name =
        prefs.getString(("n" + idx).c_str(), "Kumanda " + String(i + 1));
    item.code = prefs.getULong(("c" + idx).c_str(), 0);
    item.bits = prefs.getUShort(("b" + idx).c_str(), 24);
    item.pulse = prefs.getUShort(("u" + idx).c_str(), 350);
    item.proto = prefs.getUChar(("p" + idx).c_str(), 1);
    item.rawCount = prefs.getUShort(("rc" + idx).c_str(), 0);
    item.syncLow = prefs.getUShort(("sl" + idx).c_str(), 0);
    item.lastRssi = (int16_t)prefs.getInt(("lr" + idx).c_str(), -120);

    if (item.rawCount > MAX_RAW_PULSES) {
      item.rawCount = 0;
    }
    if (item.rawCount > 0) {
      size_t expectedBytes = item.rawCount * sizeof(uint16_t);
      size_t got =
          prefs.getBytes(("rp" + idx).c_str(), item.rawPulses, expectedBytes);
      if (got != expectedBytes) {
        item.rawCount = 0;
      }
    }

    bool validItem = isValidRemoteCode(item.code, item.bits, item.pulse) ||
                     (item.rawCount >= 20) ||
                     (item.code != 0 && item.bits >= 8 && item.bits <= 32 &&
                      item.pulse >= 100 && item.pulse <= 2000);
    if (validItem) {
      if (i == savedSel) {
        mappedSel = validCount;
      }
      remotes[validCount++] = item;
    } else {
      purgedCount++;
    }
  }
  prefs.end();

  remoteCount = validCount;
  selectedIndex = (remoteCount > 0 && mappedSel < remoteCount) ? mappedSel : 0;

  if (purgedCount > 0) {
    saveRemotes();
    addLog("[TEMIZLIK] " + String(purgedCount) +
           " adet gecersiz kayit silindi.");
  }
}

void saveSelectedIndex() {
  prefs.begin("garaj", false);
  prefs.putInt("sel", selectedIndex);
  prefs.end();
}

int findRemoteByCode(uint32_t code, uint16_t bits) {
  for (int i = 0; i < remoteCount; i++) {
    if (codesMatchFlexible(remotes[i].code, remotes[i].bits, code, bits)) {
      return i;
    }
  }
  return -1;
}

void drawOled() {
  u8g2.clearBuffer();

  // Üst Durum Çubuğu (0..8 px)
  u8g2.setFont(u8g2_font_5x7_tr);
  if (cc1101Ok) {
    char freqBuf[8];
    snprintf(freqBuf, sizeof(freqBuf), "%.1f", currentFreqMhz);
    u8g2.drawStr(0, 7, freqBuf);

    char rightBuf[12];
    if (listenMode) {
      int shownRssi =
          (millis() < displayPeakUntil) ? displayPeakRssi : currentRssi;
      snprintf(rightBuf, sizeof(rightBuf), "%ddBm", shownRssi);
    } else if (remoteCount > 0) {
      snprintf(rightBuf, sizeof(rightBuf), "%d/%d", selectedIndex + 1,
               remoteCount);
    } else {
      snprintf(rightBuf, sizeof(rightBuf), "HAZIR");
    }
    int w = u8g2.getStrWidth(rightBuf);
    u8g2.drawStr(72 - w, 7, rightBuf);
  } else {
    u8g2.drawStr(0, 7, "CC1101 HATA");
  }
  u8g2.drawHLine(0, 9, 72);

  // Orta Alan (10..30 px)
  bool showOverlay = (overlayUntil > millis());
  if (!cc1101Ok && !showOverlay) {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 19, "CC1101 YOK!");
    u8g2.setFont(u8g2_font_5x7_tr);
    char verBuf[16];
    snprintf(verBuf, sizeof(verBuf), "V:0x%02X P:0x%02X", cc1101Ver,
             cc1101Part);
    u8g2.drawStr(0, 28, verBuf);
  } else if (showOverlay) {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 19, truncateText(overlayLine1, 14).c_str());
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 28, truncateText(overlayLine2, 14).c_str());
  } else if (listenMode) {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 19, ">> DINLEME <<");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 28, "Tusa basiniz..");
  } else if (remoteCount == 0) {
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(0, 19, "KAYIT YOK");
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(0, 28, "Web'den Ekle");
  } else {
    const String &name = remotes[selectedIndex].name;
    u8g2.setFont(u8g2_font_7x13B_tr);
    int wLarge = u8g2.getStrWidth(name.c_str());
    if (wLarge <= 72) {
      u8g2.drawStr((72 - wLarge) / 2, 25, name.c_str());
    } else {
      u8g2.setFont(u8g2_font_6x10_tr);
      int wMed = u8g2.getStrWidth(name.c_str());
      if (wMed <= 72) {
        u8g2.drawStr((72 - wMed) / 2, 24, name.c_str());
      } else {
        int splitIdx = name.lastIndexOf(' ', 12);
        String line1, line2;
        if (splitIdx > 0) {
          line1 = name.substring(0, splitIdx);
          line2 = name.substring(splitIdx + 1);
        } else {
          line1 = name.substring(0, 12);
          line2 = name.substring(12);
        }
        line1.trim();
        line2.trim();
        line1 = truncateText(line1, 12);
        line2 = truncateText(line2, 12);
        int w1 = u8g2.getStrWidth(line1.c_str());
        int w2 = u8g2.getStrWidth(line2.c_str());
        u8g2.drawStr((72 - w1) / 2, 19, line1.c_str());
        u8g2.drawStr((72 - w2) / 2, 29, line2.c_str());
      }
    }
  }

  // Alt Bilgi Çubuğu (31..39 px)
  u8g2.drawHLine(0, 31, 72);
  u8g2.setFont(u8g2_font_4x6_tr);
  char footBuf[22];
  snprintf(footBuf, sizeof(footBuf), "192.168.4.1  K:%d", remoteCount);
  u8g2.drawStr(0, 39, footBuf);

  u8g2.sendBuffer();
}

void applyCc1101Config(bool startRx) {
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 3000);
  ELECHOUSE_cc1101.setCCMode(0);
  ELECHOUSE_cc1101.setModulation(2); // ASK/OOK
  ELECHOUSE_cc1101.setSyncMode(0);   // No preamble/sync
  ELECHOUSE_cc1101.setMHZ(currentFreqMhz);
  ELECHOUSE_cc1101.setRxBW(270.00); // Flipper AM270 (~270.8 kHz kanal filtresi)
  ELECHOUSE_cc1101.setDRate(
      6.0); // 48 kHz async ornekleme (~20.8 us cozunurluk, dusuk parazit)
  ELECHOUSE_cc1101.setPA(12);

  // Asynchronous OOK, +12dBm PATABLE ve 26.000 MHz kristal icin sifir frekans
  // ofseti
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0, 0x00);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG2, 0x30);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND0, 0x11);
  uint8_t paTable[8] = {0x00, 0xC0, 0, 0, 0, 0, 0, 0};
  ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_PATABLE, paTable, 8);

  // TI DN022 / Flipper AM270 OokAsync AGC ayarlari (MAGN_TARGET=33dB -> yakinda
  // doyumsuz, uzakta sessiz)
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL2, 0x03);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL1, 0x00);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_AGCCTRL0, 0x91);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND1, 0xB6);

  cc1101Agc2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_AGCCTRL2);

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SCAL);
  waitMarcState(0x01, 4000);
  if (startRx) {
    enterRxMode();
    delay(5);
  } else {
    enterIdleMode();
  }
}

void resetRssiBaseline() {
  currentRssi = ELECHOUSE_cc1101.getRssi();
  peakRssi = currentRssi;
  displayPeakRssi = currentRssi;
  displayPeakUntil = 0;
  if (currentRssi > -125) {
    noiseFloor = (float)currentRssi;
    noiseFloorInit = true;
  } else {
    noiseFloor = -95.0f;
    noiseFloorInit = false;
  }
}

void enterListeningRf() {
  applyCc1101Config(true);
  attachRfIsr();
  resetRssiBaseline();
}

bool initCC1101() {
  ELECHOUSE_cc1101.setSpiPin(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN);
  ELECHOUSE_cc1101.setGDO0(CC1101_GDO0);
  ELECHOUSE_cc1101.Init();

  cc1101Part = ELECHOUSE_cc1101.SpiReadStatus(CC1101_PARTNUM);
  cc1101Ver = ELECHOUSE_cc1101.SpiReadStatus(CC1101_VERSION);
  bool detected =
      ELECHOUSE_cc1101.getCC1101() && (cc1101Ver == 0x14 || cc1101Ver == 0x04);

  Serial.printf("CC1101 PART=0x%02X VER=0x%02X\n", cc1101Part, cc1101Ver);
  if (!detected) {
    addLog("CC1101 HATA! PART=0x" + String(cc1101Part, HEX) + " VER=0x" +
           String(cc1101Ver, HEX));
    return false;
  }

  applyCc1101Config(false);

  uint8_t rIo0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG0);
  uint8_t rPkt0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_PKTCTRL0);
  uint8_t rMdm2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG2);
  uint8_t rFrend0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FREND0);
  uint8_t rFs0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCTRL0);
  uint8_t paRead[2] = {0, 0};
  ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_PATABLE, paRead, 2);

  char regBuf[96];
  snprintf(regBuf, sizeof(regBuf),
           "IOCFG0=0x%02X PKT0=0x%02X MDM2=0x%02X FREND0=0x%02X FS0=0x%02X "
           "PA=[%02X,%02X]",
           rIo0, rPkt0, rMdm2, rFrend0, rFs0, paRead[0], paRead[1]);
  addLog("CC1101 OK (VER=0x" + String(cc1101Ver, HEX) + " MARC=0x" +
         String(cc1101Marc, HEX) + " Freq=" + String(currentFreqMhz, 2) +
         "MHz | " + String(regBuf) + ")");
  return true;
}

void sendOokCustom(uint32_t code, uint16_t bits, uint16_t shortUs,
                   uint16_t longUs, uint8_t proto, uint16_t syncUs, int repeats,
                   uint16_t stopUs) {
  if (bits < 8 || bits > 32 || shortUs < 80 || longUs < 80 || repeats <= 0)
    return;
  uint16_t gapUs = (syncUs >= 1500) ? syncUs : (uint16_t)(shortUs * 22);
  uint16_t trailStopUs = (stopUs >= 80) ? stopUs : shortUs;

  if (rmtTxActive) {
    uint16_t safeGap = (gapUs > 30000) ? 30000 : gapUs;
    rmt_data_t oneFrame[40];
    int frameLen = 0;

    if (proto == 13) {
      uint16_t tmpDur[70];
      uint8_t tmpLvl[70];
      int segs = 0;
      tmpDur[segs] = trailStopUs;
      tmpLvl[segs] = 1;
      segs++;
      for (int b = (int)bits - 1; b >= 0; b--) {
        bool bitVal = (code >> b) & 1UL;
        tmpDur[segs] = bitVal ? longUs : shortUs;
        tmpLvl[segs] = 0;
        segs++;
        tmpDur[segs] = bitVal ? shortUs : longUs;
        tmpLvl[segs] = 1;
        segs++;
      }
      tmpDur[segs] = safeGap;
      tmpLvl[segs] = 0;
      segs++;
      for (int i = 0; i + 1 < segs && frameLen < 40; i += 2) {
        oneFrame[frameLen++] =
            makeRmtSym(tmpDur[i], tmpLvl[i], tmpDur[i + 1], tmpLvl[i + 1]);
      }
    } else {
      for (int b = (int)bits - 1; b >= 0 && frameLen < 39; b--) {
        bool bitVal = (code >> b) & 1UL;
        uint16_t highUs = bitVal ? longUs : shortUs;
        uint16_t lowUs = bitVal ? shortUs : longUs;
        oneFrame[frameLen++] = makeRmtSym(highUs, 1, lowUs, 0);
      }
      oneFrame[frameLen++] = makeRmtSym(trailStopUs, 1, safeGap, 0);
    }

    bool ok = sendRmtFrameRepeated(oneFrame, frameLen, repeats);
    lastTxRmtOk = lastTxRmtOk && ok;
    return;
  }

  for (int rep = 0; rep < repeats; rep++) {
    if (proto == 13) {
      gpio_set_level((gpio_num_t)CC1101_GDO0, 1);
      delayUsSafe(trailStopUs);
      for (int b = (int)bits - 1; b >= 0; b--) {
        bool bitVal = (code >> b) & 1UL;
        uint16_t lowUs = bitVal ? longUs : shortUs;
        uint16_t highUs = bitVal ? shortUs : longUs;
        gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
        delayUsSafe(lowUs);
        gpio_set_level((gpio_num_t)CC1101_GDO0, 1);
        delayUsSafe(highUs);
      }
      gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
      delayUsSafe(gapUs);
    } else {
      for (int b = (int)bits - 1; b >= 0; b--) {
        bool bitVal = (code >> b) & 1UL;
        uint16_t highUs = bitVal ? longUs : shortUs;
        uint16_t lowUs = bitVal ? shortUs : longUs;
        gpio_set_level((gpio_num_t)CC1101_GDO0, 1);
        delayUsSafe(highUs);
        gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
        delayUsSafe(lowUs);
      }
      gpio_set_level((gpio_num_t)CC1101_GDO0, 1);
      delayUsSafe(trailStopUs);
      gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
      delayUsSafe(gapUs);
    }
  }
}

void sendSynthesizedOok(uint32_t code, uint16_t bits, uint16_t pulseUs,
                        uint8_t proto, uint16_t syncUs, int repeats,
                        uint16_t stopUs = 0) {
  if (bits < 8 || bits > 32 || pulseUs < 80)
    return;
  uint16_t longUs =
      (proto == 13) ? (uint16_t)(pulseUs * 2) : (uint16_t)(pulseUs * 3);
  uint16_t trailStopUs = (stopUs >= 80) ? stopUs : pulseUs;
  sendOokCustom(code, bits, pulseUs, longUs, proto, syncUs, repeats,
                trailStopUs);
}

void setListeningMode(bool on, const String &source) {
  listenMode = on;
  if (on) {
    listenModeUntil = millis() + LISTEN_TIMEOUT_MS;
    pendingCode = 0;
    pendingBits = 0;
    pendingRawCount = 0;
    pendingScore = 999.0f;
    lastBurstCode = 0;
    lastUnmatchedCount = 0;
    if (cc1101Ok) {
      enterListeningRf();
    }
    setStatusOverlay(">> DINLEME <<", "Tusa basiniz..", 2200);
    addLog("[DINLEME-MODU] Acildi (" + source + ", 60 sn)");
  } else {
    listenModeUntil = 0;
    if (cc1101Ok) {
      enterIdleMode();
    }
    setStatusOverlay("DINLEME KAPALI", "Hazir", 1400);
    addLog("[DINLEME-MODU] Kapatildi (" + source + ")");
  }
}

void fifoAppendPulse(uint8_t *buf, int &bitPos, int maxBits, uint16_t durUs,
                     uint8_t level, int slotUs) {
  int slots = ((int)durUs + (slotUs / 2)) / slotUs;
  if (slots < 1)
    slots = 1;
  for (int i = 0; i < slots && bitPos < maxBits; i++) {
    if (level) {
      buf[bitPos >> 3] |= (uint8_t)(0x80 >> (bitPos & 7));
    }
    bitPos++;
  }
}

uint8_t readTxBytesSafe() {
  uint8_t a = ELECHOUSE_cc1101.SpiReadStatus(CC1101_TXBYTES);
  for (int i = 0; i < 3; i++) {
    uint8_t b = ELECHOUSE_cc1101.SpiReadStatus(CC1101_TXBYTES);
    if (a == b)
      return b;
    a = b;
  }
  return a;
}

void appendUniqueAlign(OokAlign *list, int &cnt, int maxCnt, uint32_t code,
                       uint16_t bits) {
  if (cnt >= maxCnt || code == 0 || bits < 8)
    return;
  for (int i = 0; i < cnt; i++) {
    if (list[i].code == code && list[i].bits == bits)
      return;
  }
  list[cnt].code = code;
  list[cnt].bits = bits;
  cnt++;
}

uint16_t calcAvgPairPeriod(const uint16_t *pulses, uint16_t rawCount) {
  if (rawCount < 24)
    return 0;
  uint16_t pairs = rawCount / 2;
  if (pairs > 32)
    pairs = 32;
  uint32_t sum = 0;
  for (uint16_t i = 0; i < pairs; i++) {
    sum += (uint32_t)pulses[2 * i] + (uint32_t)pulses[2 * i + 1];
  }
  return (uint16_t)(sum / pairs);
}

void sendRawPulsesBurst(const uint16_t *pulses, uint16_t rawCount,
                        uint16_t gapUs, int repeats) {
  if (rawCount == 0 || repeats <= 0)
    return;

  if (rmtTxActive) {
    uint16_t safeGap = (gapUs > 30000) ? 30000 : gapUs;
    uint16_t cnt = (rawCount > MAX_RAW_PULSES) ? MAX_RAW_PULSES : rawCount;
    rmt_data_t oneFrame[80];
    int frameLen = 0;
    for (uint16_t i = 0; i + 1 < cnt && frameLen < 79; i += 2) {
      oneFrame[frameLen++] = makeRmtSym(pulses[i], 1, pulses[i + 1], 0);
    }
    if ((cnt % 2) == 1) {
      oneFrame[frameLen++] = makeRmtSym(pulses[cnt - 1], 1, safeGap, 0);
    } else {
      oneFrame[frameLen++] = makeRmtSym(safeGap, 0, 10, 0);
    }
    bool ok = sendRmtFrameRepeated(oneFrame, frameLen, repeats);
    lastTxRmtOk = lastTxRmtOk && ok;
    return;
  }

  for (int rep = 0; rep < repeats; rep++) {
    for (uint16_t i = 0; i < rawCount; i++) {
      gpio_set_level((gpio_num_t)CC1101_GDO0, (i % 2 == 0) ? 1 : 0);
      delayUsSafe(pulses[i]);
    }
    gpio_set_level((gpio_num_t)CC1101_GDO0, 0);
    delayUsSafe(gapUs);
  }
}

bool sendFifoOok(const RemoteItem &r, uint16_t shortUs, uint16_t longUs,
                 uint16_t stopUs, uint16_t gapUs, int repeats,
                 float drateKbps) {
  static uint8_t frameBuf[320];
  memset(frameBuf, 0, sizeof(frameBuf));
  int bitPos = 0;
  const int maxBits = 320 * 8;
  int slotUs = (drateKbps >= 35.0f) ? 25 : 50;

  if (r.code != 0 && r.bits >= 8) {
    if (r.proto == 13) {
      fifoAppendPulse(frameBuf, bitPos, maxBits, stopUs, 1, slotUs);
      for (int b = (int)r.bits - 1; b >= 0 && bitPos < maxBits; b--) {
        bool bv = (r.code >> b) & 1UL;
        fifoAppendPulse(frameBuf, bitPos, maxBits, bv ? longUs : shortUs, 0,
                        slotUs);
        fifoAppendPulse(frameBuf, bitPos, maxBits, bv ? shortUs : longUs, 1,
                        slotUs);
      }
    } else {
      for (int b = (int)r.bits - 1; b >= 0 && bitPos < maxBits; b--) {
        bool bv = (r.code >> b) & 1UL;
        fifoAppendPulse(frameBuf, bitPos, maxBits, bv ? longUs : shortUs, 1,
                        slotUs);
        fifoAppendPulse(frameBuf, bitPos, maxBits, bv ? shortUs : longUs, 0,
                        slotUs);
      }
      fifoAppendPulse(frameBuf, bitPos, maxBits, stopUs, 1, slotUs);
    }
  } else if (r.rawCount > 0) {
    for (uint16_t i = 0; i < r.rawCount && bitPos < maxBits; i++) {
      fifoAppendPulse(frameBuf, bitPos, maxBits, r.rawPulses[i],
                      (i % 2 == 0) ? 1 : 0, slotUs);
    }
  }

  // Tam Sync LOW boslugunu da FIFO cercevesine ekle (bitler 0 olarak kaliyor)
  int syncSlots = ((int)gapUs + (slotUs / 2)) / slotUs;
  if (syncSlots < 10)
    syncSlots = 10;
  if (bitPos + syncSlots > maxBits) {
    syncSlots = maxBits - bitPos;
  }
  bitPos += syncSlots;

  if (bitPos < 24)
    return false;
  int frameLen = (bitPos + 7) / 8;
  if (frameLen > 320)
    frameLen = 320;

  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 2000);
  ELECHOUSE_cc1101.setDRate(drateKbps);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_IOCFG0,
                               0x2E); // FIFO TX sirasinda GDO0 Hi-Z
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_PKTCTRL1, 0x00);
  ELECHOUSE_cc1101.SpiWriteReg(
      CC1101_PKTCTRL0, 0x02); // Infinite Packet Length (kesintisiz akis)
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_MDMCFG2, 0x30); // ASK/OOK, No sync
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FREND0, 0x11);
  ELECHOUSE_cc1101.SpiWriteReg(CC1101_FSCTRL0, 0x00);
  uint8_t paTable[8] = {0x00, 0xC0, 0, 0, 0, 0, 0, 0};
  ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_PATABLE, paTable, 8);

  int initChunk = (frameLen <= 60) ? frameLen : 60;
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SFTX);
  ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_TXFIFO, frameBuf,
                                    (uint8_t)initChunk);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_STX);
  if (!waitMarcState(0x13, 3000)) {
    ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
    return false;
  }

  bool allStreamOk = true;
  int pos = initChunk;
  for (int rep = 0; rep < repeats && allStreamOk; rep++) {
    unsigned long startMs = millis();
    while (pos < frameLen) {
      if (millis() - startMs > 250) {
        allStreamOk = false;
        break;
      }
      uint8_t txSt = readTxBytesSafe();
      if (txSt & 0x80) { // TXFIFO_UNDERFLOW
        allStreamOk = false;
        break;
      }
      uint8_t txBytes = txSt & 0x7F;
      if (txBytes <= 44) {
        int space = 60 - (int)txBytes;
        int chunk = frameLen - pos;
        if (chunk > 16)
          chunk = 16;
        if (chunk > space)
          chunk = space;
        if (chunk > 0) {
          ELECHOUSE_cc1101.SpiWriteBurstReg(CC1101_TXFIFO, &frameBuf[pos],
                                            (uint8_t)chunk);
          pos += chunk;
          continue;
        }
      }
      delayMicroseconds(200);
    }
    pos = 0;
  }

  unsigned long drainStart = millis();
  while (millis() - drainStart < 150) {
    uint8_t txSt = readTxBytesSafe();
    uint8_t txBytes = txSt & 0x7F;
    if (txBytes <= 1 || (txSt & 0x80))
      break;
    delayMicroseconds(500);
  }
  delay(2);
  ELECHOUSE_cc1101.SpiStrobe(CC1101_SIDLE);
  waitMarcState(0x01, 2000);
  return allStreamOk;
}

bool transmitRemote(int idx) {
  if (!cc1101Ok || idx < 0 || idx >= remoteCount)
    return false;

  selectedIndex = idx;
  saveSelectedIndex();

  const RemoteItem &r = remotes[idx];
  setStatusOverlay("GONDERILIYOR!", r.name, 1800);
  drawOled();
  setLed(true);

  detachInterrupt(CC1101_GDO0);
  bool txOk = enterTxMode();
  uint8_t txMarc = cc1101Marc;

  uint16_t pulseUs = (r.pulse >= 100 && r.pulse <= 1500) ? r.pulse : 370;
  uint16_t longUs =
      (r.proto == 13) ? (uint16_t)(pulseUs * 2) : (uint16_t)(pulseUs * 3);
  uint16_t stopUs = (r.rawCount > 0 && (r.rawCount % 2 == 1) &&
                     r.rawPulses[r.rawCount - 1] > pulseUs * 2)
                        ? (uint16_t)(pulseUs * 3)
                        : pulseUs;
  uint16_t primarySync = (r.syncLow >= 1500 && r.syncLow <= 30000)
                             ? r.syncLow
                             : (uint16_t)(pulseUs * 31UL);

  if (txOk) {
    if (r.rawCount > 0) {
      sendRawPulsesBurst(r.rawPulses, r.rawCount, primarySync, 14);
    }
    if (r.code != 0 && r.bits >= 8) {
      delay(4);
      sendOokCustom(r.code, r.bits, pulseUs, longUs, r.proto, primarySync, 12,
                    stopUs);
    }
  }
  exitAsyncTxMode();

  bool fifoOk = sendFifoOok(r, pulseUs, longUs, stopUs, primarySync, 10, 40.0f);
  if (!fifoOk) {
    fifoOk = sendFifoOok(r, pulseUs, longUs, stopUs, primarySync, 10, 20.0f);
  }

  if (listenMode) {
    enterListeningRf();
  } else {
    applyCc1101Config(false);
  }

  setLed(false);
  unsigned long now = millis();
  lastRawSignalTime = now;
  lastDecodedTime = now;

  char hexBuf[16];
  snprintf(hexBuf, sizeof(hexBuf), "0x%lX", (unsigned long)r.code);
  addLog("[TX] " + r.name + " | " +
         (r.code > 0 ? ("Kod=" + String(hexBuf) + " " + String(r.bits) + "b")
                     : ("RAW " + String(r.rawCount) +
                        "k Sync=" + String(primarySync) + "us")) +
         " | TxMARC=0x" + String(txMarc, HEX) + (txOk ? "" : "(ERR)") +
         " GDO0_PIN=" + (lastTxGdo0Ok ? "OK" : "FAIL") + " RMT=" +
         (lastTxRmtOk ? "OK" : "FAIL") + " FIFO=" + (fifoOk ? "OK" : "FAIL") +
         " -> MARC=0x" + String(cc1101Marc, HEX));

  drawOled();
  return txOk;
}

void redirectHome(const String &msg) {
  server.sendHeader("Location", "/?msg=" + urlEncode(msg), true);
  server.send(303, "text/plain", "");
}

void handleRoot() {
  String msg = server.arg("msg");
  String html;
  html.reserve(9500);

  html += F(
      "<!DOCTYPE html><html lang='tr'><head><meta charset='UTF-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>RF REMOTE</title><style>"
      "body{font-family:ui-monospace,'Cascadia Code',Consolas,monospace;"
      "background:#050507;"
      "background-image:radial-gradient(circle at 50% 0%,rgba(185,28,28,0.18),"
      "transparent 60%),repeating-linear-gradient(0deg,rgba(220,38,38,0.03) 0 "
      "1px,transparent 1px 3px);color:#e2e8f0;margin:0;padding:14px}"
      ".wrap{max-width:680px;margin:0 auto}"
      ".hdr{border-bottom:1px solid #45121a;padding-bottom:10px;margin-bottom:"
      "12px}"
      "h1{font-size:22px;margin:0;color:#ff2a3c;letter-spacing:1.5px;"
      "text-shadow:0 0 12px rgba(255,42,60,0.45);display:flex;align-items:"
      "center;gap:8px}"
      ".dot{width:9px;height:9px;border-radius:50%;background:#ff2a3c;"
      "box-shadow:0 0 8px #ff2a3c;animation:blink 1.4s infinite}"
      "@keyframes blink{0%,100%{opacity:1}50%{opacity:0.25}}"
      ".sub{font-size:11px;color:#9ca3af;margin-top:4px;letter-spacing:0.8px}"
      ".bar{display:flex;flex-wrap:wrap;gap:8px;margin-bottom:14px}"
      ".pill{background:#0f0b0c;border:1px solid #45121a;color:#e2e8f0;"
      "border-radius:6px;padding:5px 10px;font-size:12px}"
      ".pill a{color:#f87171;text-decoration:none;font-weight:700}"
      ".pill a:hover{color:#ff4d5a}"
      ".ok{border-color:#22c55e;color:#4ade80;box-shadow:0 0 8px "
      "rgba(34,197,94,0.2)}"
      ".err{border-color:#ef4444;color:#ff4d5a}"
      ".msg{background:#1a070a;border:1px solid #dc2626;color:#fecaca;"
      "padding:10px 12px;border-radius:6px;margin-bottom:12px;font-size:13px}"
      ".card{background:#0b0b0f;border:1px solid #3b0d14;border-left:3px "
      "solid #991b1b;border-radius:8px;padding:12px;margin-bottom:10px}"
      ".card.sel{border-color:#ef4444;border-left:3px solid #ff2a3c;"
      "box-shadow:0 0 14px rgba(239,68,68,0.28)}"
      ".row{display:flex;justify-content:space-between;align-items:center;"
      "gap:8px;flex-wrap:wrap}"
      ".title{font-weight:700;font-size:15px;color:#f8fafc}"
      ".meta{font-size:12px;color:#94a3b8;margin-top:4px}"
      ".btns{display:flex;gap:6px;flex-wrap:wrap;margin-top:8px}"
      "button,a.btn{cursor:pointer;border-radius:6px;padding:8px 12px;"
      "font-size:12px;font-weight:700;text-decoration:none;display:"
      "inline-block;font-family:inherit;letter-spacing:0.4px}"
      ".tx{background:#16a34a;color:#021409;border:1px solid #4ade80;"
      "box-shadow:0 0 10px rgba(34,197,94,0.25)}"
      ".selb{background:#161013;color:#e2e8f0;border:1px solid #4c1d24}"
      ".ren{background:#1c0c10;color:#fca5a5;border:1px solid #991b1b}"
      ".del{background:#7f1d1d;color:#fef2f2;border:1px solid #ef4444}"
      ".freqa{background:#dc2626;color:#fff;border:1px solid #ff4d5a;"
      "box-shadow:0 0 10px rgba(220,38,38,0.35)}"
      ".lst{background:#1a0a0c;color:#fbbf24;border:1px solid #d97706;"
      "width:100%;text-align:center;box-sizing:border-box}"
      ".lston{background:#15803d;color:#ecfdf5;border:1px solid #4ade80;"
      "box-shadow:0 0 12px rgba(34,197,94,0.35);width:100%;text-align:center;"
      "box-sizing:border-box}"
      "input,textarea{background:#060608;color:#f1f5f9;border:1px solid "
      "#45121a;border-radius:6px;padding:7px 9px;font-size:12px;"
      "font-family:inherit;box-sizing:border-box}"
      "input:focus,textarea:focus{outline:none;border-color:#ef4444;"
      "box-shadow:0 0 6px rgba(239,68,68,0.35)}"
      "textarea{width:100%;margin-top:8px}"
      "form.inline{display:flex;gap:6px;flex-wrap:wrap;align-items:center}"
      ".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
      "pre.logs{background:#020304;border:1px solid #3b0d14;border-radius:6px;"
      "padding:10px;font-size:11px;line-height:1.45;color:#4ade80;"
      "max-height:220px;overflow:auto;white-space:pre-wrap;"
      "word-break:break-word;margin:8px 0 0}"
      "@media(max-width:480px){.grid{grid-template-columns:1fr}}"
      "</style></head><body><div class='wrap'><div class='hdr'>"
      "<h1><span class='dot'></span>RF REMOTE // ");
  html += String(currentFreqMhz, 2);
  html += F(" MHz</h1><div class='sub'>// SUB-GHZ TRANSCEIVER // CC1101 // "
            "192.168.4.1</div></div><div class='bar'>");

  if (cc1101Ok) {
    char statBuf[16];
    snprintf(statBuf, sizeof(statBuf), "0x%02X/M:0x%02X", cc1101Ver,
             cc1101Marc);
    html +=
        "<span class='pill ok'>CC1101 Aktif (" + String(statBuf) + ")</span>";
    if (listenMode) {
      html += "<span class='pill ok'>Durum: Dinleme Acik (" +
              String(currentRssi) + " dBm)</span>";
    } else {
      html += F("<span class='pill'>Durum: Hazir (Bosta)</span>");
    }
  } else {
    html += F("<span class='pill err'>CC1101 Baglanti Hatasi</span>");
  }
  html += "<span class='pill'>Kayit: " + String(remoteCount) + " / " +
          String(MAX_REMOTES) + "</span>";
  html += F("<span class='pill'><a href='/dump'>Ham Veri (/dump)</a></span>");
  html += F("<span class='pill'><a href='/'>Yenile</a></span></div>");

  // Dinleme modu karti
  html += F("<div class='card'><div class='title' "
            "style='margin-bottom:8px'>Kumanda Yakalama / Dinleme Modu</div>");
  if (listenMode) {
    long remSec = (long)((listenModeUntil - millis()) / 1000UL);
    if (remSec < 0)
      remSec = 0;
    html += "<a class='btn lston' href='/listen?on=0'>Dinleme Modu AKTIF (" +
            String(remSec) + " sn kaldi) - Kapat</a>";
    html += F("<div class='meta'>Simdi RF kumandanizin tusuna basin. Kod "
              "otomatik yakalanip kaydedilecektir.</div>");
  } else {
    html += F("<a class='btn lst' href='/listen?on=1'>Dinleme "
              "Modu Baslat (60 sn)</a>");
  }
  html += F("</div>");

  // Frekans secimi
  const float freqs[4] = {433.92f, 433.42f, 315.00f, 868.35f};
  html += F("<div class='card'><div class='title' "
            "style='margin-bottom:8px'>Frekans Ayari</div><div class='btns'>");
  for (int i = 0; i < 4; i++) {
    bool activeF = fabsf(currentFreqMhz - freqs[i]) < 0.05f;
    String fStr = String(freqs[i], 2);
    html += "<a class='btn " + String(activeF ? "freqa" : "selb") +
            "' href='/freq?mhz=" + fStr + "'>" + fStr + " MHz</a>";
  }
  html += F("</div></div>");

  if (msg.length() > 0) {
    html += "<div class='msg'>" + htmlEscape(msg) + "</div>";
  }

  if (remoteCount == 0) {
    html +=
        F("<div class='card'><div class='meta' style='margin-bottom:8px'>"
          "Henuz kayitli kumanda yok. Dinleme modunu acabilir, asagidan "
          "manuel ekleyebilir veya varsayilan ev garajini yukleyebilirsiniz."
          "</div><a class='btn freqa' href='/restore_default'>Varsayilan "
          "Ev Garaji (raw.sub) Yukle</a></div>");
  } else {
    for (int i = 0; i < remoteCount; i++) {
      const RemoteItem &r = remotes[i];
      bool isSel = (i == selectedIndex);
      html += "<div class='card" + String(isSel ? " sel" : "") +
              "'><div class='row'><div>";
      html += "<div class='title'>" + String(i + 1) + ". " +
              htmlEscape(r.name) + (isSel ? " [Acilis + BOOT Secili]" : "") +
              "</div>";
      if (r.code == 0 && r.rawCount > 0) {
        html += "<div class='meta'>RAW (" + String(r.rawCount) +
                " kenar) | Sync: " + String(r.syncLow) + " us</div>";
      } else {
        char hexBuf[16];
        snprintf(hexBuf, sizeof(hexBuf), "0x%lX", (unsigned long)r.code);
        html += "<div class='meta'>Kod: " + String((unsigned long)r.code) +
                " (" + String(hexBuf) + ") | " + String(r.bits) + " bit | " +
                String(r.pulse) + " us | Proto " + String(r.proto) +
                (r.rawCount > 0 ? " | Ham(" + String(r.rawCount) + ")"
                                : " | Sentez") +
                (r.lastRssi > -120 ? " | " + String(r.lastRssi) + " dBm" : "") +
                "</div>";
      }
      html += "</div><div class='btns'><a class='btn tx' href='/tx?id=" +
              String(i) + "'>Gonder</a>";
      if (!isSel) {
        html += "<a class='btn selb' href='/select?id=" + String(i) +
                "'>Acilista Sec</a>";
      }
      html += "</div></div>";
      html +=
          "<div class='btns'><form class='inline' method='POST' "
          "action='/rename'>"
          "<input type='hidden' name='id' value='" +
          String(i) +
          "'>"
          "<input type='text' name='name' maxlength='24' value='" +
          htmlEscape(r.name) +
          "' placeholder='Yeni ad'>"
          "<button class='ren' type='submit'>Adlandir</button></form>"
          "<form class='inline' method='POST' action='/del' onsubmit=\"return "
          "confirm('Silinsin mi?');\">"
          "<input type='hidden' name='id' value='" +
          String(i) +
          "'>"
          "<button class='del' type='submit'>Sil</button></form></div></div>";
    }
    html +=
        F("<div class='card'><form method='POST' action='/clear' "
          "onsubmit=\"return confirm('Tum kayitli kumandalar silinsin mi?');\">"
          "<button class='del' type='submit' style='width:100%'>Tum Kayitlari "
          "Sifirla</button></form></div>");
  }

  html +=
      F("<div class='card'><div class='title' style='margin-bottom:8px'>Manuel "
        "Kumanda Ekle</div>"
        "<form method='POST' action='/add'><div class='grid'>"
        "<input type='text' name='name' maxlength='24' placeholder='Ad (orn: B "
        "Blok)' required>"
        "<input type='text' name='code' placeholder='Kod (orn: 16150185 veya "
        "0xF66EA9)' required>"
        "<input type='number' name='bits' value='24' min='8' max='32' "
        "placeholder='Bit'>"
        "<input type='number' name='pulse' value='372' min='100' max='2000' "
        "placeholder='Pulse (us)'>"
        "<input type='number' name='proto' value='1' min='1' max='13' "
        "placeholder='Protokol'>"
        "<button class='tx' type='submit'>Ekle</button></div></form></div>");

  html += F(
      "<div class='card'><div class='title' style='margin-bottom:8px'>Bruce "
      ".sub RAW Ekle</div>"
      "<form method='POST' action='/add_raw'><div class='grid'>"
      "<input type='text' name='name' maxlength='24' placeholder='Ad (orn: "
      "Bahce Kapisi)' required>"
      "<input type='number' name='sync' value='0' min='0' max='30000' "
      "placeholder='Sync us (0=otomatik)'></div>"
      "<textarea name='raw' rows='3' placeholder='RAW_Data: 746 -437 777 -421 "
      "...' required></textarea>"
      "<button class='tx' type='submit' style='margin-top:8px'>RAW Kumanda "
      "Ekle</button></form></div>");

  // Son olay loglari (sayfa acildiginda gosterilir, arka plan sorgusu/isinma
  // yok)
  html += F("<div class='card'><div class='row'><div class='title'>Son Olay "
            "Loglari</div>"
            "<a class='btn selb' href='/'>Loglari Yenile</a></div><pre "
            "class='logs'>");
  if (webLogCount == 0) {
    html += F("Henuz olay kaydi yok.");
  } else {
    for (int i = webLogCount - 1; i >= 0; i--) {
      html += htmlEscape(webLogs[i]) + "\n";
    }
  }
  html += F("</pre><div class='meta'>BOOT tusu: Kisa bas = Secili kumandayi "
            "gonder | Basili tut = Sonraki kumandaya gec</div></div></div>"
            "</body></html>");

  server.send(200, "text/html", html);
}

int getIdArg() {
  if (!server.hasArg("id"))
    return -1;
  String s = server.arg("id");
  s.trim();
  if (s.length() == 0)
    return -1;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isDigit(s[i]))
      return -1;
  }
  return s.toInt();
}

void handleListen() {
  bool on = (server.arg("on") != "0");
  setListeningMode(on, "Web");
  drawOled();
  redirectHome(on ? "Dinleme modu acildi (60 sn). Simdi kumanda tusuna basin."
                  : "Dinleme modu kapatildi.");
}

void handleFreq() {
  if (!cc1101Ok || !server.hasArg("mhz")) {
    redirectHome("Frekans degistirilemedi.");
    return;
  }
  float req = server.arg("mhz").toFloat();
  if (req < 300.0f || req > 928.0f) {
    redirectHome("Gecersiz frekans araligi.");
    return;
  }
  currentFreqMhz = req;
  detachInterrupt(CC1101_GDO0);
  if (listenMode) {
    enterListeningRf();
  } else {
    applyCc1101Config(false);
  }

  addLog("[FREQ] Yeni frekans: " + String(currentFreqMhz, 2) + " MHz");
  setStatusOverlay("FREKANS", String(currentFreqMhz, 2) + " MHz", 1600);
  drawOled();
  redirectHome("Frekans " + String(currentFreqMhz, 2) + " MHz yapildi.");
}

void handleTx() {
  int id = getIdArg();
  if (transmitRemote(id)) {
    redirectHome(remotes[id].name + " gonderildi!");
  } else {
    redirectHome("Gonderim basarisiz (CC1101 veya ID hatasi).");
  }
}

void handleSelect() {
  int id = getIdArg();
  if (id >= 0 && id < remoteCount) {
    selectedIndex = id;
    saveSelectedIndex();
    drawOled();
    addLog("[WEB-SEC] Acilis/BOOT kumandasi: " + remotes[id].name);
    redirectHome(remotes[id].name + " acilis ve BOOT tusu icin secildi.");
  } else {
    redirectHome("Kumanda bulunamadi.");
  }
}

void handleRename() {
  int id = getIdArg();
  String newName = server.arg("name");
  newName.trim();
  if (id >= 0 && id < remoteCount && newName.length() > 0) {
    if (newName.length() > 24)
      newName = newName.substring(0, 24);
    String oldName = remotes[id].name;
    remotes[id].name = newName;
    saveRemotes();
    addLog("[WEB-AD] " + oldName + " -> " + newName);
    redirectHome("Kumanda adi guncellendi.");
  } else {
    redirectHome("Gecersiz ad veya ID.");
  }
}

void handleDel() {
  int id = getIdArg();
  if (id >= 0 && id < remoteCount) {
    String deletedName = remotes[id].name;
    uint32_t deletedCode = remotes[id].code;
    for (int i = id; i < remoteCount - 1; i++) {
      remotes[i] = remotes[i + 1];
    }
    remoteCount--;
    if (selectedIndex >= remoteCount) {
      selectedIndex = (remoteCount > 0) ? (remoteCount - 1) : 0;
    }
    saveRemotes();
    addLog("[WEB-SILINDI] " + deletedName + " (Kod=0x" +
           String(deletedCode, HEX) +
           ") silindi. Kalan=" + String(remoteCount));
    setStatusOverlay("SILINDI", deletedName, 1500);
    drawOled();
    redirectHome(deletedName + " silindi.");
  } else {
    redirectHome("Silinecek kumanda bulunamadi.");
  }
}

void handleClear() {
  int prevCount = remoteCount;
  remoteCount = 0;
  selectedIndex = 0;
  pendingCode = 0;
  pendingBits = 0;
  pendingRawCount = 0;
  pendingScore = 999.0f;
  lastBurstCode = 0;
  saveRemotes();
  if (listenMode) {
    setListeningMode(false, "Sifirlama");
  }
  addLog("[WEB-TEMIZLENDI] Tum kayitlar (" + String(prevCount) +
         " adet) silindi.");
  drawOled();
  redirectHome("Tum kumandalar silindi.");
}

void handleDump() {
  String out;
  out.reserve(3500);
  out += "=== RF REMOTE HAM VERI DOKUMU ===\n";
  out += "Frekans: " + String(currentFreqMhz, 2) +
         " MHz | Taban: " + String((int)roundf(noiseFloor)) +
         " dBm | MARC: 0x" + String(cc1101Marc, HEX) +
         " | Kayit: " + String(remoteCount) + "\n\n";
  for (int i = 0; i < remoteCount; i++) {
    const RemoteItem &r = remotes[i];
    char hexBuf[16];
    snprintf(hexBuf, sizeof(hexBuf), "0x%lX", (unsigned long)r.code);
    out += "#" + String(i + 1) + " " + r.name + "\n";
    out += "  Kod: " + String((unsigned long)r.code) + " (" + String(hexBuf) +
           ") | " + String(r.bits) + " bit | " + String(r.pulse) +
           " us | Proto " + String(r.proto) + " | Sync=" + String(r.syncLow) +
           " us | RSSI=" + String(r.lastRssi) + " dBm\n";
    out += "  Binary: " + formatCodeBinary(r.code, r.bits) + "\n";
    out += "  HamKenar (" + String(r.rawCount) +
           "): " + formatPulsesCsv(r.rawPulses, r.rawCount, MAX_RAW_PULSES) +
           "\n\n";
  }
  out += "=== SON LOGLAR ===\n";
  for (int i = webLogCount - 1; i >= 0; i--) {
    out += webLogs[i] + "\n";
  }
  server.send(200, "text/plain; charset=utf-8", out);
}

void handleRestoreDefault() {
  if (remoteCount < MAX_REMOTES) {
    addDefaultGarageRemote();
    saveRemotes();
    drawOled();
    addLog("[WEB-VARSAYILAN] Ev Garaji (raw.sub) eklendi ve secildi.");
    redirectHome("Ev Garaji (raw.sub) eklendi ve acilis icin secildi.");
  } else {
    redirectHome("Hafiza dolu! Once gereksiz bir kumanda silin.");
  }
}

void handleAddRaw() {
  if (remoteCount >= MAX_REMOTES) {
    redirectHome("Hafiza dolu (maks 20 kumanda).");
    return;
  }
  String name = server.arg("name");
  String rawStr = server.arg("raw");
  name.trim();
  rawStr.trim();

  if (name.length() == 0 || rawStr.length() == 0) {
    redirectHome("Ad ve RAW veri alanlari zorunludur.");
    return;
  }
  if (name.length() > 24)
    name = name.substring(0, 24);

  int rawPos = rawStr.indexOf("RAW_Data:");
  if (rawPos >= 0) {
    rawStr = rawStr.substring(rawPos + 9);
    int nlPos = rawStr.indexOf('\n');
    if (nlPos >= 0) {
      rawStr = rawStr.substring(0, nlPos);
    }
  }

  uint16_t parsed[MAX_RAW_PULSES];
  uint16_t cnt = 0;
  uint16_t detectedSync = 0;
  bool foundFrame = false;

  const char *p = rawStr.c_str();
  while (*p && !foundFrame) {
    while (*p && (*p == ' ' || *p == '\t' || *p == ',' || *p == ';' ||
                  *p == '\r' || *p == '\n')) {
      p++;
    }
    if (!*p)
      break;
    char *endPtr = nullptr;
    long val = strtol(p, &endPtr, 10);
    if (endPtr == p) {
      p++;
      continue;
    }
    p = endPtr;
    if (val == 0)
      continue;

    long absVal = (val < 0) ? -val : val;
    if (absVal > 60000L)
      absVal = 60000L;

    if (val < 0 && absVal >= 1500) {
      if (cnt >= 20) {
        detectedSync = (uint16_t)absVal;
        foundFrame = true;
      } else {
        cnt = 0;
      }
      continue;
    }

    if (cnt == 0 && val < 0) {
      continue;
    }
    if (cnt < MAX_RAW_PULSES) {
      parsed[cnt++] = (uint16_t)absVal;
    } else {
      break;
    }
  }

  if (cnt < 20) {
    redirectHome("Gecerli RAW darbe dizisi bulunamadi (en az 20 kenar).");
    return;
  }

  long userSync = server.arg("sync").toInt();
  uint16_t finalSync = 3500;
  if (userSync >= 1500 && userSync <= 30000) {
    finalSync = (uint16_t)userSync;
  } else if (detectedSync >= 1500 && detectedSync <= 30000) {
    finalSync = detectedSync;
  }

  RemoteItem &nr = remotes[remoteCount];
  nr.name = name;
  nr.code = 0;
  nr.bits = cnt / 2;
  nr.pulse = 350;
  nr.proto = 1;
  nr.rawCount = cnt;
  nr.syncLow = finalSync;
  nr.lastRssi = 0;
  memcpy(nr.rawPulses, parsed, cnt * sizeof(uint16_t));
  selectedIndex = remoteCount;
  remoteCount++;
  saveRemotes();
  drawOled();

  addLog("[WEB-RAW] " + name + " | " + String(cnt) +
         " kenar | Sync=" + String(finalSync) + " us");
  redirectHome(name + " (RAW " + String(cnt) + " kenar) eklendi.");
}

void handleAdd() {
  if (remoteCount >= MAX_REMOTES) {
    redirectHome("Hafiza dolu (maks 20 kumanda).");
    return;
  }
  String name = server.arg("name");
  String codeStr = server.arg("code");
  name.trim();
  codeStr.trim();

  if (name.length() == 0 || codeStr.length() == 0) {
    redirectHome("Ad ve kod alanlari zorunludur.");
    return;
  }
  if (name.length() > 24)
    name = name.substring(0, 24);

  uint32_t code = (uint32_t)strtoul(codeStr.c_str(), nullptr, 0);
  int bits = constrain(server.arg("bits").toInt(), 8, 32);
  int pulse = constrain(server.arg("pulse").toInt(), 100, 2000);
  int proto = constrain(server.arg("proto").toInt(), 1, 13);

  if (code == 0) {
    redirectHome("Gecersiz kod degeri.");
    return;
  }

  remotes[remoteCount].name = name;
  remotes[remoteCount].code = code;
  remotes[remoteCount].bits = (uint16_t)bits;
  remotes[remoteCount].pulse = (uint16_t)pulse;
  remotes[remoteCount].proto = (uint8_t)proto;
  remotes[remoteCount].rawCount = 0;
  remotes[remoteCount].syncLow = 0;
  remotes[remoteCount].lastRssi = 0;
  selectedIndex = remoteCount;
  remoteCount++;
  saveRemotes();

  addLog("[WEB-KAYIT] " + name + " | Kod=" + String((unsigned long)code) +
         " (0x" + String(code, HEX) + ") " + String(bits) + "b " +
         String(pulse) + "us P" + String(proto));
  redirectHome(name + " eklendi.");
}

void processOneFrame(const uint16_t *localPulses, uint16_t count,
                     uint16_t syncLow, unsigned long now, bool rfActive) {
  int16_t sigRssi = (displayPeakRssi > currentRssi) ? (int16_t)displayPeakRssi
                                                    : (int16_t)currentRssi;
  int floorDbm = (int)roundf(noiseFloor);
  if (noiseFloorInit && sigRssi < floorDbm + 10 &&
      isIdleOscillation(localPulses, count, true)) {
    return;
  }

  DecodedOok dec = decodeOokFrame(localPulses, count);
  if (!dec.valid && count >= 70) {
    for (uint16_t i = 23; i + 24 < count; i += 2) {
      if (localPulses[i] >= 1100) {
        DecodedOok d1 = decodeOokFrame(localPulses, i);
        if (d1.valid) {
          dec = d1;
          syncLow = localPulses[i];
          break;
        }
        DecodedOok d2 = decodeOokFrame(&localPulses[i + 1], count - (i + 1));
        if (d2.valid) {
          d2.offset += (i + 1);
          dec = d2;
          break;
        }
      }
    }
  }

  if (dec.valid) {
    int idx = findRemoteByCode(dec.code, dec.bits);
    bool twoFrameConfirmed =
        codesMatchFlexible(pendingCode, pendingBits, dec.code, dec.bits) &&
        (now - pendingTime <= 1400);
    bool singleFrameStrong =
        rfActive && (sigRssi >= floorDbm + 10) && (sigRssi >= -84) &&
        (dec.bits == 24 || dec.bits == 25) && (dec.score <= 0.16f);
    bool allowAccept = (idx >= 0) ? (twoFrameConfirmed || singleFrameStrong)
                                  : twoFrameConfirmed;

    if (allowAccept) {
      DecodedOok chosenDec = dec;
      const uint16_t *chosenPulses = localPulses;
      uint16_t chosenCount = count;
      uint16_t chosenSync = syncLow;

      if (twoFrameConfirmed && pendingRawCount > 0 &&
          pendingScore < dec.score) {
        chosenDec.code = pendingCode;
        chosenDec.bits = pendingBits;
        chosenDec.pulse = pendingPulse;
        chosenDec.proto = pendingProto;
        chosenDec.offset = pendingOffset;
        chosenDec.edgeCount = pendingEdgeCount;
        chosenDec.score = pendingScore;
        chosenPulses = pendingPulses;
        chosenCount = pendingRawCount;
        chosenSync = pendingSyncLow;
      }

      pendingCode = 0;
      pendingBits = 0;
      pendingRawCount = 0;
      pendingScore = 999.0f;

      lastDecodedTime = now;
      lastRawSignalTime = now;

      if (idx >= 0 &&
          codesMatchFlexible(chosenDec.code, chosenDec.bits, lastBurstCode,
                             chosenDec.bits) &&
          (now - lastBurstTime < 700) && (sigRssi <= remotes[idx].lastRssi)) {
        return;
      }
      lastBurstCode = chosenDec.code;
      lastBurstTime = now;

      triggerLedBlink(150);
      uint16_t saveOffset = chosenDec.offset;
      uint16_t saveCount =
          (chosenDec.edgeCount > 0 &&
           chosenDec.offset + chosenDec.edgeCount <= chosenCount)
              ? chosenDec.edgeCount
              : chosenCount;
      if (saveCount > MAX_RAW_PULSES)
        saveCount = MAX_RAW_PULSES;

      char hexBuf[16];
      snprintf(hexBuf, sizeof(hexBuf), "0x%lX", (unsigned long)chosenDec.code);

      if (idx >= 0) {
        selectedIndex = idx;
        bool shouldRefresh =
            (remotes[idx].rawCount == 0) || (sigRssi >= remotes[idx].lastRssi);
        bool syncFixInListen =
            listenMode && chosenSync >= 1500 && chosenSync <= 25000 &&
            (abs((int)chosenSync - (int)remotes[idx].syncLow) > 800);
        if (shouldRefresh && saveCount > 0) {
          remotes[idx].code = chosenDec.code;
          remotes[idx].bits = chosenDec.bits;
          remotes[idx].pulse = chosenDec.pulse;
          remotes[idx].proto = chosenDec.proto;
          remotes[idx].rawCount = saveCount;
          if (chosenSync >= 1200 || remotes[idx].syncLow == 0) {
            remotes[idx].syncLow = chosenSync;
          }
          remotes[idx].lastRssi = sigRssi;
          for (uint16_t i = 0; i < saveCount; i++) {
            remotes[idx].rawPulses[i] = chosenPulses[saveOffset + i];
          }
          saveRemotes();
        } else if (syncFixInListen) {
          remotes[idx].syncLow = chosenSync;
          saveRemotes();
        } else {
          saveSelectedIndex();
        }

        if (listenMode) {
          setStatusOverlay("KAYIT YENILENDI", remotes[idx].name, 2400);
          addLog("[RX-GUNCEL] " + remotes[idx].name +
                 " | Kod=" + String((unsigned long)chosenDec.code) + " (" +
                 String(hexBuf) + ") " + String(chosenDec.bits) + "b " +
                 String(chosenDec.pulse) + "us P" + String(chosenDec.proto) +
                 " Skor=" + String(chosenDec.score, 2) +
                 " Kenar=" + String(saveCount) + " Sync=" + String(chosenSync) +
                 " | RSSI=" + String(sigRssi) + "dBm");
        } else {
          setStatusOverlay(shouldRefresh ? "SINYAL IYILESTI" : "KOD ALGILANDI",
                           remotes[idx].name, 2200);
          addLog("[RX] Bilinen #" + String(idx + 1) + " (" + remotes[idx].name +
                 "): " + String((unsigned long)chosenDec.code) + " (" +
                 String(hexBuf) + ") " + String(chosenDec.bits) + "b " +
                 String(chosenDec.pulse) + "us | RSSI=" + String(sigRssi) +
                 "dBm");
        }
      } else if (remoteCount < MAX_REMOTES) {
        RemoteItem &nr = remotes[remoteCount];
        nr.name = "Kumanda " + String(remoteCount + 1);
        nr.code = chosenDec.code;
        nr.bits = chosenDec.bits;
        nr.pulse = chosenDec.pulse;
        nr.proto = chosenDec.proto;
        nr.rawCount = saveCount;
        nr.syncLow = chosenSync;
        nr.lastRssi = sigRssi;
        for (uint16_t i = 0; i < saveCount; i++) {
          nr.rawPulses[i] = chosenPulses[saveOffset + i];
        }
        selectedIndex = remoteCount;
        remoteCount++;
        saveRemotes();

        setStatusOverlay("YENI KAYIT!", nr.name, 2600);
        addLog("[RX-KAYIT] " + nr.name + " | Kod=" +
               String((unsigned long)nr.code) + " (" + String(hexBuf) + ") " +
               String(nr.bits) + "b " + String(nr.pulse) + "us P" +
               String(nr.proto) + " Skor=" + String(chosenDec.score, 2) +
               " Kenar=" + String(nr.rawCount) + " Sync=" + String(nr.syncLow) +
               " | RSSI=" + String(sigRssi) + "dBm");
      } else {
        setStatusOverlay("HAFIZA DOLU!", String(hexBuf), 2000);
        addLog(
            "[RX] Hafiza dolu! Kod=" + String((unsigned long)chosenDec.code) +
            " (" + String(hexBuf) + ")");
      }
      drawOled();
    } else {
      pendingCode = dec.code;
      pendingBits = dec.bits;
      pendingPulse = dec.pulse;
      pendingProto = dec.proto;
      pendingOffset = dec.offset;
      pendingEdgeCount = dec.edgeCount;
      pendingSyncLow = syncLow;
      pendingScore = dec.score;
      pendingTime = now;
      pendingRawCount = (count <= MAX_ISR_PULSES) ? count : MAX_ISR_PULSES;
      for (uint16_t i = 0; i < pendingRawCount; i++) {
        pendingPulses[i] = localPulses[i];
      }
      char hexBuf[16];
      snprintf(hexBuf, sizeof(hexBuf), "0x%lX", (unsigned long)dec.code);
      addLog("[ADAY-KOD] " + String((unsigned long)dec.code) + " (" +
             String(hexBuf) + ") " + String(dec.bits) + "b " +
             String(dec.pulse) + "us P" + String(dec.proto) +
             " Skor=" + String(dec.score, 2) + " Kenar=" + String(count) +
             " | RSSI=" + String(sigRssi) + "dBm");
    }
  } else if (count >= 24 && (rfActive || listenMode || sigRssi >= -80)) {
    bool allowUnmatchedLog =
        (now - lastUnmatchedTime > 150) ||
        (abs((int)count - (int)lastUnmatchedCount) >= 12) ||
        (syncLow >= 1500 && lastUnmatchedSync == 0);
    if (allowUnmatchedLog) {
      lastUnmatchedCount = count;
      lastUnmatchedSync = syncLow;
      lastUnmatchedTime = now;
      String extraTail = "";
      if (count > 55) {
        extraTail =
            " | Son20=" + formatPulsesCsv(&localPulses[count - 20], 20, 20);
      }
      addLog("[HAM-ADAY] Kenar=" + String(count) + " Sync=" + String(syncLow) +
             "us RSSI=" + String(sigRssi) + "dBm | Ilk40=" +
             formatPulsesCsv(localPulses, count, 40) + extraTail);
    }
    if (listenMode && (sigRssi >= -84) &&
        (!noiseFloorInit || sigRssi >= floorDbm + 8) &&
        (now - lastRawSavedTime > 4000)) {
      bool twoRawConfirmed = (pendingCode == 0) && (pendingRawCount >= 24) &&
                             (abs((int)pendingRawCount - (int)count) <= 8) &&
                             (now - pendingTime <= 1800);
      if (twoRawConfirmed && remoteCount < MAX_REMOTES) {
        lastRawSavedTime = now;
        uint16_t saveCount = (count <= MAX_RAW_PULSES) ? count : MAX_RAW_PULSES;
        RemoteItem &nr = remotes[remoteCount];
        nr.name = "Ham Kumanda " + String(remoteCount + 1);
        nr.code = 0;
        nr.bits = count / 2;
        nr.pulse = 350;
        nr.proto = 1;
        nr.rawCount = saveCount;
        nr.syncLow = (syncLow >= 1200) ? syncLow : 3500;
        nr.lastRssi = sigRssi;
        for (uint16_t i = 0; i < saveCount; i++) {
          nr.rawPulses[i] = localPulses[i];
        }
        selectedIndex = remoteCount;
        remoteCount++;
        saveRemotes();

        pendingRawCount = 0;
        pendingTime = 0;
        lastDecodedTime = now;
        lastRawSignalTime = now;
        triggerLedBlink(200);
        setStatusOverlay("HAM KAYIT!", nr.name, 2600);
        addLog("[HAM-KAYIT] " + nr.name + " | Kenar=" + String(nr.rawCount) +
               " Sync=" + String(nr.syncLow) + "us | RSSI=" + String(sigRssi) +
               "dBm");
        drawOled();
      } else {
        pendingCode = 0;
        pendingBits = 0;
        pendingRawCount = (count <= MAX_ISR_PULSES) ? count : MAX_ISR_PULSES;
        pendingSyncLow = syncLow;
        pendingTime = now;
        for (uint16_t i = 0; i < pendingRawCount; i++) {
          pendingPulses[i] = localPulses[i];
        }
      }
    }
  }
}

void handleRfReceive(unsigned long now) {
  if (!cc1101Ok || !listenMode)
    return;

  if (now >= listenModeUntil) {
    setListeningMode(false, "ZamanAsimi");
    drawOled();
    return;
  }

  // 25 ms hizli RSSI okuma (ve kuyrukta cerceve varsa anlik guncelleme) ve
  // dinamik gurultu tabani takibi
  bool periodicRssiTick = (now - lastRssiRead >= 25);
  if (periodicRssiTick || qTail != qHead) {
    if (periodicRssiTick) {
      lastRssiRead = now;
    }
    int sample = ELECHOUSE_cc1101.getRssi();
    if (sample > -125) {
      currentRssi = sample;

      if (!noiseFloorInit) {
        noiseFloor = (float)sample;
        noiseFloorInit = true;
      } else if (periodicRssiTick && sample < (int)roundf(noiseFloor) + 10) {
        noiseFloor += ((float)sample - noiseFloor) * 0.05f;
      }

      if (sample > peakRssi) {
        peakRssi = sample;
      }
      if (sample > displayPeakRssi || now >= displayPeakUntil) {
        displayPeakRssi = sample;
        displayPeakUntil = now + 800;
      }
    }
  }

  int peakSig =
      (displayPeakRssi > currentRssi) ? (int)displayPeakRssi : (int)currentRssi;
  bool rfActive = noiseFloorInit && (peakSig >= -88) &&
                  (peakSig >= (int)roundf(noiseFloor) + 8);

  // 4-slot kuyruktaki tum hazir cerceveleri sirayla isle
  while (qTail != qHead) {
    uint16_t localPulses[MAX_ISR_PULSES];
    uint16_t count = frameQueue[qTail].count;
    uint16_t syncLow = frameQueue[qTail].syncLow;
    if (count > MAX_ISR_PULSES)
      count = MAX_ISR_PULSES;
    for (uint16_t i = 0; i < count; i++) {
      localPulses[i] = frameQueue[qTail].pulses[i];
    }
    qTail = (uint8_t)((qTail + 1) % FRAME_QUEUE_LEN);

    processOneFrame(localPulses, count, syncLow, millis(), rfActive);
  }

  // Dinamik ham RF hareket uyarisi (en az -86 dBm ve Taban + 10 dB veya > -72
  // dBm)
  bool rfSpike =
      noiseFloorInit && (currentRssi >= -86) &&
      ((currentRssi >= (int)roundf(noiseFloor) + 10) || (currentRssi > -72));
  if (rfSpike && (now - lastRawSignalTime > 1200) &&
      (now - lastDecodedTime > 1500)) {
    lastRawSignalTime = now;
    triggerLedBlink(140);
    String rssiTxt = "Peak " + String(displayPeakRssi) + "dBm";
    setStatusOverlay("RF HAREKET!", rssiTxt, 1600);
    uint16_t minDt = (isrMinDtSeen == 65535) ? 0 : isrMinDtSeen;
    addLog("[RF-HAREKET] RSSI=" + String(currentRssi) +
           " dBm Peak=" + String(displayPeakRssi) +
           " dBm Taban=" + String((int)roundf(noiseFloor)) +
           " dBm | IsrKenar=" + String(isrEdgeTotal) + " MaxSayi=" +
           String(isrMaxCountSeen) + " Glitch=" + String(isrGlitchCount) +
           " Dt=" + String(minDt) + ".." + String(isrMaxDtSeen) + "us");
    isrMaxCountSeen = 0;
    isrGlitchCount = 0;
    isrMinDtSeen = 65535;
    isrMaxDtSeen = 0;
  }
}

void handleBootButton(unsigned long now) {
  bool reading = digitalRead(BOOT_BUTTON);
  if (reading != lastBtnReading) {
    lastBtnDebounce = now;
    lastBtnReading = reading;
  }

  if ((now - lastBtnDebounce) > 40 && reading != stableBtnState) {
    stableBtnState = reading;
    if (stableBtnState == LOW) {
      btnPressStartMs = now;
      btnLongHandled = false;
    } else {
      // Tus birakildi: uzun basma yapilmadiysa kisa basma (TX) uygula
      if (!btnLongHandled) {
        if (!cc1101Ok) {
          setStatusOverlay("CC1101 HATA!", "TX Yapilamaz", 1800);
        } else if (remoteCount == 0) {
          setStatusOverlay("KAYIT YOK!", "Web'den Ekle", 1800);
        } else {
          transmitRemote(selectedIndex);
        }
        drawOled();
      }
    }
  }

  // 600 ms basili tutuldugunda sonraki kumandaya gec
  if (stableBtnState == LOW && !btnLongHandled &&
      (now - btnPressStartMs >= 600)) {
    btnLongHandled = true;
    if (remoteCount > 1) {
      selectedIndex = (selectedIndex + 1) % remoteCount;
      saveSelectedIndex();
      triggerLedBlink(120);
      setStatusOverlay("SECILDI " + String(selectedIndex + 1) + "/" +
                           String(remoteCount),
                       remotes[selectedIndex].name, 1800);
      addLog("[BOOT-SEC] Secili kumanda: #" + String(selectedIndex + 1) + " " +
             remotes[selectedIndex].name);
      drawOled();
    } else if (remoteCount == 1) {
      triggerLedBlink(80);
      setStatusOverlay("TEK KUMANDA", remotes[0].name, 1200);
      drawOled();
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  setLed(false);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  unsigned long usbWait = millis();
  while (!Serial && millis() - usbWait < 200) {
    delay(10);
  }
  Serial.println("\n=== RF Remote Baslatiliyor ===");

  u8g2.begin();
  loadRemotes();

  cc1101Ok = initCC1101();

  if (cc1101Ok && remoteCount > 0) {
    addLog("[AUTO-TX] Acilista secili kumanda gonderiliyor: " +
           remotes[selectedIndex].name);
    transmitRemote(selectedIndex);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.printf("Wi-Fi AP Baslatildi -> SSID: %s | IP: %s\n", AP_SSID,
                WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/listen", HTTP_GET, handleListen);
  server.on("/freq", HTTP_GET, handleFreq);
  server.on("/tx", handleTx);
  server.on("/select", handleSelect);
  server.on("/rename", HTTP_POST, handleRename);
  server.on("/del", HTTP_POST, handleDel);
  server.on("/del", HTTP_GET, handleDel);
  server.on("/clear", HTTP_POST, handleClear);
  server.on("/restore_default", HTTP_GET, handleRestoreDefault);
  server.on("/dump", HTTP_GET, handleDump);
  server.on("/add", HTTP_POST, handleAdd);
  server.on("/add_raw", HTTP_POST, handleAddRaw);
  server.begin();

  drawOled();
}

void dumpCc1101RegsSerial() {
  if (!cc1101Ok) {
    Serial.println("[CC1101-REGS] CC1101 baslatilamadi!");
    return;
  }
  uint8_t iocfg0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_IOCFG0);
  uint8_t pktctrl1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_PKTCTRL1);
  uint8_t pktctrl0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_PKTCTRL0);
  uint8_t fsctrl1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCTRL1);
  uint8_t fsctrl0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCTRL0);
  uint8_t freq2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FREQ2);
  uint8_t freq1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FREQ1);
  uint8_t freq0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FREQ0);
  uint8_t mdm4 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG4);
  uint8_t mdm3 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG3);
  uint8_t mdm2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG2);
  uint8_t mdm1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG1);
  uint8_t mdm0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MDMCFG0);
  uint8_t agc2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_AGCCTRL2);
  uint8_t agc1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_AGCCTRL1);
  uint8_t agc0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_AGCCTRL0);
  uint8_t frend1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FREND1);
  uint8_t frend0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FREND0);
  uint8_t mcsm0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_MCSM0);
  uint8_t fscal3 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL3);
  uint8_t fscal2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL2);
  uint8_t fscal1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL1);
  uint8_t fscal0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_FSCAL0);
  uint8_t test2 = ELECHOUSE_cc1101.SpiReadReg(CC1101_TEST2);
  uint8_t test1 = ELECHOUSE_cc1101.SpiReadReg(CC1101_TEST1);
  uint8_t test0 = ELECHOUSE_cc1101.SpiReadReg(CC1101_TEST0);
  uint8_t marc = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
  uint8_t pa[8] = {0};
  ELECHOUSE_cc1101.SpiReadBurstReg(CC1101_PATABLE, pa, 8);

  Serial.printf("[CC1101-REGS] IOCFG0=0x%02X PKTCTRL1=0x%02X PKTCTRL0=0x%02X "
                "FSCTRL1=0x%02X FSCTRL0=0x%02X FREQ=0x%02X%02X%02X\n",
                iocfg0, pktctrl1, pktctrl0, fsctrl1, fsctrl0, freq2, freq1,
                freq0);
  Serial.printf("[CC1101-REGS] MDMCFG4=0x%02X MDMCFG3=0x%02X MDMCFG2=0x%02X "
                "MDMCFG1=0x%02X MDMCFG0=0x%02X\n",
                mdm4, mdm3, mdm2, mdm1, mdm0);
  Serial.printf("[CC1101-REGS] AGCCTRL2=0x%02X AGCCTRL1=0x%02X AGCCTRL0=0x%02X "
                "FREND1=0x%02X FREND0=0x%02X MCSM0=0x%02X\n",
                agc2, agc1, agc0, frend1, frend0, mcsm0);
  Serial.printf("[CC1101-REGS] FSCAL=[%02X,%02X,%02X,%02X] "
                "TEST=[%02X,%02X,%02X] MARCSTATE=0x%02X\n",
                fscal3, fscal2, fscal1, fscal0, test2, test1, test0, marc);
  Serial.printf("[CC1101-REGS] PA=[%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X]\n",
                pa[0], pa[1], pa[2], pa[3], pa[4], pa[5], pa[6], pa[7]);
}

void handleSerialCommands() {
  static String serialCmdBuf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r')
      continue;
    if (c != '\n') {
      if (serialCmdBuf.length() < 32)
        serialCmdBuf += c;
      continue;
    }
    String cmd = serialCmdBuf;
    serialCmdBuf = "";
    cmd.trim();
    cmd.toLowerCase();
    if (cmd.length() == 0)
      continue;

    if (cmd == "dump") {
      dumpRemotesSerial();
    } else if (cmd == "regs") {
      dumpCc1101RegsSerial();
    } else if (cmd == "listen") {
      setListeningMode(!listenMode, "Serial");
    } else if (cmd.startsWith("tx")) {
      int num = cmd.substring(2).toInt();
      int idx = num - 1;
      if (idx >= 0 && idx < remoteCount) {
        Serial.printf("[SERI-CMD] tx%d calistiriliyor: %s\n", num,
                      remotes[idx].name.c_str());
        bool ok = transmitRemote(idx);
        Serial.printf("[SERI-CMD] tx%d sonuc=%s TxGDO0=%s RMT=%s MARC=0x%02X\n",
                      num, ok ? "OK" : "ERR", lastTxGdo0Ok ? "OK" : "FAIL",
                      lastTxRmtOk ? "OK" : "FAIL", cc1101Marc);
      } else {
        Serial.printf("[SERI-CMD] Gecersiz slot: %s (Kayit=%d)\n", cmd.c_str(),
                      remoteCount);
      }
    } else {
      Serial.printf("[SERI-CMD] Bilinmeyen komut: %s (Komutlar: tx1..tx%d, "
                    "dump, regs, listen)\n",
                    cmd.c_str(), remoteCount);
    }
  }
}

void loop() {
  unsigned long now = millis();

  if (!nvsDumpedToSerial && now >= 3000) {
    nvsDumpedToSerial = true;
    dumpRemotesSerial();
  }

  handleSerialCommands();

  if (ledOffTime > 0 && now >= ledOffTime) {
    setLed(false);
    ledOffTime = 0;
  }

  server.handleClient();
  handleRfReceive(now);
  handleBootButton(now);

  // Aktif RF darbe dizisi gelirken I2C OLED cizimini 150 ms ertele (sessiz
  // pencere)
  bool pulseStreamBusy = listenMode && (isrCount >= 6) &&
                         ((uint32_t)(micros() - isrLastUs) < 15000);
  if (now - lastOledDraw >= 400) {
    if (!pulseStreamBusy || now >= oledBusySkipUntil) {
      lastOledDraw = now;
      oledBusySkipUntil = now + 150;
      drawOled();
    }
  }

  if (now - lastSerialLog >= 1000) {
    lastSerialLog = now;
    if (cc1101Ok) {
      cc1101Marc = ELECHOUSE_cc1101.SpiReadStatus(CC1101_MARCSTATE) & 0x1F;
      if (listenMode && cc1101Marc != 0x0D) {
        uint8_t badMarc = cc1101Marc;
        enterRxMode();
        attachRfIsr();
        addLog("[RX-WATCHDOG] MARC=0x" + String(badMarc, HEX) + " -> RX (0x" +
               String(cc1101Marc, HEX) + ") kurtarildi");
      }
    }
    Serial.printf(
        "[DURUM] CC1101=%s PART=0x%02X VER=0x%02X MARC=0x%02X Freq=%.2fMHz | "
        "RSSI=%d Peak=%d Taban=%.0f dBm | Kayit=%d | Dinleme=%s | AP=%s\n",
        cc1101Ok ? "OK" : "HATA", cc1101Part, cc1101Ver, cc1101Marc,
        currentFreqMhz, currentRssi, peakRssi, noiseFloor, remoteCount,
        listenMode ? "ACIK" : "KAPALI", WiFi.softAPIP().toString().c_str());
    peakRssi = currentRssi;
  }
}
