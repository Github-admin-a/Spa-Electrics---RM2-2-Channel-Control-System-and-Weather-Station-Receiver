/*
  Pool Light + HP1000 Weather Bridge  —  NodeMCU (ESP8266) + 2 x RFM69  (v5.0)
  ===========================================================================
  One board, two radios on the same SPI bus:

    RADIO A  915 MHz RFM69H  - Pool Light Bridge (was PoolLightBridge v4.7)
             sends/receives the Spa Electrics remote's power-toggle and SYNC
             packets at 916.02 MHz and tracks the light's on/off + colour.
    RADIO B  433 MHz RFM69   - HP1000 Weather Station receiver (was the
             ESP32-C3 + CC1101 sketch v1.3). Receive only: Outdoor array
             (Fine Offset WH24) and Indoor sensor (WH25), 2-FSK 17.24 kbps,
             sync 0x2DD4, 433.906 MHz.

  WIRING (3.3 V ONLY - never 5 V on an RFM69)
    Shared by both radios:
      SCK  -> D5 (GPIO14)     MISO -> D6 (GPIO12)     MOSI -> D7 (GPIO13)
      RST  -> D2 (GPIO4)      (both radios' RESET pins joined to D2)
      3.3V -> 3V3             GND  -> GND
    915 MHz RFM69H (pool light):
      NSS  -> D8 (GPIO15)     DIO0 -> D1 (GPIO5)
    433 MHz RFM69 (weather):
      NSS  -> D3 (GPIO0)      DIO0 -> D0 (GPIO16)
    Left free: D4 (on-board LED), RX, TX, A0.

    Why these pins: D3/D4/D8 are the ESP8266's boot-strap pins. Only chip
    selects are on them (D8 has the board's pull-down, D3 its pull-up, and an
    RFM69's NSS is an input, so neither can upset booting). A DIO0 output on
    D3/D4 could hold the pin low at power-up and stop the board booting, so
    the 433 MHz DIO0 goes to D0, which has no interrupt and is simply polled
    (the RFM69 holds a received frame in its FIFO until it is read).

    Fit a 10-100 uF capacitor across 3.3V/GND next to the 915 MHz RFM69H
    (it draws ~130 mA while transmitting at 20 dBm). Antennas: 433 MHz =
    17.3 cm wire, 915 MHz = 8.2 cm wire; keep the two a few cm apart.

  WEB PAGES (http://<ip>/ or http://poollight.local/)
    /                     Pool light control
    /settings             Device, WiFi, MQTT (shared), pool remote + timing
    /weather              Weather sensors + receiver status
    /weather/setup        433 MHz radio, allowed sensor IDs, weather topic
    /weather/calibration  Wind direction / pressure / solar / UV calibration
    /update               Firmware upload (OTA)

  First boot opens the WiFi portal "PoolWeather-Setup".
  Then on /settings set your timezone and your pool remote's ID + light
  number (they default to UTC and 0000/0, which the light ignores).

  SECURITY: the web pages, /update and Arduino IDE OTA have no password.
  Only run this on a network you trust.

  MQTT: one broker connection for both. Pool topics are unchanged
  (<base>/..., default base "poollight"); weather topics are unchanged
  (<weather prefix>/outdoor_<id>/state etc., default prefix
  "homeassistant/sensor/hp1000"). The weather entities keep their Home
  Assistant identity (hp1000_station_*); their availability now follows the
  bridge's single LWT topic <base>/status. To keep the pool light's existing
  Home Assistant device, copy its "Home Assistant device ID" from the old
  board's Settings page into this one.

  Self-recovery (from the weather sketch v1.3): WiFi reconnect every 30 s
  while offline and a restart after 10 minutes; restart if the MQTT broker
  is unreachable for 15 minutes with WiFi up. Neither restart happens while
  a pool light action is running. (The ESP8266's own watchdogs already
  reset the chip if loop() stops.)

  Libraries: RadioLib 7.6.0 or 7.7.x, WiFiManager (tzapu) 2.0.x,
             PubSubClient 2.8, ArduinoJson 7.x
  Board: "NodeMCU 1.0 (ESP-12E Module)", ESP8266 core 3.x,
         Flash Size "4MB (FS:2MB OTA:~1019KB)"
*/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <time.h>
#include <RadioLib.h>
#include <SPI.h>

#define FW_VERSION "5.0"

// ------------------------------------------------------------------
//  Pins
// ------------------------------------------------------------------
#define PIN_RST      4    // D2  RESET of BOTH radios
#define PIN_NSS      15   // D8  915 MHz (pool) chip select
#define PIN_DIO0     5    // D1  915 MHz (pool) DIO0 (interrupt)
#define PIN_WX_NSS   0    // D3  433 MHz (weather) chip select
#define PIN_WX_DIO0  16   // D0  433 MHz (weather) DIO0 (polled)

// ------------------------------------------------------------------
//  Radio A: 915 MHz pool light
// ------------------------------------------------------------------
#define RF_FREQUENCY    916.02  // MHz
#define RF_BITRATE      9.6     // kbps
#define RF_DEVIATION    155.0   // kHz
#define RF_RXBW         500.0   // kHz
// TX power and high-power mode are now Settings (defaults: 20 dBm, high
// power on - required for an RFM69H, whose antenna pin is only wired to
// the high-power amplifier).
#define RF_PREAMBLE_LEN 32      // bits (4 x 0xAA)

// ------------------------------------------------------------------
//  Radio B: 433 MHz weather (frequency / deviation / bandwidth are settings)
// ------------------------------------------------------------------
#define WX_BITRATE     17.24  // kbps (Fine Offset WH24/WH25)
#define MAX_PKT        17     // Outdoor (WH24) frame length (Indoor/WH25 is shorter)

// Reset is done by hand on the shared PIN_RST, so RadioLib gets "no reset pin".
SPISettings rfmSpiSettings(1000000, MSBFIRST, SPI_MODE0);
Module rfmModule(PIN_NSS, PIN_DIO0, RADIOLIB_NC, RADIOLIB_NC, SPI, rfmSpiSettings);
RF69 radio(&rfmModule);   // own Module object so registers can be read back
bool radioOk = false;
String radioError;

Module wxModule(PIN_WX_NSS, PIN_WX_DIO0, RADIOLIB_NC, RADIOLIB_NC, SPI, rfmSpiSettings);
RF69 wxRadio(&wxModule);
bool wxRadioOk = false;
String wxRadioError = "not started";
uint8_t wxChipVersion = 0;

#define CMD_POWER 0x11
#define CMD_SYNC  0x13    // remote's SYNC button: only works while the light is on;
                          // light flashes blue (reset) until switched off

// ------------------------------------------------------------------
//  Light colours, in the order the light steps through them
// ------------------------------------------------------------------
struct Colour { const char* name; const char* css; };
const Colour COLOURS[] = {
  {"Electric Blue",      "#1f5bff"},
  {"Midnight Magenta",   "#c0269e"},
  {"Ruby Red",           "#e0115f"},
  {"Spring Lime",        "#a4e33b"},
  {"Lagoon Green",       "#1fb57a"},
  {"Tropical Aqua",      "#19d3d3"},
  {"Daylight White",     "#f4f7ff"},
  {"Warm White",         "#ffd9a0"},
  {"Slow Colour Blend",  "linear-gradient(90deg,#1f5bff,#c0269e,#a4e33b)"},
  {"Fast Colour Change", "linear-gradient(90deg,#e0115f,#19d3d3,#ffd9a0)"},
};
const int NUM_COLOURS = sizeof(COLOURS) / sizeof(COLOURS[0]);

// ------------------------------------------------------------------
//  Weather types (kept above the first function so the Arduino IDE's
//  auto-generated prototypes see them)
// ------------------------------------------------------------------
enum { DEC_NONE = 0, DEC_OK = 1, DEC_BADCRC = 2 };

struct Reading {
  uint8_t model;      // 24 or 25
  uint8_t id;
  int     battOk;     // 1 = battery OK, 0 = low
  float   temp;       // C,   NAN if invalid/absent
  int     hum;        // %,   -1 if invalid/absent
  int     windDir;    // deg, -1 if invalid/absent
  float   wind;       // m/s, NAN if invalid/absent
  float   gust;       // m/s
  float   rain;       // mm (cumulative)
  int     uv;         // UV index, -1 if absent
  int     uvRaw;      // raw UV sensor value, -1 if absent
  float   pressure;   // hPa, NAN if absent
  float   solarWm2;   // solar radiation, W/m^2, NAN if absent (Outdoor/WH24 only)
};

struct Device {
  bool     used;
  bool     discovered;   // HA discovery published for this device
  uint8_t  model, id;
  uint32_t count;
  char     timeStr[24];
  int      rssi;
  Reading  r;
};

#define MAX_DEVICES 8
Device devs[MAX_DEVICES];

#define RECENT_N 10
String  recentLog[RECENT_N];
uint8_t recentHead  = 0;
uint8_t recentCount = 0;

// ------------------------------------------------------------------
//  Config (LittleFS /config.json)
// ------------------------------------------------------------------
struct Config {
  char deviceName[32]  = "Pool Light";
  char hostname[32]    = "poollight";
  char deviceId[24]    = "";            // stable HA identity, not tied to MAC
  char mqttHost[64]    = "";
  uint16_t mqttPort    = 1883;
  char mqttUser[32]    = "";
  char mqttPass[64]    = "";
  char baseTopic[48]   = "poollight";
  char discPrefix[32]  = "homeassistant";
  // Set these to your own remote on the Settings page (capture them with the
  // RX log or an SDR). All zero = not set; the light will ignore it.
  uint8_t remoteIdHi   = 0x00;
  uint8_t remoteIdLo   = 0x00;
  uint8_t lightNum     = 0x00;
  uint8_t unknownByte  = 0x01;
  // The real remote sends each press as 4 identical packets ~17 ms apart
  // (4 packets over ~50 ms in the log). Matching it gives the light's
  // receiver four chances to catch the press.
  uint8_t txRepeats    = 4;             // RF packets per press
  uint8_t txGapMs      = 4;             // extra gap between packets (ms)
  int8_t txPower       = 20;            // dBm: -2..20 high power, -18..13 normal
  uint8_t highPower    = 1;             // 1 = RFM69H/HW/HCW (PA_BOOST)
  // --- light timing ---
  uint16_t stepOffMs   = 2500;  // OFF time used to step colour (manual: 2 s)
  uint16_t onDwellMs   = 2000;  // ON time before the next step
  uint16_t rapidOffMs  = 500;   // OFF time for the "rapid" OFF/ON in sync
  uint16_t rapidMaxMs  = 1000;  // an OFF shorter than this counts as rapid
  uint16_t resetWaitMs = 3000;  // wait in RESET mode before the final OFF/ON
  uint16_t syncSettleMs = 3000; // light ON this long before sending SYNC
  uint16_t keepSecs    = 60;    // OFF longer than this keeps the colour
  uint16_t rxWindowMs  = 400;   // identical remote packets within this = 1 press
} cfg;

enum JobType { JOB_NONE, JOB_GOTO, JOB_ON, JOB_OFF, JOB_SYNC };

// --- weather settings (saved in the same /config.json) ---
char  mqtt_topic[64]  = "homeassistant/sensor/hp1000";   // weather topic prefix
float radio_freq_mhz  = 433.906f;   // centre of the sensors' FSK tones
float radio_dev_khz   = 57.0f;
float radio_bw_khz    = 200.0f;     // RFM69 steps: ...100, 125, 166.7, 200, 250...
char  allowed_ids[48] = "";         // comma separated; blank = accept every ID
char  tz_string[40]   = "UTC0";     // POSIX timezone string, set on the Settings page

// Calibration offsets/gains applied after decode, to match what the
// station's own console shows.
int   cal_wind_dir_offset = 0;      // degrees, added then wrapped to 0-359
float cal_pressure_offset = 0.0f;   // hPa, added to the Indoor reading
float cal_solar_gain      = 1.0f;   // multiplier on the computed W/m^2
float cal_uv_gain         = 1.0f;   // multiplier on the raw UV value

// --- weather runtime ---
uint32_t validFrames = 0;
uint32_t crcFrames   = 0;
uint32_t noiseFrames = 0;
unsigned long lastBridgePub = 0;

// --- self-recovery / diagnostics ---
#define WIFI_RETRY_MS    30000UL            // force a reconnect this often while offline
#define WIFI_RESTART_MS  (10UL * 60000UL)   // offline this long -> restart
#define MQTT_RESTART_MS  (15UL * 60000UL)   // WiFi up but no broker this long -> restart
volatile uint32_t wifiDrops      = 0;       // WiFi disconnects since boot
volatile int      wifiLastReason = 0;       // reason code of the last disconnect
volatile bool     wifiSeenUp     = false;
String            resetReason    = "Unknown";
bool              otaInProgress  = false;
unsigned long     mqttDownSince  = 0;       // 0 = MQTT is up (or not configured)

int colourIndexByName(String n) {
  n.trim();
  for (int i = 0; i < NUM_COLOURS; i++) if (n.equalsIgnoreCase(COLOURS[i].name)) return i;
  if (n.toInt() >= 1 && n.toInt() <= NUM_COLOURS) return n.toInt() - 1;   // "1".."10"
  return -1;
}

// Entities from older versions, removed from Home Assistant on connect.
const char* const OLD_ENTITIES[][2] = {
  {"button", "btn_white"}, {"button", "btn_aqua"}, {"button", "btn_blue"},
  {"button", "btn_pink"}, {"button", "btn_red"}, {"button", "btn_yellow"},
  {"button", "btn_green"}, {"button", "btn_preset1"}, {"button", "btn_preset2"},
  {"button", "btn_preset3"}, {"button", "btn_preset4"}, {"button", "btn_off"},
  {"button", "cycle_start"}, {"button", "cycle_stop"}, {"sensor", "last_tx"},
};

void copyStr(char* dst, size_t n, const char* src) {
  strncpy(dst, src ? src : "", n - 1);
  dst[n - 1] = 0;
}

void saveConfig() {
  JsonDocument d;
  d["deviceName"] = cfg.deviceName;  d["hostname"] = cfg.hostname;
  d["deviceId"]   = cfg.deviceId;    d["mqttHost"] = cfg.mqttHost;
  d["mqttPort"]   = cfg.mqttPort;    d["mqttUser"] = cfg.mqttUser;
  d["mqttPass"]   = cfg.mqttPass;    d["baseTopic"] = cfg.baseTopic;
  d["discPrefix"] = cfg.discPrefix;
  d["remoteIdHi"] = cfg.remoteIdHi;  d["remoteIdLo"] = cfg.remoteIdLo;
  d["lightNum"]   = cfg.lightNum;    d["unknownByte"] = cfg.unknownByte;
  d["txPackets"]  = cfg.txRepeats;   d["txGapMs"] = cfg.txGapMs;
  d["txPower"]    = cfg.txPower;     d["highPower"] = cfg.highPower;
  d["stepOffMs"]  = cfg.stepOffMs;   d["onDwellMs"]  = cfg.onDwellMs;
  d["rapidOffMs"] = cfg.rapidOffMs;  d["rapidMaxMs"] = cfg.rapidMaxMs;
  d["resetWaitMs"] = cfg.resetWaitMs; d["keepSecs"]  = cfg.keepSecs;
  d["syncSettleMs"] = cfg.syncSettleMs;
  d["rxWindowMs"] = cfg.rxWindowMs;
  d["tz"]         = tz_string;       d["wxTopic"] = mqtt_topic;
  d["wxFreq"]     = radio_freq_mhz;  d["wxDev"]   = radio_dev_khz;
  d["wxBw"]       = radio_bw_khz;    d["wxIds"]   = allowed_ids;
  d["calWindDir"] = cal_wind_dir_offset;  d["calPressure"] = cal_pressure_offset;
  d["calSolar"]   = cal_solar_gain;       d["calUv"]       = cal_uv_gain;
  File f = LittleFS.open("/config.json", "w");
  if (f) { serializeJson(d, f); f.close(); }
}

void loadConfig() {
  File f = LittleFS.open("/config.json", "r");
  if (f) {
    JsonDocument d;
    if (!deserializeJson(d, f)) {
      copyStr(cfg.deviceName, sizeof(cfg.deviceName), d["deviceName"] | cfg.deviceName);
      copyStr(cfg.hostname,   sizeof(cfg.hostname),   d["hostname"]   | cfg.hostname);
      copyStr(cfg.deviceId,   sizeof(cfg.deviceId),   d["deviceId"]   | "");
      copyStr(cfg.mqttHost,   sizeof(cfg.mqttHost),   d["mqttHost"]   | "");
      cfg.mqttPort = d["mqttPort"] | cfg.mqttPort;
      copyStr(cfg.mqttUser,   sizeof(cfg.mqttUser),   d["mqttUser"]   | "");
      copyStr(cfg.mqttPass,   sizeof(cfg.mqttPass),   d["mqttPass"]   | "");
      copyStr(cfg.baseTopic,  sizeof(cfg.baseTopic),  d["baseTopic"]  | cfg.baseTopic);
      copyStr(cfg.discPrefix, sizeof(cfg.discPrefix), d["discPrefix"] | cfg.discPrefix);
      cfg.remoteIdHi  = d["remoteIdHi"]  | cfg.remoteIdHi;
      cfg.remoteIdLo  = d["remoteIdLo"]  | cfg.remoteIdLo;
      cfg.lightNum    = d["lightNum"]    | cfg.lightNum;
      cfg.unknownByte = d["unknownByte"] | cfg.unknownByte;
      cfg.txRepeats   = d["txPackets"]   | cfg.txRepeats;
      cfg.txGapMs     = d["txGapMs"]     | cfg.txGapMs;
      cfg.txPower     = d["txPower"]     | cfg.txPower;
      cfg.highPower   = d["highPower"]   | cfg.highPower;
      cfg.stepOffMs   = d["stepOffMs"]   | cfg.stepOffMs;
      cfg.onDwellMs   = d["onDwellMs"]   | cfg.onDwellMs;
      cfg.rapidOffMs  = d["rapidOffMs"]  | cfg.rapidOffMs;
      cfg.rapidMaxMs  = d["rapidMaxMs"]  | cfg.rapidMaxMs;
      cfg.resetWaitMs = d["resetWaitMs"] | cfg.resetWaitMs;
      cfg.syncSettleMs = d["syncSettleMs"] | cfg.syncSettleMs;
      cfg.keepSecs    = d["keepSecs"]    | cfg.keepSecs;
      cfg.rxWindowMs  = d["rxWindowMs"]  | cfg.rxWindowMs;
      copyStr(tz_string,   sizeof(tz_string),   d["tz"]      | tz_string);
      copyStr(mqtt_topic,  sizeof(mqtt_topic),  d["wxTopic"] | mqtt_topic);
      copyStr(allowed_ids, sizeof(allowed_ids), d["wxIds"]   | "");
      radio_freq_mhz      = d["wxFreq"]      | radio_freq_mhz;
      radio_dev_khz       = d["wxDev"]       | radio_dev_khz;
      radio_bw_khz        = d["wxBw"]        | radio_bw_khz;
      cal_wind_dir_offset = d["calWindDir"]  | 0;
      cal_pressure_offset = d["calPressure"] | 0.0f;
      cal_solar_gain      = d["calSolar"]    | 1.0f;
      cal_uv_gain         = d["calUv"]       | 1.0f;
    }
    f.close();
  }
  if (cfg.deviceId[0] == 0) {
    snprintf(cfg.deviceId, sizeof(cfg.deviceId), "poollight_%08x", (unsigned)RANDOM_REG32);
    saveConfig();
  }
  if (cfg.txRepeats < 1) cfg.txRepeats = 1;
}

// ------------------------------------------------------------------
//  Event log
// ------------------------------------------------------------------
const int LOG_SIZE = 30;
String logBuf[LOG_SIZE];
int logHead = 0, logCount = 0;

String timeStamp() {
  time_t now = time(nullptr);
  char b[20];
  if (now > 1700000000) {
    struct tm t; localtime_r(&now, &t);
    strftime(b, sizeof(b), "%H:%M:%S", &t);
  } else {
    unsigned long s = millis() / 1000;
    snprintf(b, sizeof(b), "+%lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60);
  }
  return String(b);
}

void addLog(const String& msg) {
  String line = timeStamp() + "  " + msg;
  Serial.println(line);
  logBuf[logHead] = line;
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

// ------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

volatile bool radioFlag = false;
IRAM_ATTR void onDio0() { radioFlag = true; }

String lastRxJson = "";
uint32_t noisePackets = 0;
uint8_t lastRxBuf[6] = {0};
unsigned long lastRxMs = 0, burstStartMs = 0;
int burstCount = 0;
bool burstOpen = false;

bool stateDirty = true;
unsigned long lastStatePub = 0;
unsigned long lastMqttAttempt = 0;

String topic(const char* suffix) { return String(cfg.baseTopic) + "/" + suffix; }

// ------------------------------------------------------------------
//  Tracked light model
// ------------------------------------------------------------------
bool lightOn = false;
int colourIdx = 0;
bool resetMode = false;           // light is flashing blue (mid-sync)
unsigned long lastToggleMs = 0;   // when power last changed
bool offTimeKnown = false;        // false after boot = treat as "long off"

void saveTrack() {
  JsonDocument d;
  d["on"] = lightOn; d["colour"] = colourIdx;
  File f = LittleFS.open("/state.json", "w");
  if (f) { serializeJson(d, f); f.close(); }
}

void loadTrack() {
  File f = LittleFS.open("/state.json", "r");
  if (!f) return;
  JsonDocument d;
  if (!deserializeJson(d, f)) {
    lightOn = d["on"] | false;
    colourIdx = constrain((int)(d["colour"] | 0), 0, NUM_COLOURS - 1);
  }
  f.close();
}

unsigned long offElapsed() {
  if (lightOn) return 0;
  if (!offTimeKnown) return 0xFFFFFFFFUL;
  return millis() - lastToggleMs;
}

String trackText() {
  if (!lightOn) return "Off (last colour " + String(COLOURS[colourIdx].name) + ")";
  if (resetMode) return "On - SYNC/reset (flashing blue)";
  return String("On - ") + COLOURS[colourIdx].name;
}

// Apply one power toggle to the model (from us or from the real remote).
// Returns a note for the log, e.g. "on after 0.62 s off (rapid)".
String applyToggle() {
  unsigned long now = millis();
  String note;
  if (lightOn) {
    lightOn = false;
    resetMode = false;          // flashing stops; light is now reset to colour 1
    note = "off after " + String((now - lastToggleMs) / 1000.0, 1) + " s on";
  } else {
    unsigned long off = offElapsed();
    lightOn = true;
    if (!offTimeKnown) note = "on (off time unknown - treated as long)";
    else note = "on after " + String(off / 1000.0, 2) + " s off" +
                (off < cfg.rapidMaxMs ? " (rapid)" :
                 off <= (unsigned long)cfg.keepSecs * 1000UL ? " (step)" : " (long - same colour)");
    if (off < cfg.rapidMaxMs) {
      resetMode = true; colourIdx = 0;           // rapid OFF/ON = manual reset (flashing blue)
    } else if (off <= (unsigned long)cfg.keepSecs * 1000UL) {
      colourIdx = (colourIdx + 1) % NUM_COLOURS; // step (after a reset: Blue -> Magenta)
    }                                            // long off: colour kept
  }
  lastToggleMs = now;
  offTimeKnown = true;
  saveTrack();
  stateDirty = true;
  return note;
}

// ------------------------------------------------------------------
//  Protocol / sending
// ------------------------------------------------------------------
uint8_t calcChecksum(uint8_t idHi, uint8_t idLo, uint8_t light, uint8_t cmd, uint8_t unk) {
  int sum = 0x2D + 0xD4 + idHi + idLo + light + cmd + unk;
  return (uint8_t)(0x100 - (sum % 0x100));
}

String hex2(uint8_t v) { char b[3]; snprintf(b, sizeof(b), "%02X", v); return String(b); }

unsigned long lastTxBurstMs = 0;
String paRegs = "not read yet (sends once to check)";
bool paChecked = false;

String paRegText() {
  Module* m = &rfmModule;
  uint8_t pa = m->SPIreadRegister(RADIOLIB_RF69_REG_PA_LEVEL);
  uint8_t t1 = m->SPIreadRegister(RADIOLIB_RF69_REG_TEST_PA1);
  uint8_t t2 = m->SPIreadRegister(RADIOLIB_RF69_REG_TEST_PA2);
  uint8_t ocp = m->SPIreadRegister(RADIOLIB_RF69_REG_OCP);
  String s = "PA_LEVEL 0x" + hex2(pa) + " TEST_PA1 0x" + hex2(t1) + " TEST_PA2 0x" + hex2(t2) +
             " OCP 0x" + hex2(ocp);
  // 20 dBm high-power expects PA_LEVEL 0x7F, TEST_PA1 0x5D, TEST_PA2 0x7C, OCP 0x0F
  bool full = pa == 0x7F && t1 == 0x5D && t2 == 0x7C;
  if (cfg.highPower && cfg.txPower >= 18) s += full ? "  (20 dBm boost ON - correct)" : "  (NOT full power!)";
  return s;
}

// Sends one burst (txRepeats packets) of any 6-byte payload.
void setupRadio();     // resets and re-initialises BOTH radios (shared reset pin)

bool sendBurst(const uint8_t* p, bool retry) {
  if (!radioOk) { addLog("TX skipped - radio not ready"); return false; }
  int state = RADIOLIB_ERR_NONE;
  unsigned long t0 = millis();
  for (int i = 0; i < cfg.txRepeats; i++) {
    state = radio.transmit(p, 6);    // always the 6-byte payload
    if (state != RADIOLIB_ERR_NONE) break;
    if (i + 1 < cfg.txRepeats && cfg.txGapMs) delay(cfg.txGapMs);
  }
  lastTxBurstMs = millis() - t0;
  if (!paChecked && state == RADIOLIB_ERR_NONE) {   // registers as used for TX
    paChecked = true;
    paRegs = paRegText();
    addLog("TX amplifier check: " + paRegs);
  }
  radioFlag = false;          // DIO0 "packet sent" is not a received packet
  radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    addLog("TX FAILED, code " + String(state));
    if (retry) {                       // reset + reconfigure the radio, try once more
      addLog("Re-initialising radio and retrying");
      setupRadio();
      return sendBurst(p, false);
    }
    return false;
  }
  return true;
}

// Change TX power live (no restart) and save it.
void setTxPower(long dbm) {
  cfg.txPower = cfg.highPower ? constrain(dbm, -2L, 20L) : constrain(dbm, -18L, 13L);
  if (radioOk) {
    // twice: RadioLib's RF69 setOutputPower() applies the PREVIOUS value
    radio.setOutputPower(cfg.txPower, cfg.highPower);
    radio.setOutputPower(cfg.txPower, cfg.highPower);
    radio.startReceive();
  }
  saveConfig();
  paChecked = false;                 // re-log amplifier registers on next send
  paRegs = "not read yet (checked on next send)";
  addLog("TX power set to " + String(cfg.txPower) + " dBm");
  stateDirty = true;
}

bool sendCodeB4(uint8_t code, uint8_t b4) {
  uint8_t p[6] = { cfg.remoteIdHi, cfg.remoteIdLo, cfg.lightNum, code, b4,
                   calcChecksum(cfg.remoteIdHi, cfg.remoteIdLo, cfg.lightNum, code, b4) };
  return sendBurst(p, true);
}

bool sendCode(uint8_t code) { return sendCodeB4(code, cfg.unknownByte); }

// RF test: 10 s of bursts every 250 ms with a dummy remote ID (0000) and
// light 0, so the pool light ignores them. For SDR / range comparisons.
unsigned long rfTestUntil = 0, rfTestLast = 0;
void startRfTest() {
  rfTestUntil = millis() + 10000; rfTestLast = 0;
  addLog("RF test: dummy packets for 10 s (light ignores them)");
}
void handleRfTest() {
  if (!rfTestUntil) return;
  if ((long)(millis() - rfTestUntil) >= 0) { rfTestUntil = 0; addLog("RF test finished"); return; }
  if (millis() - rfTestLast < 250) return;
  rfTestLast = millis();
  uint8_t p[6] = {0x00, 0x00, 0x00, CMD_POWER, cfg.unknownByte,
                  calcChecksum(0x00, 0x00, 0x00, CMD_POWER, cfg.unknownByte)};
  sendBurst(p, true);
}

// One press of the power button: send it and update the model.
bool pressPower(const char* why) {
  if (!sendCode(CMD_POWER)) return false;
  String note = applyToggle();
  addLog(String("TX (") + why + "): " + note + " -> " + trackText() + "  [" + String(cfg.txRepeats) +
         " packets, " + String(lastTxBurstMs) + " ms]");
  return true;
}

// ------------------------------------------------------------------
//  Jobs: multi-press sequences run from loop()
// ------------------------------------------------------------------
JobType job = JOB_NONE;
int jobTarget = 0, jobPresses = 0, syncStep = 0;
bool syncWasOn = false; int syncPrevColour = 0;   // restored after a sync
unsigned long jobNextMs = 0;

String jobText() {
  switch (job) {
    case JOB_GOTO: return String("Changing to ") + COLOURS[jobTarget].name;
    case JOB_ON:   return "Turning on";
    case JOB_OFF:  return "Turning off";
    case JOB_SYNC:
      switch (syncStep) {
        case 0:  return "Sync 1/3: light on, settling";
        case 1:  return "Sync 2/3: SYNC sent - should be flashing blue";
        default: return "Sync 3/3: turning off";
      }
    default:       return "Idle";
  }
}

void endJob(const String& msg) {
  if (job == JOB_NONE) return;
  job = JOB_NONE;
  addLog(msg);
  stateDirty = true;
}

// (int, not JobType: keeps the Arduino IDE's auto-generated prototypes happy)
void startJob(int j, int target) {
  if (job != JOB_NONE) endJob("Previous action cancelled");
  job = (JobType)j; jobTarget = target; jobPresses = 0; syncStep = 0;
  syncWasOn = lightOn; syncPrevColour = colourIdx;
  addLog(jobText() + " (currently " + trackText() + ")");
  stateDirty = true;
}

void jobPress() {
  jobPresses++;
  if (!pressPower(jobText().c_str())) endJob("Stopped - transmit failed");
}

void handleJob() {
  if (job == JOB_NONE) return;
  unsigned long now = millis();
  unsigned long sinceToggle = now - lastToggleMs;
  if (jobPresses > NUM_COLOURS * 2 + 6) { endJob("Stopped - too many presses, try Sync"); return; }

  switch (job) {
    case JOB_GOTO:
      if (lightOn && !resetMode && colourIdx == jobTarget) {
        endJob(String("Done: ") + COLOURS[jobTarget].name);
      } else if (lightOn) {
        if (sinceToggle >= cfg.onDwellMs) jobPress();               // OFF
      } else if (offElapsed() >= cfg.stepOffMs) {
        jobPress();                                                  // ON
      }
      break;

    case JOB_ON:
      if (lightOn) endJob("Done: " + trackText());
      else if (offElapsed() >= cfg.stepOffMs) jobPress();           // never a rapid ON
      break;

    case JOB_OFF:
      if (!lightOn) endJob("Done: off");
      else if (sinceToggle >= 300) jobPress();
      break;

    case JOB_SYNC:
      // Uses the remote's own SYNC command (0x13), captured from the remote:
      // 0: make sure the light is on (SYNC is ignored while it's off) and has
      //    been on for syncSettleMs, then send SYNC -> flashing blue, reset to
      //    colour 1 (Electric Blue)
      // 1: after resetWaitMs, power OFF (stops the flashing)
      // Then, if the light was on before the sync, step it back to the colour
      // it was showing (from a reset, the next ON is Magenta, and so on).
      if (syncStep == 0) {
        if (!lightOn) {
          if (offElapsed() >= cfg.stepOffMs) jobPress();          // ON first
        } else if (sinceToggle >= cfg.syncSettleMs) {
          // As captured from the remote: <id hi> <id lo> <light> 13 03 <checksum>
          if (!sendCodeB4(CMD_SYNC, 0x03)) { endJob("Sync stopped - transmit failed"); break; }
          lightOn = true; colourIdx = 0; resetMode = true; saveTrack();
          addLog("TX SYNC -> " + trackText());
          syncStep = 1; jobNextMs = millis() + cfg.resetWaitMs; stateDirty = true;
        }
      } else if ((long)(now - jobNextMs) >= 0) {
        syncStep = 2; stateDirty = true;
        jobPress();                                               // OFF
        if (job == JOB_NONE) break;
        if (syncWasOn) {
          addLog(String("Synchronised - restoring ") + COLOURS[syncPrevColour].name);
          job = JOB_GOTO; jobTarget = syncPrevColour; jobPresses = 0;
        } else {
          endJob("Synchronised - off; comes back on as " + String(COLOURS[1].name) +
                 " within " + String(cfg.keepSecs) + " s, else " + COLOURS[0].name);
        }
      }
      break;

    default: break;
  }
}

// ==================================================================
//  WEATHER (HP1000: Outdoor = WH24, Indoor = WH25)
// ==================================================================
// --------------------------------------------------------------------------------
// DECODING HELPERS
// --------------------------------------------------------------------------------
static uint8_t crc8(const uint8_t *d, int len, uint8_t poly, uint8_t init) {
  uint8_t crc = init;
  for (int i = 0; i < len; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
  }
  return crc;
}

static uint8_t sum8(const uint8_t *d, int len) {
  uint8_t s = 0;
  for (int i = 0; i < len; i++) s += d[i];
  return s;
}

// UV raw -> UV index. Thresholds and comparison verified against rtl_433's
// fineoffset_WH24_callback() (src/devices/fineoffset.c) - the table this
// replaced had different breakpoints and could report the wrong index for
// the same raw reading (e.g. UVI 4 showing up as 10).
static int uvIndexFromRaw(int raw) {
  static const uint16_t upper[] = {432, 851, 1210, 1570, 2017, 2450, 2761, 3100, 3512, 3918, 4277, 4650, 5029};
  int idx = 0;
  while (idx < 13 && upper[idx] < raw) idx++;
  return idx;
}

// Outdoor sensor array (WH24 protocol, 17 bytes):
// FAMILY(0x24) ID [dir,batt,temp] HUM WIND GUST RAIN RAIN UV UV LUX LUX LUX CRC SUM
//
// The last 3 data bytes before the checksum are a 24-bit "light" value in
// units of 0.1 lux. Fine Offset/Ecowitt consoles (including the HP1000)
// convert that to the displayed solar radiation in W/m^2 by dividing the
// lux figure by 126.7 - the standard lux-to-W/m^2 factor for daylight.
static int decodeWH24(const uint8_t *b, int n, Reading &r) {
  if (n < 17 || b[0] != 0x24) return DEC_NONE;
  if (crc8(b, 15, 0x31, 0x00) != b[15] || sum8(b, 16) != b[16]) return DEC_BADCRC;

  int windDir  = b[2] | ((b[3] & 0x80) << 1);
  int lowBatt  = (b[3] & 0x08) >> 3;
  int tempRaw  = ((b[3] & 0x07) << 8) | b[4];
  int hum      = b[5];
  int windRaw  = b[6] | ((b[3] & 0x10) << 4);
  int gustRaw  = b[7];
  int rainRaw  = (b[8] << 8) | b[9];
  int uvRaw    = (b[10] << 8) | b[11];
  uint32_t lightRaw = ((uint32_t)b[12] << 16) | ((uint32_t)b[13] << 8) | b[14];

  r.model    = 24;
  r.id       = b[1];
  r.battOk   = !lowBatt;
  r.temp     = (tempRaw != 0x7FF) ? (tempRaw - 400) * 0.1f : NAN;
  r.hum      = (hum != 0xFF) ? hum : -1;
  r.windDir  = (windDir != 0x1FF) ? windDir : -1;
  r.wind     = (windRaw != 0x1FF) ? windRaw * 0.125f * 1.12f : NAN;
  r.gust     = (gustRaw != 0xFF) ? gustRaw * 1.12f : NAN;
  r.rain     = rainRaw * 0.3f;
  r.uvRaw    = (uvRaw != 0xFFFF) ? uvRaw : -1;
  r.uv       = (uvRaw != 0xFFFF) ? uvIndexFromRaw(uvRaw) : -1;
  r.pressure = NAN;
  r.solarWm2 = (lightRaw != 0xFFFFFF) ? (lightRaw * 0.1f) / 126.7f : NAN;
  return DEC_OK;
}

// Indoor sensor (WH25 protocol): family nibble 0xE, ID, battery, temp, humidity, pressure, sum
static int decodeWH25(const uint8_t *b, int n, Reading &r) {
  if (n < 8 || (b[0] & 0xF0) != 0xE0) return DEC_NONE;
  // Byte 6 is a plain 8-bit sum of bytes 0..5. Byte 7 is a further check byte
  // whose algorithm isn't identified, so it is not used.
  if ((uint8_t)(b[0] + b[1] + b[2] + b[3] + b[4] + b[5]) != b[6]) return DEC_NONE;
  int lowBatt = (b[1] & 0x08) >> 3;
  int tempRaw = ((b[1] & 0x07) << 8) | b[2];
  int pRaw    = (b[4] << 8) | b[5];
  // Sanity limits so noise that happens to pass the 8-bit sum is still rejected
  if (tempRaw > 1100 || b[3] < 1 || b[3] > 100 || pRaw < 8000 || pRaw > 11500) return DEC_NONE;

  r.model    = 25;
  r.id       = ((b[0] & 0x0F) << 4) | ((b[1] & 0xF0) >> 4);
  r.battOk   = !lowBatt;
  r.temp     = (tempRaw - 400) * 0.1f;
  r.hum      = b[3];
  r.windDir  = -1;
  r.wind     = NAN;
  r.gust     = NAN;
  r.rain     = NAN;
  r.uv       = -1;
  r.uvRaw    = -1;
  r.pressure = pRaw * 0.1f;
  r.solarWm2 = NAN;
  return DEC_OK;
}

// Applies the user's calibration settings to a freshly-decoded reading, so
// the offsets/gain live in exactly one place rather than being duplicated
// across the dashboard, /data.json and MQTT.
static void applyCalibration(Reading &r) {
  if (r.windDir >= 0) {
    int wd = (r.windDir + cal_wind_dir_offset) % 360;
    if (wd < 0) wd += 360;
    r.windDir = wd;
  }
  if (!isnan(r.pressure)) {
    r.pressure += cal_pressure_offset;
  }
  if (!isnan(r.solarWm2)) {
    r.solarWm2 *= cal_solar_gain;
  }
  // UV gain scales the RAW sensor value before the index lookup (like a
  // sensitivity correction), so low readings can still step up from 0 to 1.
  if (r.uvRaw >= 0) {
    long scaled = lroundf(r.uvRaw * cal_uv_gain);
    if (scaled < 0) scaled = 0;
    r.uv = uvIndexFromRaw((int)scaled);
  }
}

// --------------------------------------------------------------------------------
// GENERAL HELPERS
// --------------------------------------------------------------------------------
static float r1(float v) { return roundf(v * 10.0f) / 10.0f; }

// 16-point compass bearing for a wind direction in degrees (0-359).
static const char *compassBearing(int deg) {
  static const char *pts[16] = {
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };
  int idx = ((deg % 360) + 360) % 360;          // normalise, just in case
  idx = (int)((idx / 22.5f) + 0.5f) % 16;       // 16 sectors of 22.5 deg each
  return pts[idx];
}

static String htmlEsc(const String &s) {
  String o;
  o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '&':  o += "&amp;";  break;
      case '<':  o += "&lt;";   break;
      case '>':  o += "&gt;";   break;
      case '"':  o += "&quot;"; break;
      case '\'': o += "&#39;";  break;
      default:   o += c;
    }
  }
  return o;
}

static void timeString(char *out, size_t len, bool full) {
  time_t now = time(nullptr);
  if (now > 1700000000) {
    struct tm tm;
    localtime_r(&now, &tm);
    strftime(out, len, full ? "%Y-%m-%d %H:%M:%S" : "%H:%M:%S", &tm);
    return;
  }
  uint32_t s = millis() / 1000;
  snprintf(out, len, "uptime %02u:%02u:%02u", (unsigned)(s / 3600), (unsigned)((s / 60) % 60), (unsigned)(s % 60));
}

static bool idAllowed(uint8_t id) {
  if (strlen(allowed_ids) == 0) return true;
  String list = String(",") + allowed_ids + ",";
  list.replace(" ", "");
  return list.indexOf("," + String((int)id) + ",") >= 0;
}

static void logFrame(const String &line) {
  recentLog[recentHead] = line;
  recentHead = (recentHead + 1) % RECENT_N;
  if (recentCount < RECENT_N) recentCount++;
  Serial.println(line);
}

static const char *modelLabel(uint8_t model) {
  return (model == 24) ? "Outdoor" : "Indoor";
}

static String devKey(const Device &d) {
  return String(d.model == 24 ? "outdoor_" : "indoor_") + String((int)d.id);
}

// One shared Home Assistant device identity for the whole receiver - Outdoor,
// Indoor and the receiver's own diagnostics all group under a single device
// card. This is a fixed name, NOT the board's MAC/chip ID, so any board
// running this sketch shows up as the same Home Assistant device - swap
// boards freely without reconfiguring HA. (Not tied to a sensor ID either,
// so a sensor battery change that gives it a new radio ID doesn't matter.)
// Only change this if you run two separate receivers on the same broker.
#define HP_DEVICE_ID "hp1000_station"

static String hpDeviceIdent() {
  return String(HP_DEVICE_ID);
}

// One MQTT connection = one Last Will, so the weather entities share the
// bridge's availability topic.
static String availTopic() {
  return topic("status");
}

String getRssiDescription(int rssi) {
  if (rssi >= -60) return " (Strong)";
  if (rssi >= -75) return " (Medium)";
  if (rssi >= -85) return " (Weak)";
  return " (Very Weak)";
}

// --------------------------------------------------------------------------------
// MQTT: HOME ASSISTANT DISCOVERY
// --------------------------------------------------------------------------------
static void publishDisc(const char *comp, const String &devIdent, const String &devName, const char *devModel,
                        const String &stateTopic, const char *key, const char *name, const char *tmpl,
                        const char *unit, const char *devClass, const char *stateClass, const char *icon, bool diag) {
  JsonDocument doc;
  String uid = devIdent + "_" + key;

  doc["name"]               = name;
  doc["state_topic"]        = stateTopic;
  doc["availability_topic"] = availTopic();
  doc["unique_id"]          = uid;
  doc["value_template"]     = tmpl;
  if (unit && unit[0])           doc["unit_of_measurement"] = unit;
  if (devClass && devClass[0])   doc["device_class"]        = devClass;
  if (stateClass && stateClass[0]) doc["state_class"]       = stateClass;
  if (icon && icon[0])           doc["icon"]                = icon;
  if (diag)                      doc["entity_category"]     = "diagnostic";

  JsonObject dev = doc["device"].to<JsonObject>();
  dev["identifiers"][0] = devIdent;
  dev["name"]           = devName;
  dev["model"]          = devModel;
  dev["manufacturer"]   = "HP1000 Weather Station";

  String tp = String(cfg.discPrefix) + "/" + String(comp) + "/" + uid + "/config";
  String payload;
  serializeJson(doc, payload);
  mqtt.publish(tp.c_str(), payload.c_str(), true);
}

static void publishDeviceDiscovery(Device &d) {
  String key   = devKey(d);                    // e.g. "outdoor_42" - unique per physical sensor
  String ident = hpDeviceIdent();               // shared across Outdoor, Indoor, Receiver
  String name  = "HP1000 Weather Station";
  String label = String(modelLabel(d.model));   // "Outdoor" / "Indoor" - goes in entity names
  String st    = String(mqtt_topic) + "/" + key + "/state";

  publishDisc("sensor", ident, name, "HP1000", st, (key + "_temperature").c_str(), (label + " Temperature").c_str(),
              "{{ value_json.temperature }}", "\xC2\xB0""C", "temperature", "measurement", "", false);
  publishDisc("sensor", ident, name, "HP1000", st, (key + "_humidity").c_str(), (label + " Humidity").c_str(),
              "{{ value_json.humidity }}", "%", "humidity", "measurement", "", false);

  if (d.model == 24) {
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_wind_dir").c_str(), (label + " Wind Direction").c_str(),
                "{{ value_json.wind_dir }}", "\xC2\xB0", "", "measurement", "mdi:compass-outline", false);
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_wind_bearing").c_str(), (label + " Wind Bearing").c_str(),
                "{{ value_json.wind_bearing }}", "", "", "", "mdi:compass-outline", false);
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_wind_speed").c_str(), (label + " Wind Speed").c_str(),
                "{{ value_json.wind_speed }}", "m/s", "wind_speed", "measurement", "", false);
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_gust_speed").c_str(), (label + " Gust Speed").c_str(),
                "{{ value_json.gust_speed }}", "m/s", "wind_speed", "measurement", "mdi:weather-windy", false);
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_rain").c_str(), (label + " Rainfall Total").c_str(),
                "{{ value_json.rain }}", "mm", "precipitation", "total_increasing", "", false);
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_uv_index").c_str(), (label + " UV Index").c_str(),
                "{{ value_json.uv_index }}", "", "", "measurement", "mdi:sun-wireless-outline", false);
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_solar_radiation").c_str(), (label + " Solar Radiation").c_str(),
                "{{ value_json.solar_radiation }}", "W/m\xC2\xB2", "irradiance", "measurement", "mdi:solar-power", false);
  } else {
    publishDisc("sensor", ident, name, "HP1000", st, (key + "_pressure").c_str(), (label + " Pressure").c_str(),
                "{{ value_json.pressure }}", "hPa", "pressure", "measurement", "", false);
  }

  publishDisc("binary_sensor", ident, name, "HP1000", st, (key + "_battery").c_str(), (label + " Battery").c_str(),
              "{{ 'OFF' if value_json.battery_ok else 'ON' }}", "", "battery", "", "", false);
  publishDisc("sensor", ident, name, "HP1000", st, (key + "_rssi").c_str(), (label + " Signal Strength").c_str(),
              "{{ value_json.rssi }}", "dBm", "signal_strength", "measurement", "", true);

  d.discovered = true;
}

static void publishBridgeDiscovery() {
  String ident = hpDeviceIdent();
  String name  = "HP1000 Weather Station";
  String st    = String(mqtt_topic) + "/bridge/state";

  publishDisc("sensor", ident, name, "HP1000", st, "bridge_ip", "Receiver IP Address",
              "{{ value_json.ip }}", "", "", "", "mdi:ip-network", true);
  publishDisc("sensor", ident, name, "HP1000", st, "bridge_wifi", "Receiver Wi-Fi Signal",
              "{{ value_json.wifi_rssi }}", "dBm", "signal_strength", "measurement", "", true);
  publishDisc("sensor", ident, name, "HP1000", st, "bridge_frames", "Receiver Valid Frames",
              "{{ value_json.frames_valid }}", "", "", "total_increasing", "mdi:counter", true);
  // v1.3 diagnostics
  publishDisc("sensor", ident, name, "HP1000", st, "bridge_reset_reason", "Receiver Restart Reason",
              "{{ value_json.reset_reason }}", "", "", "", "mdi:restart", true);
  publishDisc("sensor", ident, name, "HP1000", st, "bridge_uptime", "Receiver Uptime",
              "{{ value_json.uptime_s }}", "s", "duration", "measurement", "", true);
  publishDisc("sensor", ident, name, "HP1000", st, "bridge_wifi_drops", "Receiver Wi-Fi Drops",
              "{{ value_json.wifi_drops }}", "", "", "measurement", "mdi:wifi-alert", true);
}

// --------------------------------------------------------------------------------
// MQTT: STATE PUBLISHERS
// --------------------------------------------------------------------------------
static void putF(JsonDocument &doc, const char *k, float v) {
  if (isnan(v)) doc[k] = nullptr; else doc[k] = r1(v);
}
static void putI(JsonDocument &doc, const char *k, int v) {
  if (v < 0) doc[k] = nullptr; else doc[k] = v;
}

static void publishDeviceState(const Device &d) {
  if (!mqtt.connected()) return;
  const Reading &r = d.r;

  JsonDocument doc;
  putF(doc, "temperature", r.temp);
  putI(doc, "humidity", r.hum);
  if (d.model == 24) {
    putI(doc, "wind_dir", r.windDir);
    if (r.windDir >= 0) doc["wind_bearing"] = compassBearing(r.windDir);
    putF(doc, "wind_speed", r.wind);
    putF(doc, "gust_speed", r.gust);
    putF(doc, "rain", r.rain);
    putI(doc, "uv_index", r.uv);
    putI(doc, "uv_raw", r.uvRaw);
    putF(doc, "solar_radiation", r.solarWm2);
  } else {
    putF(doc, "pressure", r.pressure);
  }
  doc["battery_ok"] = (bool)r.battOk;
  doc["rssi"]       = d.rssi;
  doc["count"]      = d.count;
  doc["time"]       = d.timeStr;

  String payload;
  serializeJson(doc, payload);
  String tp = String(mqtt_topic) + "/" + devKey(d) + "/state";
  mqtt.publish(tp.c_str(), payload.c_str());
}

static void publishBridgeState() {
  if (!mqtt.connected()) return;
  JsonDocument doc;
  doc["ip"]              = WiFi.localIP().toString();
  doc["wifi_rssi"]       = WiFi.RSSI();
  doc["frames_valid"]    = validFrames;
  doc["frames_crc_fail"] = crcFrames;
  // v1.3 diagnostics
  doc["reset_reason"]     = resetReason;
  doc["uptime_s"]         = millis() / 1000UL;
  doc["wifi_drops"]       = (uint32_t)wifiDrops;
  doc["wifi_last_reason"] = (int)wifiLastReason;
  String payload;
  serializeJson(doc, payload);
  String tp = String(mqtt_topic) + "/bridge/state";
  mqtt.publish(tp.c_str(), payload.c_str());
}

// --------------------------------------------------------------------------------
// FRAME HANDLING
// --------------------------------------------------------------------------------
static void updateDevice(const Reading &r, int rssi) {
  int slot = -1;
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (devs[i].used && devs[i].model == r.model && devs[i].id == r.id) { slot = i; break; }
  }
  if (slot < 0) {
    for (int i = 0; i < MAX_DEVICES; i++) {
      if (!devs[i].used) {
        slot = i;
        devs[i].used = true;
        devs[i].discovered = false;
        devs[i].model = r.model;
        devs[i].id = r.id;
        devs[i].count = 0;
        break;
      }
    }
  }
  if (slot < 0) return;   // table full

  Device &d = devs[slot];
  d.count++;
  d.r = r;
  d.rssi = rssi;
  timeString(d.timeStr, sizeof(d.timeStr), true);

  if (mqtt.connected()) {
    if (!d.discovered) publishDeviceDiscovery(d);
    publishDeviceState(d);
  }
}

static void handleFrame(const uint8_t *b, int n, int rssi) {
  Reading r;
  int res = decodeWH24(b, n, r);
  if (res == DEC_NONE) res = decodeWH25(b, n, r);

  String hex;
  hex.reserve(n * 3);
  for (int i = 0; i < n; i++) {
    char t[4];
    snprintf(t, sizeof(t), "%02X ", b[i]);
    hex += t;
  }

  char ts[24];
  timeString(ts, sizeof(ts), false);
  String line = String(ts) + "  " + String(rssi) + " dBm  ";

  if (res == DEC_OK) {
    applyCalibration(r);
    if (!idAllowed(r.id)) {
      line += "IGNORED (ID " + String((int)r.id) + " not in allowed list)  " + hex;
      logFrame(line);
      return;
    }
    validFrames++;
    line += "OK " + String(modelLabel(r.model)) + " id " + String((int)r.id) + "  " + hex;
    logFrame(line);
    updateDevice(r, rssi);
  } else if (res == DEC_BADCRC) {
    crcFrames++;
    line += "CRC FAIL  " + hex;
    logFrame(line);
  } else {
    noiseFrames++;
    if (rssi > -70) {          // only log noise that is strong enough to be interesting
      line += "noise  " + hex;
      logFrame(line);
    }
  }
}

// ------------------------------------------------------------------
//  Radio setup / receive
// ------------------------------------------------------------------
bool radioStep(const char* name, int state) {
  if (state == RADIOLIB_ERR_NONE) return true;
  radioError = String(name) + " failed, code " + String(state);
  addLog("Radio: " + radioError);
  return false;
}

// Both radios' RESET pins are on PIN_RST (active high).
void resetRadios() {
  pinMode(PIN_NSS, OUTPUT);     digitalWrite(PIN_NSS, HIGH);      // nobody selected
  pinMode(PIN_WX_NSS, OUTPUT);  digitalWrite(PIN_WX_NSS, HIGH);
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, HIGH);  delay(2);
  digitalWrite(PIN_RST, LOW);   delay(10);
}

void setupPoolRadio() {
  // RadioLib 7.7.x bug: begin() returns -25 at its default sync-word step,
  // after the important setup. Everything it skips is set below.
  int st = radio.begin();
  if (st == RADIOLIB_ERR_UNSUPPORTED) st = RADIOLIB_ERR_NONE;
  uint8_t syncWord[] = {0x2D, 0xD4};   // const matters on 7.7.x
  radioOk =
    radioStep("begin",        st) &&
    radioStep("frequency",    radio.setFrequency(RF_FREQUENCY)) &&
    radioStep("bit rate",     radio.setBitRate(RF_BITRATE)) &&
    radioStep("rx bandwidth", radio.setRxBandwidth(RF_RXBW)) &&
    radioStep("deviation",    radio.setFrequencyDeviation(RF_DEVIATION)) &&
    // Twice on purpose: RadioLib's RF69 setOutputPower() writes the PREVIOUS
    // value to the chip; the second call applies the configured power.
    radioStep("tx power",     radio.setOutputPower(cfg.txPower, cfg.highPower)) &&
    radioStep("tx power",     radio.setOutputPower(cfg.txPower, cfg.highPower)) &&
    radioStep("preamble",     radio.setPreambleLength(RF_PREAMBLE_LEN)) &&
    radioStep("sync word",    radio.setSyncWord(syncWord, 2)) &&
    radioStep("shaping",      radio.setDataShaping(RADIOLIB_SHAPING_NONE)) &&
    radioStep("encoding",     radio.setEncoding(RADIOLIB_ENCODING_NRZ)) &&
    radioStep("length",       radio.fixedPacketLengthMode(6)) &&
    radioStep("crc off",      radio.setCrcFiltering(false)) &&
    radioStep("address off",  radio.disableAddressFiltering());
  if (radioOk) {
    radio.setDio0Action(onDio0);
    radio.startReceive();
    addLog("Radio ready, TX " + String(cfg.txPower) + " dBm" +
           (cfg.highPower ? " (high-power module)" : "") + ", RadioLib " +
           String(RADIOLIB_VERSION_MAJOR) + "." + String(RADIOLIB_VERSION_MINOR) + "." +
           String(RADIOLIB_VERSION_PATCH));
  }
}

bool wxStep(const char* name, int state) {
  if (state == RADIOLIB_ERR_NONE) return true;
  wxRadioError = String(name) + " failed, code " + String(state);
  addLog("Weather radio: " + wxRadioError);
  return false;
}

// Nearest receiver bandwidth the RFM69 actually has (same formula RadioLib
// uses; it rejects anything that is not one of these steps).
float snapRxBw(float khz) {
  float best = 200.0f, bestDiff = 1e9f;
  for (int e = 0; e < 8; e++) {
    for (int m = 0; m < 3; m++) {
      float p = (32.0f * 1000000.0f) / (((4 * m) + 16) * ((uint32_t)1 << (e + 2))) / 1000.0f;
      float diff = fabsf(p - khz);
      if (diff < bestDiff) { bestDiff = diff; best = p; }
    }
  }
  return best;
}

// 433 MHz weather receiver. Also called from the Weather Setup page to
// apply new radio settings straight away.
void setupWxRadio() {
  wxRadioError = "";
  radio_bw_khz = snapRxBw(radio_bw_khz);
  int st = wxRadio.begin();
  if (st == RADIOLIB_ERR_UNSUPPORTED) st = RADIOLIB_ERR_NONE;   // RadioLib 7.7.x, see above
  uint8_t syncWord[] = {0x2D, 0xD4};   // Fine Offset sync word
  wxRadioOk =
    wxStep("begin",        st) &&
    wxStep("frequency",    wxRadio.setFrequency(radio_freq_mhz)) &&
    wxStep("bit rate",     wxRadio.setBitRate(WX_BITRATE)) &&
    wxStep("rx bandwidth", wxRadio.setRxBandwidth(radio_bw_khz)) &&
    wxStep("deviation",    wxRadio.setFrequencyDeviation(radio_dev_khz)) &&
    wxStep("sync word",    wxRadio.setSyncWord(syncWord, 2)) &&
    wxStep("shaping",      wxRadio.setDataShaping(RADIOLIB_SHAPING_NONE)) &&
    wxStep("encoding",     wxRadio.setEncoding(RADIOLIB_ENCODING_NRZ)) &&
    wxStep("length",       wxRadio.fixedPacketLengthMode(MAX_PKT)) &&
    wxStep("crc off",      wxRadio.setCrcFiltering(false)) &&
    wxStep("address off",  wxRadio.disableAddressFiltering());
  wxChipVersion = wxModule.SPIreadRegister(RADIOLIB_RF69_REG_VERSION);
  pinMode(PIN_WX_DIO0, INPUT_PULLDOWN_16);   // reads low if the wire is missing
  if (wxRadioOk) {
    wxRadio.startReceive();
    addLog("Weather radio ready, " + String(radio_freq_mhz, 3) + " MHz, BW " +
           String(radio_bw_khz, 1) + " kHz");
  }
}

// Reset both radios (shared reset line) and configure each.
void setupRadio() {
  SPI.begin();
  resetRadios();
  setupPoolRadio();
  setupWxRadio();
}

void publishRx(const uint8_t* b, float rssi) {
  JsonDocument d;
  char id[5]; snprintf(id, sizeof(id), "%02X%02X", b[0], b[1]);
  d["remote_id"] = id;
  d["light"]     = b[2];
  d["code"]      = "0x" + hex2(b[3]);
  d["unknown"]   = "0x" + hex2(b[4]);
  d["rssi"]      = rssi;
  d["time"]      = timeStamp();
  lastRxJson = "";
  serializeJson(d, lastRxJson);
  if (mqtt.connected()) mqtt.publish(topic("rx").c_str(), lastRxJson.c_str(), true);
}

void handlePoolRadio() {
  // Report how many packets the last remote press contained - useful for
  // setting "RF packets per press" to match the real remote.
  if (burstOpen && millis() - lastRxMs > cfg.rxWindowMs) {
    burstOpen = false;
    addLog("  (remote sent " + String(burstCount) + " packet" + (burstCount == 1 ? "" : "s") +
           " over " + String(lastRxMs - burstStartMs) + " ms)");
  }

  if (!radioOk || !radioFlag) return;
  radioFlag = false;
  uint8_t b[6] = {0};
  int st = radio.readData(b, 6);
  float rssi = radio.getRSSI();
  radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) return;
  if (calcChecksum(b[0], b[1], b[2], b[3], b[4]) != b[5]) { noisePackets++; return; }

  // A press is sent as a burst of identical packets - count it once.
  bool dup = memcmp(b, lastRxBuf, 6) == 0 && millis() - lastRxMs < cfg.rxWindowMs;
  memcpy(lastRxBuf, b, 6);
  lastRxMs = millis();
  if (dup) { burstCount++; return; }
  burstOpen = true; burstCount = 1; burstStartMs = lastRxMs;

  char id[5]; snprintf(id, sizeof(id), "%02X%02X", b[0], b[1]);
  bool ours = b[0] == cfg.remoteIdHi && b[1] == cfg.remoteIdLo && b[2] == cfg.lightNum;
  String info = String(id) + " light " + b[2] + " code 0x" + hex2(b[3]) + " byte4 0x" + hex2(b[4]) +
                ", " + String(rssi, 0) + " dBm";
  publishRx(b, rssi);

  if (ours && b[3] == CMD_POWER) {
    if (job != JOB_NONE) endJob("Action cancelled - real remote was used");
    String note = applyToggle();
    addLog("RX remote: " + note + " -> " + trackText() + " [" + info + "]");
  } else if (ours && b[3] == CMD_SYNC) {
    if (job != JOB_NONE) endJob("Action cancelled - real remote was used");
    if (lightOn) {
      colourIdx = 0; resetMode = true; saveTrack(); stateDirty = true;
      addLog("RX remote SYNC -> " + trackText() + " [" + info + "]");
    } else {
      addLog("RX remote SYNC while off - light ignores it [" + info + "]");
    }
  } else {
    addLog("RX other packet [" + info + "]");
  }
}

// Weather receive: DIO0 goes high when a full 17-byte frame is in the FIFO
// and stays high until it is read, so polling it from loop() loses nothing.
void handleWxRadio() {
  if (!wxRadioOk) return;
  if (digitalRead(PIN_WX_DIO0)) {
    int rssi = (int)lroundf(wxRadio.getRSSI());
    uint8_t buf[MAX_PKT] = {0};
    int st = wxRadio.readData(buf, MAX_PKT);
    wxRadio.startReceive();
    if (st == RADIOLIB_ERR_NONE) handleFrame(buf, MAX_PKT, rssi);
    return;
  }
  // Safety: put the radio back into receive if it ever dropped out
  static uint32_t tCheck = 0;
  if (millis() - tCheck > 2000) {
    tCheck = millis();
    if ((wxModule.SPIreadRegister(RADIOLIB_RF69_REG_OP_MODE) & 0x1C) != RADIOLIB_RF69_RX) wxRadio.startReceive();
  }
}

// ------------------------------------------------------------------
//  MQTT + Home Assistant discovery
// ------------------------------------------------------------------
void addDevice(JsonDocument& d) {
  JsonObject dev = d["dev"].to<JsonObject>();
  dev["ids"][0] = cfg.deviceId;
  dev["name"]   = cfg.deviceName;
  dev["mdl"]    = "RFM69 Pool Light Bridge";
  dev["mf"]     = "DIY";
  dev["sw"]     = FW_VERSION;
  dev["cu"]     = "http://" + WiFi.localIP().toString() + "/";
  d["avty_t"]   = topic("status");
}

void pubDiscovery(const char* component, const String& objId, JsonDocument& d) {
  addDevice(d);
  String t = String(cfg.discPrefix) + "/" + component + "/" + cfg.deviceId + "/" + objId + "/config";
  String payload; serializeJson(d, payload);
  mqtt.publish(t.c_str(), (const uint8_t*)payload.c_str(), payload.length(), true);
}

void publishDiscovery() {
  for (auto& e : OLD_ENTITIES) {
    String t = String(cfg.discPrefix) + "/" + e[0] + "/" + cfg.deviceId + "/" + e[1] + "/config";
    mqtt.publish(t.c_str(), "", true);
  }
  {
    JsonDocument d;                                // main light entity
    d["name"] = nullptr;                           // = device name
    d["uniq_id"] = String(cfg.deviceId) + "_light";
    d["cmd_t"] = topic("light/set");
    d["stat_t"] = topic("light/state");
    d["fx_cmd_t"] = topic("light/effect/set");
    d["fx_stat_t"] = topic("light/effect");
    JsonArray fx = d["fx_list"].to<JsonArray>();
    for (int i = 0; i < NUM_COLOURS; i++) fx.add(COLOURS[i].name);
    d["icon"] = "mdi:pool";
    pubDiscovery("light", "light", d);
  }
  {
    JsonDocument d;
    d["name"] = "Sync lights"; d["uniq_id"] = String(cfg.deviceId) + "_sync";
    d["cmd_t"] = topic("sync"); d["pl_prs"] = "sync"; d["icon"] = "mdi:sync";
    pubDiscovery("button", "sync", d);
  }
  {
    JsonDocument d;
    d["name"] = "Power toggle (raw)"; d["uniq_id"] = String(cfg.deviceId) + "_power";
    d["cmd_t"] = topic("command"); d["pl_prs"] = "power"; d["icon"] = "mdi:power";
    d["ent_cat"] = "config";
    pubDiscovery("button", "power", d);
  }
  {
    JsonDocument d;
    d["name"] = "Tracked colour"; d["uniq_id"] = String(cfg.deviceId) + "_track_colour";
    d["cmd_t"] = topic("track/colour/set"); d["stat_t"] = topic("light/effect");
    JsonArray o = d["ops"].to<JsonArray>();
    for (int i = 0; i < NUM_COLOURS; i++) o.add(COLOURS[i].name);
    d["ent_cat"] = "config"; d["icon"] = "mdi:palette";
    pubDiscovery("select", "track_colour", d);
  }
  {
    JsonDocument d;
    d["name"] = "Tracked power"; d["uniq_id"] = String(cfg.deviceId) + "_track_power";
    d["cmd_t"] = topic("track/power/set"); d["stat_t"] = topic("light/state");
    JsonArray o = d["ops"].to<JsonArray>(); o.add("ON"); o.add("OFF");
    d["ent_cat"] = "config"; d["icon"] = "mdi:power-settings";
    pubDiscovery("select", "track_power", d);
  }
  {
    JsonDocument d;
    d["name"] = "Status"; d["uniq_id"] = String(cfg.deviceId) + "_status";
    d["stat_t"] = topic("state"); d["val_tpl"] = "{{ value_json.status }}";
    d["icon"] = "mdi:information-outline";
    pubDiscovery("sensor", "status", d);
  }
  {
    JsonDocument d;
    d["name"] = "Last remote press"; d["uniq_id"] = String(cfg.deviceId) + "_last_rx";
    d["stat_t"] = topic("rx"); d["val_tpl"] = "{{ value_json.time }}";
    d["json_attr_t"] = topic("rx"); d["icon"] = "mdi:remote";
    pubDiscovery("sensor", "last_rx", d);
  }
  {
    JsonDocument d;
    d["name"] = "WiFi signal"; d["uniq_id"] = String(cfg.deviceId) + "_rssi";
    d["stat_t"] = topic("state"); d["val_tpl"] = "{{ value_json.rssi }}";
    d["unit_of_meas"] = "dBm"; d["dev_cla"] = "signal_strength";
    d["stat_cla"] = "measurement"; d["ent_cat"] = "diagnostic";
    pubDiscovery("sensor", "rssi", d);
  }
  {
    JsonDocument d;
    d["name"] = "IP address"; d["uniq_id"] = String(cfg.deviceId) + "_ip";
    d["stat_t"] = topic("state"); d["val_tpl"] = "{{ value_json.ip }}";
    d["ent_cat"] = "diagnostic"; d["icon"] = "mdi:ip-network";
    pubDiscovery("sensor", "ip", d);
  }
  addLog("MQTT discovery published");
}

String statusText() {
  return job != JOB_NONE ? jobText() : (resetMode ? String("Reset mode") : String("Idle"));
}

void publishState() {
  if (!mqtt.connected()) return;
  mqtt.publish(topic("light/state").c_str(), lightOn ? "ON" : "OFF", true);
  mqtt.publish(topic("light/effect").c_str(), COLOURS[colourIdx].name, true);
  JsonDocument d;
  d["power"]  = lightOn ? "ON" : "OFF";
  d["colour"] = COLOURS[colourIdx].name;
  d["status"] = statusText();
  d["rssi"]   = WiFi.RSSI();
  d["ip"]     = WiFi.localIP().toString();
  d["uptime"] = millis() / 1000;
  d["radio"]  = radioOk ? "ok" : radioError;
  d["weather_radio"] = wxRadioOk ? "ok" : wxRadioError;
  String s; serializeJson(d, s);
  mqtt.publish(topic("state").c_str(), s.c_str(), true);
  lastStatePub = millis();
  stateDirty = false;
}

void setTracked(int on, int colour, const char* why) {   // -1 = unchanged
  if (on >= 0 && (bool)on != lightOn) {
    lightOn = on; lastToggleMs = millis(); offTimeKnown = false;   // off time unknown
  }
  if (colour >= 0) colourIdx = colour;
  resetMode = false;
  saveTrack();
  stateDirty = true;
  addLog(String("Tracking corrected (") + why + ") -> " + trackText());
}

void onMqtt(char* t, byte* payload, unsigned int len) {
  String msg; msg.reserve(len);
  for (unsigned int i = 0; i < len; i++) msg += (char)payload[i];
  msg.trim();
  String ts(t);

  if (ts == topic("light/set")) {
    if (msg.equalsIgnoreCase("ON")) {
      if (job == JOB_NONE && !lightOn) startJob(JOB_ON, 0);   // ignore ON sent with an effect
    } else if (msg.equalsIgnoreCase("OFF")) {
      startJob(JOB_OFF, 0);
    }
  } else if (ts == topic("light/effect/set")) {
    int i = colourIndexByName(msg);
    if (i >= 0) startJob(JOB_GOTO, i); else addLog("Unknown colour: " + msg);
  } else if (ts == topic("sync")) {
    startJob(JOB_SYNC, 0);
  } else if (ts == topic("command")) {
    if (job != JOB_NONE) endJob("Action cancelled - raw toggle");
    pressPower("MQTT raw");
  } else if (ts == topic("track/colour/set")) {
    int i = colourIndexByName(msg);
    if (i >= 0) setTracked(-1, i, "MQTT");
  } else if (ts == topic("track/power/set")) {
    setTracked(msg.equalsIgnoreCase("ON") ? 1 : 0, -1, "MQTT");
  }
  stateDirty = true;
}

void handleMqtt() {
  if (cfg.mqttHost[0] == 0) return;
  if (mqtt.connected()) { mqtt.loop(); return; }
  if (millis() - lastMqttAttempt < 5000) return;
  if (job != JOB_NONE) return;      // a connect attempt can block; don't stretch light timing
  lastMqttAttempt = millis();
  mqtt.setServer(cfg.mqttHost, cfg.mqttPort);
  String st = topic("status");
  bool ok = mqtt.connect(cfg.deviceId,
                         cfg.mqttUser[0] ? cfg.mqttUser : nullptr,
                         cfg.mqttUser[0] ? cfg.mqttPass : nullptr,
                         st.c_str(), 0, true, "offline");
  if (ok) {
    mqtt.publish(st.c_str(), "online", true);
    const char* subs[] = {"light/set", "light/effect/set", "sync", "command",
                          "track/colour/set", "track/power/set"};
    for (const char* s : subs) mqtt.subscribe(topic(s).c_str());
    addLog(String("MQTT connected to ") + cfg.mqttHost);
    publishDiscovery();
    publishState();
    // weather: receiver diagnostics + every sensor heard so far
    publishBridgeDiscovery();
    for (int i = 0; i < MAX_DEVICES; i++) {
      if (devs[i].used) publishDeviceDiscovery(devs[i]);
    }
    publishBridgeState();
    lastBridgePub = millis();
  } else {
    addLog("MQTT connect failed, rc=" + String(mqtt.state()));
  }
}

// ------------------------------------------------------------------
//  Web pages
// ------------------------------------------------------------------
const char CSS[] PROGMEM = R"CSS(
<meta name=viewport content="width=device-width,initial-scale=1">
<style>
:root{--bg:#0f1720;--card:#18222e;--fg:#e6edf3;--mut:#8b9bb0;--acc:#2f81f7;--ok:#3fb950;--bad:#f85149}
body{margin:0;background:var(--bg);color:var(--fg);font:15px system-ui,sans-serif}
.wrap{max-width:760px;margin:auto;padding:16px}
h1{font-size:20px;margin:4px 0 14px}h2{font-size:15px;margin:0 0 10px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em}
.card{background:var(--card);border-radius:10px;padding:14px;margin-bottom:14px}
nav a{color:var(--acc);margin-right:14px;text-decoration:none}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(130px,1fr));gap:8px}
button{background:#243244;color:var(--fg);border:1px solid #33465c;border-radius:8px;padding:10px;font-size:14px;cursor:pointer}
button:hover{border-color:var(--acc)}button.pri{background:var(--acc);border-color:var(--acc)}
button.cur{outline:2px solid var(--fg)}
.sw{display:block;height:26px;border-radius:6px;margin-bottom:6px}
.now{display:flex;align-items:center;gap:12px;font-size:18px}.now .sw{width:44px;height:44px;margin:0;border-radius:50%}
.row{display:flex;gap:10px;flex-wrap:wrap}.row>*{flex:1}
table{width:100%;border-collapse:collapse}td{padding:4px 0;vertical-align:top}td:first-child{color:var(--mut);width:40%}
pre{background:#0b1118;border-radius:8px;padding:10px;font-size:12.5px;max-height:340px;overflow:auto;white-space:pre-wrap;margin:0}
input,select{width:100%;box-sizing:border-box;background:#0b1118;color:var(--fg);border:1px solid #33465c;border-radius:6px;padding:8px;font-size:14px}
label{display:block;margin:10px 0 4px;color:var(--mut);font-size:13px}
.ok{color:var(--ok)}.bad{color:var(--bad)}.note{color:var(--mut);font-size:13px}
</style>)CSS";

const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html><head><title>Pool Light</title>%CSS%</head><body><div class=wrap>
<h1 id=title>Pool Light</h1>
<nav><a href="/">Control</a><a href="/weather">Weather</a><a href="/settings">Settings</a><a href="/update">Firmware</a></nav><br>
<div class=card><div class=now><span class=sw id=nsw></span><div><div id=ntxt>-</div><div class=note id=job></div></div></div><br>
<div class=row><button class=pri onclick="api('/api/on')">On</button><button onclick="api('/api/off')">Off</button>
<button onclick="api('/api/sync')">Sync</button></div></div>
<div class=card><h2>Colour</h2><div class=grid id=cols></div>
<p class=note>Steps the light through OFF/ON cycles until it reaches the chosen colour. Presses on the real remote are tracked too.</p></div>
<div class=card><h2>Correct tracking</h2><p class=note>If the light shows something different, set what it actually shows (nothing is sent to the light).</p>
<div class=row><select id=tc></select><select id=tp><option>ON</option><option>OFF</option></select>
<button onclick="api('/api/track?colour='+tc.value+'&power='+tp.value)">Set</button></div></div>
<div class=card><h2>Status</h2><table id=st></table></div>
<div class=card><h2>Log</h2><pre id=log></pre></div>
</div><script>
let built=false;
function api(u){fetch(u,{method:'POST'}).then(load)}
function load(){fetch('/api/status').then(r=>r.json()).then(s=>{
 title.textContent=s.name;
 if(!built){let h='',o='';s.colours.forEach((c,i)=>{h+=`<button id=c${i} onclick="api('/api/colour?i=${i}')"><span class=sw style="background:${c.css}"></span>${i+1}. ${c.name}</button>`;o+=`<option value=${i}>${c.name}</option>`});
  cols.innerHTML=h;tc.innerHTML=o;tc.value=s.colour;tp.value=s.power?'ON':'OFF';built=true}
 s.colours.forEach((c,i)=>document.getElementById('c'+i).classList.toggle('cur',s.power&&i==s.colour));
 nsw.style.background=s.power?(s.reset?'#1f5bff':s.colours[s.colour].css):'#333';
 ntxt.textContent=s.track;job.textContent=s.job=='Idle'?'':s.job+'...';
 const rows=[['915 MHz radio',s.radio=='ok'?'<span class=ok>OK</span>':'<span class=bad>'+s.radio+'</span>'],
  ['MQTT',s.mqtt?'<span class=ok>connected</span>':(s.mqtt_host?'<span class=bad>disconnected</span>':'not configured')],
  ['Last remote press',s.last_rx||'-'],['Noise packets ignored',s.noise],['Remote ID / light',s.remote],
  ['TX power',s.txpower],['WiFi',s.ssid+' ('+s.rssi+' dBm)'],['IP',s.ip],['Uptime',s.uptime],['Last restart',s.restart],['Firmware',s.fw]];
 st.innerHTML=rows.map(r=>`<tr><td>${r[0]}</td><td>${r[1]}</td></tr>`).join('');
 log.textContent=s.log.join('\n');
})}
load();setInterval(load,1500);
</script></body></html>)HTML";

String withCss(const char* page) {
  String s = FPSTR(page);
  s.replace("%CSS%", FPSTR(CSS));
  return s;
}

// Web pages and OTA have no password - keep this on a trusted network.
// Kept as a stub so authentication can be added back in one place.
bool requireAuth() {
  return true;
}

String fmtUptime() {
  unsigned long s = millis() / 1000;
  char b[24]; snprintf(b, sizeof(b), "%lud %02lu:%02lu:%02lu", s / 86400, (s / 3600) % 24, (s / 60) % 60, s % 60);
  return String(b);
}

void handleStatus() {
  JsonDocument d;
  d["name"] = cfg.deviceName; d["fw"] = FW_VERSION;
  d["radio"] = radioOk ? String("ok") : radioError;
  d["mqtt"] = mqtt.connected(); d["mqtt_host"] = cfg.mqttHost;
  d["power"] = lightOn; d["colour"] = colourIdx; d["reset"] = resetMode;
  d["track"] = trackText(); d["job"] = statusText();
  if (lastRxJson.length()) {
    JsonDocument rx; deserializeJson(rx, lastRxJson);
    d["last_rx"] = String((const char*)rx["time"]) + " (remote " + (const char*)rx["remote_id"] +
                   ", " + String((float)rx["rssi"], 0) + " dBm)";
  }
  d["noise"] = noisePackets;
  char rem[32]; snprintf(rem, sizeof(rem), "%02X%02X / %u", cfg.remoteIdHi, cfg.remoteIdLo, cfg.lightNum);
  d["remote"] = rem;
  d["txp"] = cfg.txPower;
  d["txmin"] = cfg.highPower ? -2 : -18;
  d["txmax"] = cfg.highPower ? 20 : 13;
  d["txpower"] = String(cfg.txPower) + " dBm" + (cfg.highPower ? " (high power)" : "") + "<br><small>" + paRegs + "</small>";
  d["ssid"] = WiFi.SSID(); d["rssi"] = WiFi.RSSI(); d["ip"] = WiFi.localIP().toString();
  d["uptime"] = fmtUptime();
  d["restart"] = resetReason;
  JsonArray c = d["colours"].to<JsonArray>();
  for (int i = 0; i < NUM_COLOURS; i++) {
    JsonObject o = c.add<JsonObject>();
    o["name"] = COLOURS[i].name; o["css"] = COLOURS[i].css;
  }
  JsonArray l = d["log"].to<JsonArray>();
  for (int i = 0; i < logCount; i++) l.add(logBuf[(logHead - 1 - i + LOG_SIZE) % LOG_SIZE]);
  String out; serializeJson(d, out);
  server.send(200, "application/json", out);
}

String field(const char* label, const char* name, const String& val, const char* type) {
  return String("<label>") + label + "</label><input type=" + type + " name=" + name +
         " value=\"" + htmlEsc(val) + "\">";
}

String pair(const String& a, const String& b) {
  return "<div class=row><div>" + a + "</div><div>" + b + "</div></div>";
}

void handleSettingsGet() {
  if (!requireAuth()) return;
  char idhex[5]; snprintf(idhex, sizeof(idhex), "%02X%02X", cfg.remoteIdHi, cfg.remoteIdLo);
  String h = F("<!doctype html><html><head><title>Settings</title>%CSS%</head><body><div class=wrap>"
               "<h1>Settings</h1><nav><a href=\"/\">Control</a><a href=\"/weather\">Weather</a><a href=\"/settings\">Settings</a>"
               "<a href=\"/update\">Firmware</a></nav><br><form method=post action=/settings>");
  h.replace("%CSS%", FPSTR(CSS));
  h += F("<div class=card><h2>Device</h2>");
  h += field("Device name (shown in Home Assistant)", "deviceName", cfg.deviceName, "text");
  h += field("Hostname (for OTA / mDNS)", "hostname", cfg.hostname, "text");
  h += field("Timezone (POSIX string, e.g. UTC0, EST5EDT, CET-1CEST,M3.5.0,M10.5.0/3)", "tz", tz_string, "text");
  h += field("Home Assistant device ID (keep the same when swapping boards)", "deviceId", cfg.deviceId, "text");
  h += F("</div>");
  h += F("<div class=card><h2>MQTT (pool light + weather)</h2>");
  h += field("Broker host / IP (blank = MQTT off)", "mqttHost", cfg.mqttHost, "text");
  h += pair(field("Port", "mqttPort", String(cfg.mqttPort), "number"),
            field("Username", "mqttUser", cfg.mqttUser, "text"));
  h += field("Password", "mqttPass", "", "password");
  h += F("<p class=note>Leave the password blank to keep the current value.</p>");
  h += pair(field("Base topic", "baseTopic", cfg.baseTopic, "text"),
            field("Discovery prefix", "discPrefix", cfg.discPrefix, "text")) + "</div>";
  h += F("<div class=card><h2>Pool remote / 915 MHz radio</h2>");
  h += pair(field("Remote ID (hex, 4 digits)", "remoteId", idhex, "text"),
            field("Light # (decimal)", "lightNum", String(cfg.lightNum), "number"));
  h += pair(field("Byte 4 (hex)", "unknownByte", hex2(cfg.unknownByte), "text"),
            field("RF packets per press (remote: 4)", "txRepeats", String(cfg.txRepeats), "number"));
  h += field("Extra gap between packets, ms", "txGapMs", String(cfg.txGapMs), "number");
  h += field("High-power module? 1 = RFM69H/HW/HCW, 0 = RFM69W/CW", "highPower", String(cfg.highPower), "number");
  h += field("Remote repeat window, ms (identical packets within this = one press)", "rxWindowMs", String(cfg.rxWindowMs), "number");
  h += F("<p class=note>Press the real remote and check the log: it shows how many packets one press "
         "sends and over how long; each TX line shows the same for this bridge. Aim for a similar total time.</p></div>");
  h += F("<div class=card><h2>Light timing</h2>");
  h += pair(field("OFF time to step colour (ms)", "stepOffMs", String(cfg.stepOffMs), "number"),
            field("ON time between steps (ms)", "onDwellMs", String(cfg.onDwellMs), "number"));
  h += pair(field("Sync: light ON this long before sending SYNC (ms)", "syncSettleMs", String(cfg.syncSettleMs), "number"),
            field("Sync: flash this long before turning off (ms)", "resetWaitMs", String(cfg.resetWaitMs), "number"));
  h += pair(field("OFF longer than this keeps colour (s)", "keepSecs", String(cfg.keepSecs), "number"),
            field("OFF shorter than this = manual reset (ms)", "rapidMaxMs", String(cfg.rapidMaxMs), "number"));
  h += F("</div><button class=pri type=submit>Save &amp; restart</button></form><br>");
  // Radio power: applied live (no restart), outside the form
  int lo = cfg.highPower ? -2 : -18, hi = cfg.highPower ? 20 : 13;
  h += "<div class=card><h2>915 MHz radio power</h2><label class=note>TX power: <b id=txpv>" + String(cfg.txPower) +
       "</b> dBm (range " + String(lo) + " to " + String(hi) + ")</label>"
       "<div class=row><input type=range id=txp min=" + String(lo) + " max=" + String(hi) +
       " value=" + String(cfg.txPower) + " oninput=\"txpv.textContent=this.value\">"
       "<button type=button onclick=\"fetch('/api/txpower?dbm='+txp.value,{method:'POST'}).then(()=>txst.textContent='Saved: '+txp.value+' dBm')\">Apply</button></div>"
       "<p class=note id=txst>Applies immediately and is saved - no restart needed.</p>"
       "<button type=button onclick=\"fetch('/api/rftest',{method:'POST'});this.textContent='RF test running (10 s)...'\">Start 10 s RF test</button>"
       "<span class=note> dummy packets the light ignores, for SDR comparisons</span></div>";
  h += F("<div class=card><h2>WiFi</h2><p class=note>Clears saved WiFi and restarts into the "
         "<b>PoolWeather-Setup</b> access point.</p>"
         "<form method=post action=/wifireset onsubmit=\"return confirm('Forget WiFi and restart?')\">"
         "<button>Reset WiFi</button></form></div>"
         "<form method=post action=/reboot><button>Restart device</button></form></div></body></html>");
  server.send(200, "text/html", h);
}

uint16_t argU16(const char* n, uint16_t lo, uint16_t hi, uint16_t def) {
  long v = server.arg(n).toInt();
  if (server.arg(n).length() == 0) return def;
  return (uint16_t)constrain(v, (long)lo, (long)hi);
}

void handleSettingsPost() {
  if (!requireAuth()) return;
  auto arg = [](const char* n) { return server.arg(n); };
  copyStr(cfg.deviceName, sizeof(cfg.deviceName), arg("deviceName").c_str());
  copyStr(cfg.hostname,   sizeof(cfg.hostname),   arg("hostname").c_str());
  if (arg("tz").length()) copyStr(tz_string, sizeof(tz_string), arg("tz").c_str());
  if (arg("deviceId").length()) copyStr(cfg.deviceId, sizeof(cfg.deviceId), arg("deviceId").c_str());
  copyStr(cfg.mqttHost, sizeof(cfg.mqttHost), arg("mqttHost").c_str());
  cfg.mqttPort = argU16("mqttPort", 1, 65535, 1883);
  copyStr(cfg.mqttUser, sizeof(cfg.mqttUser), arg("mqttUser").c_str());
  if (arg("mqttPass").length()) copyStr(cfg.mqttPass, sizeof(cfg.mqttPass), arg("mqttPass").c_str());
  if (arg("baseTopic").length()) copyStr(cfg.baseTopic, sizeof(cfg.baseTopic), arg("baseTopic").c_str());
  if (arg("discPrefix").length()) copyStr(cfg.discPrefix, sizeof(cfg.discPrefix), arg("discPrefix").c_str());
  unsigned long rid = strtoul(arg("remoteId").c_str(), nullptr, 16);
  cfg.remoteIdHi = (rid >> 8) & 0xFF; cfg.remoteIdLo = rid & 0xFF;
  cfg.lightNum    = (uint8_t)arg("lightNum").toInt();
  cfg.unknownByte = (uint8_t)strtoul(arg("unknownByte").c_str(), nullptr, 16);
  cfg.txRepeats   = argU16("txRepeats", 1, 20, 4);
  cfg.txGapMs     = argU16("txGapMs", 0, 100, 4);
  cfg.highPower   = argU16("highPower", 0, 1, 1);
  {
    long pw = server.arg("txPower").length() ? server.arg("txPower").toInt() : cfg.txPower;
    cfg.txPower = cfg.highPower ? constrain(pw, -2L, 20L) : constrain(pw, -18L, 13L);
  }
  cfg.rxWindowMs  = argU16("rxWindowMs", 50, 2000, 400);
  cfg.stepOffMs   = argU16("stepOffMs", 1000, 20000, 2500);
  cfg.onDwellMs   = argU16("onDwellMs", 300, 20000, 2000);
  cfg.rapidMaxMs  = argU16("rapidMaxMs", 200, 5000, 1000);
  cfg.resetWaitMs = argU16("resetWaitMs", 500, 20000, 3000);
  cfg.syncSettleMs = argU16("syncSettleMs", 0, 30000, 3000);
  cfg.keepSecs    = argU16("keepSecs", 5, 3600, 60);
  saveConfig();
  server.send(200, "text/html", F("<meta http-equiv=refresh content='8;url=/'>Saved. Restarting..."));
  delay(500);
  ESP.restart();
}

// --------------------------------------------------------------------------------
// WEB PAGE BUILDERS
// --------------------------------------------------------------------------------
String getHeader(String title) {
  String html;
  html.reserve(1800);
  html += F("<!DOCTYPE html><html><head>"
            "<meta charset='UTF-8'>"
            "<meta name='viewport' content='width=device-width, initial-scale=1'>");
  html += "<title>" + title + "</title>";
  html += F("<style>"
            "body{font-family:Arial,sans-serif;margin:0;padding:0;background:#f4f4f4;}"
            ".header-banner{background:#0066cc;color:#ffffff;padding:15px 20px;text-align:center;box-shadow:0 2px 4px rgba(0,0,0,0.1);}"
            ".header-banner h1{margin:0;font-size:24px;font-weight:bold;letter-spacing:0.5px;}"
            ".container{max-width:650px;background:#fff;padding:20px;margin:20px auto;border-radius:8px;box-shadow:0 0 10px rgba(0,0,0,0.1);}"
            "h2{color:#333;margin-top:15px;} nav a{margin-right:15px;text-decoration:none;color:#0066cc;font-weight:bold;}"
            "table{width:100%;border-collapse:collapse;margin-top:15px;} td,th{padding:8px;border-bottom:1px solid #ddd;text-align:left;}"
            "input[type='text'],input[type='number'],input[type='password'],select{width:100%;padding:8px;margin:4px 0 12px;box-sizing:border-box;}"
            "input[type='submit'],.btn{background:#28a745;color:#fff;border:none;padding:10px 15px;border-radius:4px;cursor:pointer;text-decoration:none;display:inline-block;}"
            ".btn-danger{background:#dc3545;color:#fff;border:none;padding:10px 15px;border-radius:4px;cursor:pointer;}"
            ".payload-box{background:#1e1e1e;color:#00ff66;font-family:monospace;padding:12px;border-radius:4px;word-break:break-all;white-space:pre-wrap;font-size:12px;margin-top:10px;}"
            ".disconnected{color:#dc3545;font-weight:bold;}"
            ".fault-active{color:#dc3545;font-weight:bold;}"
            ".fault-ok{color:#28a745;font-weight:bold;}"
            ".hint{color:#666;font-size:13px;margin:-8px 0 12px;}"
            ".card-danger{background:#fff3f3;border:1px solid #ffcdd2;padding:15px;border-radius:6px;margin-top:20px;}"
            "</style></head><body>");

  html += F("<div class='header-banner'><h1>HP1000 Weather Station</h1></div>");
  html += F("<div class='container'>");
  html += F("<nav><a href='/weather'>Weather</a><a href='/weather/setup'>Weather Setup</a><a href='/weather/calibration'>Calibration</a><a href='/'>Pool Light</a><a href='/settings'>Settings</a></nav><hr>");
  return html;
}

String getFooter() {
  return F("</div></body></html>");
}

void redirectHome() {
  server.sendHeader("Location", "/weather", true);
  server.send(302, "text/plain", "");
}

// --------------------------------------------------------------------------------
// LIVE-UPDATE JAVASCRIPT
// --------------------------------------------------------------------------------
static const char LIVE_JS[] PROGMEM = R"rawliteral(
<script>
function setText(id,t){var e=document.getElementById(id);if(e)e.textContent=t;}
function row(k,v){return(v==null||v==='')?'':'<tr><th>'+k+'</th><td>'+v+'</td></tr>';}
function num(v,d,u){return(v==null)?null:Number(v).toFixed(d)+(u?' '+u:'');}
async function refreshStatus(){
  try{
    const res=await fetch('/weather/data.json');
    if(!res.ok)return;
    const d=await res.json();
    setText('validVal',d.valid);
    setText('crcVal',d.crcfail);
    setText('noiseVal',d.noise);
    setText('rssiVal',d.rssi+' dBm'+d.rssiDesc);
    var m=document.getElementById('mqttVal');
    if(m){
      if(d.mqtt){m.textContent='Connected';m.className='';}
      else{m.textContent='Disconnected';m.className='disconnected';}
    }
    var h='';
    if(!d.devices.length){
      h='<p>No valid frames received yet. The outdoor sensor sends about every 16 s and the indoor sensor about every 48 s.</p>';
    }
    d.devices.forEach(function(x){
      h+='<h2>HP1000 '+(x.model==24?'Outdoor':'Indoor')+' &ndash; ID '+x.id+'</h2><table>';
      h+=row('Last Message',x.time)+row('N mes.',x.count);
      h+=row('Battery',x.batt?'OK':'<span class="fault-active">LOW</span>');
      h+=row('Temperature',num(x.temp,1,'\u00B0C'));
      h+=row('Humidity',x.hum==null?null:x.hum+' %');
      h+=row('Wind direction',x.wd==null?null:x.wd+'\u00B0'+(x.wb?(' ('+x.wb+')'):''));
      h+=row('Wind speed',num(x.ws,1,'m/s'));
      h+=row('Gust speed',num(x.gs,1,'m/s'));
      h+=row('Rainfall',num(x.rain,1,'mm'));
      h+=row('UV index',x.uv==null?null:(x.uv+(x.uvr==null?'':' (raw '+x.uvr+')')));
      h+=row('Solar Radiation',num(x.solar,1,'W/m\u00B2'));
      h+=row('Pressure',num(x.pr,1,'hPa'));
      h+=row('Signal',x.rssi+' dBm');
      h+='</table>';
    });
    document.getElementById('devices').innerHTML=h;
    document.getElementById('frameLog').textContent=d.recent.length?d.recent.join('\n'):'No frames yet';
  }catch(e){}
}
window.addEventListener('load',refreshStatus);
setInterval(refreshStatus,2000);
</script>
)rawliteral";

// --------------------------------------------------------------------------------
// STATUS PAGE
// --------------------------------------------------------------------------------
void wxStatusPage() {
  String html;
  html.reserve(3600);
  html += getHeader("HP1000 Weather Station");

  html += F("<h2>Weather Sensors</h2><div id='devices'><p>Waiting for data...</p></div>");

  html += F("<h2>Receiver Status</h2><table>");
  html += "<tr><th>WiFi SSID</th><td>" + htmlEsc(WiFi.SSID()) + "</td></tr>";
  html += "<tr><th>WiFi IP</th><td>" + WiFi.localIP().toString() + "</td></tr>";
  int rssi = WiFi.RSSI();
  html += "<tr><th>WiFi RSSI Signal</th><td id='rssiVal'>" + String(rssi) + " dBm" + getRssiDescription(rssi) + "</td></tr>";

  bool mqttOk = mqtt.connected();
  html += "<tr><th>MQTT Connection</th><td id='mqttVal' class='" + String(mqttOk ? "" : "disconnected") + "'>" + String(mqttOk ? "Connected" : "Disconnected") + "</td></tr>";
  html += "<tr><th>Uptime</th><td>" + String(millis() / 3600000UL) + " h " + String((millis() / 60000UL) % 60) + " min</td></tr>";
  html += "<tr><th>Last Restart Reason</th><td>" + String(resetReason) + "</td></tr>";
  html += "<tr><th>Wi-Fi Drops Since Boot</th><td>" + String((uint32_t)wifiDrops) + " (last reason code " + String((int)wifiLastReason) + ")</td></tr>";
  html += "<tr><th>Radio</th><td>" + String(radio_freq_mhz, 3) + " MHz, dev " + String(radio_dev_khz, 1) + " kHz, BW " + String(radio_bw_khz, 1) + " kHz</td></tr>";
  String chipText = (wxChipVersion != 0x24)
                      ? String("<span class='disconnected'>NOT DETECTED - check wiring (read 0x") + String(wxChipVersion, HEX) + ")</span>"
                      : (wxRadioOk ? String("OK (version 0x24)")
                                   : "<span class='disconnected'>" + htmlEsc(wxRadioError) + "</span>");
  html += "<tr><th>433 MHz RFM69</th><td>" + chipText + "</td></tr>";
  html += "<tr><th>Valid Frames</th><td id='validVal'>" + String(validFrames) + "</td></tr>";
  html += "<tr><th>CRC Failures</th><td id='crcVal'>" + String(crcFrames) + "</td></tr>";
  html += "<tr><th>Noise Triggers</th><td id='noiseVal'>" + String(noiseFrames) + "</td></tr>";
  html += F("</table>");

  html += F("<h2>Recent Frames</h2><div class='payload-box' id='frameLog'>Loading...</div>");

  html += FPSTR(LIVE_JS);
  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

// --------------------------------------------------------------------------------
// LIVE DATA ENDPOINT
// --------------------------------------------------------------------------------
void wxData() {
  int rssi = WiFi.RSSI();
  JsonDocument doc;
  doc["valid"]    = validFrames;
  doc["crcfail"]  = crcFrames;
  doc["noise"]    = noiseFrames;
  doc["rssi"]     = rssi;
  doc["rssiDesc"] = getRssiDescription(rssi);
  doc["mqtt"]     = mqtt.connected();
  doc["ip"]       = WiFi.localIP().toString();

  JsonArray arr = doc["devices"].to<JsonArray>();
  for (int i = 0; i < MAX_DEVICES; i++) {
    if (!devs[i].used) continue;
    const Reading &r = devs[i].r;
    JsonObject o = arr.add<JsonObject>();
    o["model"] = devs[i].model;
    o["id"]    = devs[i].id;
    o["count"] = devs[i].count;
    o["time"]  = devs[i].timeStr;
    o["rssi"]  = devs[i].rssi;
    o["batt"]  = (bool)r.battOk;
    if (!isnan(r.temp))     o["temp"] = r1(r.temp);
    if (r.hum >= 0)         o["hum"]  = r.hum;
    if (r.windDir >= 0)     o["wd"]   = r.windDir;
    if (r.windDir >= 0)     o["wb"]   = compassBearing(r.windDir);
    if (!isnan(r.wind))     o["ws"]   = r1(r.wind);
    if (!isnan(r.gust))     o["gs"]   = r1(r.gust);
    if (!isnan(r.rain))     o["rain"] = r1(r.rain);
    if (r.uv >= 0)          o["uv"]   = r.uv;
    if (r.uvRaw >= 0)       o["uvr"]  = r.uvRaw;
    if (!isnan(r.solarWm2)) o["solar"] = r1(r.solarWm2);
    if (!isnan(r.pressure)) o["pr"]   = r1(r.pressure);
  }

  JsonArray rec = doc["recent"].to<JsonArray>();
  for (int i = 0; i < recentCount; i++) {
    int idx = (recentHead - 1 - i + RECENT_N * 2) % RECENT_N;
    rec.add(recentLog[idx]);
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json; charset=utf-8", out);
}

// --------------------------------------------------------------------------------
// WEATHER SETUP (433 MHz radio, allowed IDs, topic prefix)
// --------------------------------------------------------------------------------
void wxSetupPage() {
  String html;
  html.reserve(3200);
  html += getHeader("Weather Setup");
  html += F("<h2>Weather Radio (433 MHz RFM69)</h2><form action='/weather/setup' method='POST'>");

  html += "Centre Frequency (MHz):<br><input type='number' step='0.001' min='424' max='510' name='radio_freq' value='" + String(radio_freq_mhz, 3) + "'>";
  html += F("<div class='hint'>Midpoint of the two FSK peaks seen in an SDR. 433.906 works for the Outdoor/Indoor sensors here.</div>");

  html += "FSK Deviation (kHz):<br><input type='number' step='0.1' min='1' max='200' name='radio_dev' value='" + String(radio_dev_khz, 1) + "'>";
  html += F("<div class='hint'>Half the spacing between the two FSK peaks (about 57 kHz for 112 kHz spacing).</div>");

  html += "Receiver Bandwidth (kHz):<br><input type='number' step='0.1' min='50' max='500' name='radio_bw' value='" + String(radio_bw_khz, 1) + "'>";
  html += F("<div class='hint'>Must cover both peaks plus drift. Wider lets in more noise. Snaps to the nearest RFM69 step: 100, 125, 166.7, 200, 250, 333.3, 400, 500.</div>");

  html += "Allowed Device IDs:<br><input type='text' name='allowed_ids' placeholder='e.g. 42,117' value='" + htmlEsc(String(allowed_ids)) + "'>";
  html += F("<div class='hint'>Comma separated. Leave blank to accept every sensor, including neighbours' ones.</div>");

  html += "Weather MQTT Topic Prefix:<br><input type='text' name='mqtt_topic' value='" + htmlEsc(String(mqtt_topic)) + "'>";
  html += F("<div class='hint'>Each sensor publishes to &lt;prefix&gt;/outdoor_42/state etc. The broker itself and the timezone are on the main <a href='/settings'>Settings</a> page.</div>");

  html += F("<input type='submit' value='Save &amp; Apply'></form>");
  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void wxSetupSave() {
  if (server.hasArg("radio_freq") && server.arg("radio_freq").length() > 0)
    radio_freq_mhz = constrain(server.arg("radio_freq").toFloat(), 424.0f, 510.0f);
  if (server.hasArg("radio_dev") && server.arg("radio_dev").length() > 0)
    radio_dev_khz = constrain(server.arg("radio_dev").toFloat(), 1.0f, 200.0f);
  if (server.hasArg("radio_bw") && server.arg("radio_bw").length() > 0)
    radio_bw_khz = snapRxBw(constrain(server.arg("radio_bw").toFloat(), 50.0f, 500.0f));
  if (server.hasArg("allowed_ids")) copyStr(allowed_ids, sizeof(allowed_ids), server.arg("allowed_ids").c_str());
  bool topicChanged = false;
  if (server.hasArg("mqtt_topic") && server.arg("mqtt_topic").length() > 0) {
    topicChanged = server.arg("mqtt_topic") != mqtt_topic;
    copyStr(mqtt_topic, sizeof(mqtt_topic), server.arg("mqtt_topic").c_str());
  }

  saveConfig();
  setupWxRadio();       // apply new radio settings immediately
  if (topicChanged && mqtt.connected()) {       // point Home Assistant at the new topics
    publishBridgeDiscovery();
    for (int i = 0; i < MAX_DEVICES; i++) {
      if (devs[i].used) publishDeviceDiscovery(devs[i]);
    }
    publishBridgeState();
  }
  redirectHome();
}

// --------------------------------------------------------------------------------
// CALIBRATION SETUP HANDLERS
// --------------------------------------------------------------------------------
void wxCalibrationPage() {
  String html;
  html.reserve(2400);
  html += getHeader("Calibration");
  html += F("<h2>Sensor Calibration</h2>");
  html += F("<p class='hint'>These adjust decoded readings to match your station's own display. They apply to new frames only, not to past MQTT history.</p>");
  html += F("<form action='/weather/calibration' method='POST'>");

  html += "Wind Direction Offset (&deg;):<br><input type='number' step='1' min='-359' max='359' name='cal_wind_dir' value='" + String(cal_wind_dir_offset) + "'>";
  html += F("<div class='hint'>Added to the decoded bearing, then wrapped to 0-359&deg;. Use 180 to correct a vane mounted reversed.</div>");

  html += "Pressure Offset (hPa):<br><input type='number' step='0.1' name='cal_pressure' value='" + String(cal_pressure_offset, 1) + "'>";
  html += F("<div class='hint'>Added to the Indoor sensor's raw (absolute) pressure. Set this so the result matches your console's relative/sea-level figure.</div>");

  html += "Solar Radiation Gain:<br><input type='number' step='0.001' min='0' name='cal_solar_gain' value='" + String(cal_solar_gain, 3) + "'>";
  html += F("<div class='hint'>Multiplies the computed W/m&sup2; value. 1.0 = no change; e.g. 1.244 to match a console reading ~24% higher.</div>");

  html += "UV Index Gain:<br><input type='number' step='0.01' min='0' name='cal_uv' value='" + String(cal_uv_gain, 2) + "'>";
  html += F("<div class='hint'>Multiplies the raw UV sensor value (shown on the status page) before it is converted to a UV index. 1.0 = no change. Raw above 432 = UVI 1, above 851 = 2, 1210 = 3, 1570 = 4, 2017 = 5, 2450 = 6, 2761 = 7, 3100 = 8, 3512 = 9, 3918 = 10.</div>");

  html += F("<input type='submit' value='Save'></form>");
  html += getFooter();
  server.send(200, "text/html; charset=utf-8", html);
}

void wxCalibrationSave() {
  if (server.hasArg("cal_wind_dir") && server.arg("cal_wind_dir").length() > 0)
    cal_wind_dir_offset = server.arg("cal_wind_dir").toInt();
  if (server.hasArg("cal_pressure") && server.arg("cal_pressure").length() > 0)
    cal_pressure_offset = server.arg("cal_pressure").toFloat();
  if (server.hasArg("cal_solar_gain") && server.arg("cal_solar_gain").length() > 0)
    cal_solar_gain = constrain(server.arg("cal_solar_gain").toFloat(), 0.0f, 100.0f);
  if (server.hasArg("cal_uv") && server.arg("cal_uv").length() > 0)
    cal_uv_gain = constrain(server.arg("cal_uv").toFloat(), 0.0f, 20.0f);

  saveConfig();
  redirectHome();
}

void setupWeb() {
  server.on("/", HTTP_GET, [] { server.send(200, "text/html", withCss(INDEX_HTML)); });
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/on", HTTP_POST, [] { startJob(JOB_ON, 0); server.send(200, "text/plain", "OK"); });
  server.on("/api/off", HTTP_POST, [] { startJob(JOB_OFF, 0); server.send(200, "text/plain", "OK"); });
  server.on("/api/sync", HTTP_POST, [] { startJob(JOB_SYNC, 0); server.send(200, "text/plain", "OK"); });
  server.on("/api/rftest", HTTP_POST, [] { startRfTest(); server.send(200, "text/plain", "OK"); });
  server.on("/api/txpower", HTTP_POST, [] {
    if (!server.hasArg("dbm")) { server.send(400, "text/plain", "dbm missing"); return; }
    setTxPower(server.arg("dbm").toInt());
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/colour", HTTP_POST, [] {
    int i = server.arg("i").toInt();
    if (i < 0 || i >= NUM_COLOURS) { server.send(400, "text/plain", "Bad colour"); return; }
    startJob(JOB_GOTO, i);
    server.send(200, "text/plain", "OK");
  });
  server.on("/api/track", HTTP_POST, [] {
    int i = server.arg("colour").toInt();
    if (job != JOB_NONE) endJob("Action cancelled - tracking corrected");
    setTracked(server.arg("power") == "ON" ? 1 : 0, (i >= 0 && i < NUM_COLOURS) ? i : -1, "web");
    server.send(200, "text/plain", "OK");
  });
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/reboot", HTTP_POST, [] {
    if (!requireAuth()) return;
    server.send(200, "text/html", F("<meta http-equiv=refresh content='8;url=/'>Restarting..."));
    delay(500); ESP.restart();
  });
  server.on("/wifireset", HTTP_POST, [] {
    if (!requireAuth()) return;
    server.send(200, "text/html", F("WiFi cleared. Connect to the PoolWeather-Setup access point."));
    delay(500);
    WiFiManager wm; wm.resetSettings();
    ESP.restart();
  });
  // weather pages
  server.on("/weather", HTTP_GET, wxStatusPage);
  server.on("/weather/data.json", HTTP_GET, wxData);
  server.on("/weather/setup", HTTP_GET, wxSetupPage);
  server.on("/weather/setup", HTTP_POST, wxSetupSave);
  server.on("/weather/calibration", HTTP_GET, wxCalibrationPage);
  server.on("/weather/calibration", HTTP_POST, wxCalibrationSave);
  server.onNotFound([] { server.send(404, "text/plain", "Not found"); });
  httpUpdater.setup(&server, "/update");      // no password
  server.begin();
}

// ------------------------------------------------------------------
//  WiFi / OTA
// ------------------------------------------------------------------
bool wmSaveNeeded = false;

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.hostname(cfg.hostname);

  WiFiManager wm;
  WiFiManagerParameter pHost("mqtt_host", "MQTT broker host / IP", cfg.mqttHost, sizeof(cfg.mqttHost) - 1);
  char portStr[6]; snprintf(portStr, sizeof(portStr), "%u", cfg.mqttPort);
  WiFiManagerParameter pPort("mqtt_port", "MQTT port", portStr, 5);
  WiFiManagerParameter pUser("mqtt_user", "MQTT username", cfg.mqttUser, sizeof(cfg.mqttUser) - 1);
  WiFiManagerParameter pPass("mqtt_pass", "MQTT password", cfg.mqttPass, sizeof(cfg.mqttPass) - 1);
  wm.addParameter(&pHost); wm.addParameter(&pPort);
  wm.addParameter(&pUser); wm.addParameter(&pPass);
  wm.setSaveParamsCallback([] { wmSaveNeeded = true; });
  wm.setSaveConfigCallback([] { wmSaveNeeded = true; });
  wm.setConfigPortalTimeout(180);

  addLog("Connecting to WiFi...");
  if (!wm.autoConnect("PoolWeather-Setup")) {
    Serial.println("WiFi portal timed out - restarting");
    delay(1000);
    ESP.restart();
  }
  if (wmSaveNeeded) {
    copyStr(cfg.mqttHost, sizeof(cfg.mqttHost), pHost.getValue());
    cfg.mqttPort = atoi(pPort.getValue()) > 0 ? atoi(pPort.getValue()) : 1883;
    copyStr(cfg.mqttUser, sizeof(cfg.mqttUser), pUser.getValue());
    copyStr(cfg.mqttPass, sizeof(cfg.mqttPass), pPass.getValue());
    saveConfig();
  }
  addLog("WiFi connected: " + WiFi.SSID() + "  IP " + WiFi.localIP().toString());
}

void setupOTA() {
  ArduinoOTA.setHostname(cfg.hostname);
  ArduinoOTA.onStart([] { job = JOB_NONE; otaInProgress = true; Serial.println("OTA start"); });
  ArduinoOTA.onError([](ota_error_t e) { otaInProgress = false; Serial.printf("OTA error %u\n", e); });
  ArduinoOTA.begin();   // also starts mDNS: http://<hostname>.local/
}

// ------------------------------------------------------------------
//  Self-recovery (ported from the weather sketch v1.3)
// ------------------------------------------------------------------
WiFiEventHandler evGotIp, evDisconnected;

void setupWifiEvents() {
  wifiSeenUp = true;
  evGotIp = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP&) { wifiSeenUp = true; });
  evDisconnected = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& e) {
    // the event repeats on every failed retry - count each outage once
    if (wifiSeenUp) { wifiSeenUp = false; wifiDrops = wifiDrops + 1; }
    wifiLastReason = e.reason;
  });
  WiFi.setAutoReconnect(true);
}

// Returns true while WiFi is up. While it is down: force a reconnect every
// 30 s and restart after 10 minutes. Both radios keep working meanwhile.
bool wifiMaintain() {
  static unsigned long downSince = 0, retry = 0;    // downSince 0 = WiFi is up
  if (WiFi.status() != WL_CONNECTED) {
    mqttDownSince = 0;
    if (downSince == 0) {
      downSince = millis() | 1;
      retry = millis();
      addLog("WiFi connection lost");
    }
    if (!otaInProgress) {
      if (millis() - downSince >= WIFI_RESTART_MS && job == JOB_NONE) {
        Serial.println(F("WiFi offline for 10 minutes - restarting"));
        delay(200);
        ESP.restart();
      }
      if (millis() - retry >= WIFI_RETRY_MS) {
        retry = millis();
        Serial.println(F("WiFi still offline - forcing reconnect"));
        WiFi.reconnect();
      }
    }
    return false;
  }
  if (downSince != 0) {
    addLog("WiFi back online after " + String((millis() - downSince) / 1000UL) + " s");
    downSince = 0;
  }
  return true;
}

// WiFi says it is connected but the broker has been unreachable for 15
// minutes -> restart (covers a "looks connected but isn't" WiFi state).
void mqttWatchdog() {
  if (cfg.mqttHost[0] == 0 || mqtt.connected()) { mqttDownSince = 0; return; }
  if (mqttDownSince == 0) { mqttDownSince = millis() | 1; return; }
  if (!otaInProgress && job == JOB_NONE && millis() - mqttDownSince >= MQTT_RESTART_MS) {
    Serial.println(F("MQTT unreachable for 15 minutes - restarting"));
    delay(200);
    ESP.restart();
  }
}

// ------------------------------------------------------------------
void setup() {
  // Deselect both radios before anything touches the SPI bus
  pinMode(PIN_NSS, OUTPUT);     digitalWrite(PIN_NSS, HIGH);
  pinMode(PIN_WX_NSS, OUTPUT);  digitalWrite(PIN_WX_NSS, HIGH);

  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("Pool Light + HP1000 Weather Bridge v" FW_VERSION);
  resetReason = ESP.getResetReason();
  Serial.println("Last restart reason: " + resetReason);

  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed - formatting");
    LittleFS.format(); LittleFS.begin();
  }
  loadConfig();
  loadTrack();
  addLog("Tracked state: " + trackText());

  setupRadio();
  setupWiFi();
  setupWifiEvents();
  configTime(tz_string, "pool.ntp.org", "time.google.com");
  setupOTA();
  setupWeb();

  wifiClient.setTimeout(2000);       // don't let an unreachable broker stall loop() for long
  mqtt.setBufferSize(1280);
  mqtt.setSocketTimeout(2);
  mqtt.setCallback(onMqtt);

  addLog("Web page: http://" + WiFi.localIP().toString() + "/  or http://" + cfg.hostname + ".local/");
}

void loop() {
  ArduinoOTA.handle();
  server.handleClient();
  MDNS.update();
  handlePoolRadio();
  handleWxRadio();
  handleJob();
  handleRfTest();
  if (!wifiMaintain()) return;       // offline: radios and light jobs still run above
  handleMqtt();
  mqttWatchdog();
  if (stateDirty || millis() - lastStatePub > 60000UL) publishState();
  if (mqtt.connected() && millis() - lastBridgePub > 60000UL) {
    lastBridgePub = millis();
    publishBridgeState();
  }
}
