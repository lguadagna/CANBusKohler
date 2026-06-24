// BMS CAN Dashboard - Version 2.26
// Authors: LisaG + Grok (xAI)
// Project: Power Systems - BMS CAN Dashboard

#include <AsyncUDP.h>
#include "driver/twai.h"
#include "WiFi.h"
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "AsyncWebSocket.h"
#include "ArduinoJson.h"
#include "esp_wifi.h"
#include <Preferences.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <time.h>
#include <algorithm> // for min() 

// ====================== SIMULATION MODE ======================
#define SIMULATION_MODE false    // ← CHANGE TO false when connected to real CAN bus

bool ledState = false;
Preferences preferences;

// NTP Settings
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.google.com";
const char* ntpServer3 = "time.nist.gov";
const long  gmtOffset_sec = -25200;
const int   daylightOffset_sec = 3600;
bool ntpSynced = false;
unsigned long lastNTPSyncAttempt = 0;

unsigned long lastSimUpdate = 0;
unsigned long lastDebugLog = 0;

// Boot button
unsigned long bootButtonPressTime = 0;
const unsigned long BOOT_ERASE_HOLD_TIME = 3000;

const char* apSSID = "ESP32";
const char* apPassword = "12345678";
String homeSSID = "";
String homePass = "";
String deviceSerial = "";
uint8_t configNumModules = 8;

IPAddress apIP(192, 168, 4, 1);
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ====================== LOGGING ======================
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 15 * 60 * 1000UL;  // 15 min
const int MAX_LOG_ENTRIES = 80;
bool logFullCells = true;   // default: full logging with cells


// BMS Data
struct BMSData {
  float cellVoltages[64];
  float moduleVoltages[8];
  float temperatures[16];
  float packVoltage;
  float maxTemp;
  float minTemp;
  float cellSpread;
  float moduleSpread;
  int worstModuleIndex;
  float worstModuleInternalSpread;
  float globalMaxCell;
  int globalMaxModule;
  int globalMaxCellIndex;
  float globalMinCell;
  int globalMinModule;
  int globalMinCellIndex;
  unsigned long lastUpdate;
  bool nodeStatus[8];
} bmsData;

unsigned long moduleLastSeen[8] = {0};   // tracks last time we received data for each module

const char* moduleNames[] = {"AA", "AB", "AC", "AD", "AE", "AF", "B0", "B1" };

struct RawMessage {
  uint32_t id;
  String description;
  String dataHex;
  unsigned long lastSeen;
  uint32_t count;
};
RawMessage knownRaw[30];
int knownRawCount = 0;
RawMessage unknownRaw[40];
int unknownRawCount = 0;

String f4History[12];
int f4HistoryIndex = 0;
unsigned long f4LastSeen[12];

// Forward declarations
String getBMSDataJSON();
String getDashboardHTML();
String getConfigHTML();
String getHistoryHTML();
String getGraphsHTML();
String getReferenceHTML();
void setupWebServer();
void procesCANMessage(twai_message_t& msg);
void broadcastUpdate();
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
void syncRealTime();
void setupTWAI();
void setupWiFi();
void calculatePackStats();
void trackKnownMessage(uint32_t id, const char* description, twai_message_t& msg);
void logUnknownMessage(twai_message_t& msg);
int getModuleIndex(uint8_t nodeId);
void trimLogFileIfNeeded();
void logCellVoltages();
void simulateBMSData();

void setup() {
  Serial.begin(115200);
  delay(2000);

  uint64_t chipid = ESP.getEfuseMac();
  uint8_t last_byte = (uint8_t)chipid;
  char unique_suffix[3];
  snprintf(unique_suffix, sizeof(unique_suffix), "%02X", last_byte);
  char full_ssid[32];
  snprintf(full_ssid, sizeof(full_ssid), "%s-%s", apSSID, unique_suffix);
  deviceSerial = String(full_ssid);
  // Logging mode: true = full cells, false = light (smaller logs)
  bool logFullCells = preferences.getBool("log_full_cells", true);
  logFullCells = preferences.getBool("log_full_cells", true);
  Serial.printf("📝 Logging mode: %s\n", logFullCells ? "FULL (with cells)" : "LIGHT");

  pinMode(GPIO_NUM_2, OUTPUT);
  digitalWrite(GPIO_NUM_2, ledState);
  pinMode(0, INPUT_PULLUP);

  delay(100);
  if (digitalRead(0) == LOW) {
    Serial.println("BOOT button held — ERASING ALL CONFIGS!");
    for (int i = 0; i < 10; i++) {
      digitalWrite(GPIO_NUM_2, !digitalRead(GPIO_NUM_2));
      delay(150);
    }
    preferences.begin("wifi-config", false);
    preferences.clear();
    preferences.end();
    delay(1000);
    ESP.restart();
  }

  memset(&bmsData, 0, sizeof(bmsData));
  memset(knownRaw, 0, sizeof(knownRaw));
  memset(unknownRaw, 0, sizeof(unknownRaw));
  memset(f4History, 0, sizeof(f4History));
  memset(f4LastSeen, 0, sizeof(f4LastSeen));

  Serial.println("=== BMS CAN Dashboard v2.26 Starting ===");
  if (!LittleFS.begin(true)) Serial.println("❌ LittleFS mount failed");
  else Serial.println("✅ LittleFS mounted");

  setupTWAI();
  setupWiFi();
  WiFi.onEvent(WiFiEvent);
  syncRealTime();
  setupWebServer();

  if (SIMULATION_MODE) {
    Serial.println("🚀 SIMULATION MODE ENABLED - Fake BMS data active");
    simulateBMSData();
  }

  Serial.println("✅ Dashboard ready!");
  Serial.printf("Access at http://%s or http://%s\n", WiFi.localIP().toString().c_str(), deviceSerial.c_str());
}

void loop() {
  if (SIMULATION_MODE) {
    simulateBMSData();
  }

  twai_message_t rx_message;
  uint8_t processed = 0;
  while (twai_receive(&rx_message, 0) == ESP_OK && processed < 30) {
    procesCANMessage(rx_message);
    processed++;
    yield();
  }

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate >= 5000) {
    broadcastUpdate();
    lastUpdate = millis();
    ledState = !ledState;
    digitalWrite(GPIO_NUM_2, ledState);
  }

  if (digitalRead(0) == LOW) {
    if (bootButtonPressTime == 0) bootButtonPressTime = millis();
    if (millis() - bootButtonPressTime >= BOOT_ERASE_HOLD_TIME) {
      preferences.begin("wifi-config", false);
      preferences.clear();
      preferences.end();
      delay(1000);
      ESP.restart();
    }
  } else {
    bootButtonPressTime = 0;
  }

  ws.cleanupClients();
  logCellVoltages();
}

// ====================== SIMULATION ======================
void simulateBMSData() {
  if (millis() - lastSimUpdate < 8000) return;
  lastSimUpdate = millis();

  static float baseV = 3.280;
  baseV += random(-12, 13) / 10000.0;

  // === CRITICAL: Populate BOTH arrays consistently ===
  for (int m = 0; m < 8; m++) {
    for (int c = 0; c < 8; c++) {
      int idx = m * 8 + c;
      float variation = sin(millis() / 9000.0 + idx) * 0.028 + (random(-12, 13) / 1000.0);
      bmsData.cellVoltages[idx] = constrain(baseV + variation, 3.10, 3.42);
    }
    
    // Module voltage = EXACT SUM of 8 cells (matches your logging)
    bmsData.moduleVoltages[m] = 0.0;
    for (int c = 0; c < 8; c++) {
      bmsData.moduleVoltages[m] += bmsData.cellVoltages[m*8 + c];
    }
    // NO division by 8 — keep it as sum (~26V)
  }

  // Temperatures & status (unchanged)
  bmsData.temperatures[0] = 74.0 + sin(millis()/14000.0) * 5.0;
  bmsData.temperatures[1] = 69.5 + cos(millis()/11000.0) * 4.0;
  for (int i = 2; i < 16; i++) bmsData.temperatures[i] = bmsData.temperatures[i%2];
  for (int i = 0; i < 8; i++) bmsData.nodeStatus[i] = true;

  calculatePackStats();   // This will now see consistent data
}

// ====================== NTP ======================
void syncRealTime() {
  lastNTPSyncAttempt = millis();
  if (WiFi.status() != WL_CONNECTED) {
    ntpSynced = false;
    return;
  }
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2, ntpServer3);
  struct tm timeinfo;
  int attempts = 0;
  while (attempts < 20) {
    if (getLocalTime(&timeinfo, 2000)) {
      ntpSynced = true;
      return;
    }
    attempts++;
    delay(1000);
  }
  ntpSynced = false;
}

// helper function 
int countValidCells() {
  int count = 0;
  for (int i = 0; i < 64; i++) {
    if (bmsData.cellVoltages[i] > 0.5) count++;
  }
  return count;
}

// ====================== LOGGING ======================

void logCellVoltages() {
  if (millis() - lastLogTime < LOG_INTERVAL) {
    if (millis() - lastDebugLog > 5000) {
      lastDebugLog = millis();
    }
    return;
  }

  int validCells = countValidCells();
  if (validCells < 8) {
    lastLogTime = millis();
    return;
  }

  time_t now = 0;
  time(&now);
  if (now < 1700000000) {
    lastLogTime = millis();
    return;
  }

  File file = LittleFS.open("/cell_log.jsonl", "a");
  if (!file) return;

  DynamicJsonDocument doc(2300);
  doc["ts"] = (long long)now;

  doc["packV"] = round(bmsData.packVoltage * 100) / 100.0;
  doc["spread"] = round(bmsData.cellSpread);
  doc["moduleSpread"] = round(bmsData.moduleSpread);

  JsonArray modVs = doc.createNestedArray("moduleV");
  for (int m = 0; m < 8; m++) modVs.add(round(bmsData.moduleVoltages[m] * 100) / 100.0);

  // NEW: Conditional full cells logging
  if (logFullCells) {
    JsonArray cells = doc.createNestedArray("cells");
    for (int i = 0; i < 64; i++) {
      cells.add(round(bmsData.cellVoltages[i] * 1000) / 1000.0);
    }
  }

  serializeJson(doc, file);
  file.println();
  file.close();

  lastLogTime = millis();
  trimLogFileIfNeeded();

  Serial.printf("[LOG] Saved %s mode entry | Valid cells: %d\n", 
                logFullCells ? "FULL" : "LIGHT", validCells);
}

void trimLogFileIfNeeded() {
  File file = LittleFS.open("/cell_log.jsonl", "r");
  if (!file) return;
  int lineCount = 0;
  while (file.available()) if (file.read() == '\n') lineCount++;
  file.close();

  if (lineCount <= MAX_LOG_ENTRIES) return;

  File readFile = LittleFS.open("/cell_log.jsonl", "r");
  File tempFile = LittleFS.open("/temp_log.jsonl", "w");
  int skip = lineCount - MAX_LOG_ENTRIES;
  String line;
  while (readFile.available()) {
    line = readFile.readStringUntil('\n');
    if (line.length() > 10) {
      if (skip > 0) skip--;
      else tempFile.println(line);
    }
  }
  readFile.close();
  tempFile.close();
  LittleFS.remove("/cell_log.jsonl");
  LittleFS.rename("/temp_log.jsonl", "/cell_log.jsonl");
}

// ====================== TRIM TO LAST N ENTRIES ======================
void trimToLastN(int keepCount) {
  File file = LittleFS.open("/cell_log.jsonl", "r");
  if (!file) {
    Serial.println("[TRIM] No log file found");
    return;
  }

  // Count total lines
  int totalLines = 0;
  String line;
  while (file.available()) {
    line = file.readStringUntil('\n');
    if (line.length() > 40) totalLines++;
  }
  file.close();

  if (totalLines <= keepCount) {
    Serial.printf("[TRIM] Already only %d entries (<= %d), nothing to do\n", totalLines, keepCount);
    return;
  }

  int skip = totalLines - keepCount;

  File readFile = LittleFS.open("/cell_log.jsonl", "r");
  File tempFile = LittleFS.open("/temp_log.jsonl", "w");

  int processed = 0;
  while (readFile.available()) {
    line = readFile.readStringUntil('\n');
    if (line.length() > 40) {
      processed++;
      if (processed > skip) {
        tempFile.println(line);
      }
    }
  }
  readFile.close();
  tempFile.close();

  LittleFS.remove("/cell_log.jsonl");
  LittleFS.rename("/temp_log.jsonl", "/cell_log.jsonl");

  Serial.printf("[TRIM] ✅ Kept last %d of %d entries (deleted %d old ones)\n", 
                keepCount, totalLines, totalLines - keepCount);
}

// ====================== WiFi ======================
void setupWiFi() {
  preferences.begin("wifi-config", false);
  homeSSID = preferences.getString("ssid", "");
  homePass = preferences.getString("pass", "");
  configNumModules = preferences.getUChar("modules", 8);

  uint64_t chipid = ESP.getEfuseMac();
  uint8_t last_byte = (uint8_t)chipid;
  char unique_suffix[3];
  snprintf(unique_suffix, sizeof(unique_suffix), "%02X", last_byte);
  char full_ssid[32];
  snprintf(full_ssid, sizeof(full_ssid), "%s-%s", apSSID, unique_suffix);

  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(full_ssid, apPassword);
  if (MDNS.begin("ESP32")) Serial.println("mDNS started");

  if (homeSSID.length() > 0) {
    WiFi.mode(WIFI_STA);
    bool useDHCP = preferences.getBool("use_dhcp", false);
    if (!useDHCP) {
      uint8_t octet = preferences.getUChar("static_octet", 100);
      IPAddress staticIP(192, 168, 1, octet);
      IPAddress gateway(192, 168, 1, 1);
      IPAddress subnet(255, 255, 255, 0);
      IPAddress primaryDNS(8, 8, 8, 8);
      IPAddress secondaryDNS(8, 8, 4, 4);
      WiFi.config(staticIP, gateway, subnet, primaryDNS, secondaryDNS);
    }
    WiFi.begin(homeSSID.c_str(), homePass.c_str());
    uint16_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
      delay(1000);
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) Serial.println("\n✅ Connected to home WiFi");
  }
}

void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    Serial.println("WiFi disconnected - reconnecting...");
    WiFi.begin(homeSSID.c_str(), homePass.c_str());
  }
}

// ====================== TWAI ======================
void setupTWAI() {
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_general_config_t g_config = {
    .mode = TWAI_MODE_LISTEN_ONLY,
    .tx_io = GPIO_NUM_17,
    .rx_io = GPIO_NUM_16,
    .tx_queue_len = 5,
    .rx_queue_len = 30
  };
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) twai_start();
}

// ====================== CAN ======================
void trackKnownMessage(uint32_t id, const char* description, twai_message_t& msg) {
  String dataHex = "";
  for (int i = 0; i < msg.data_length_code; i++) {
    char buf[3];
    sprintf(buf, "%02X", msg.data[i]);
    dataHex += buf;
    if (i < msg.data_length_code-1) dataHex += " ";
  }
  for (int i = 0; i < knownRawCount; i++) {
    if (knownRaw[i].id == id) {
      knownRaw[i].count++;
      knownRaw[i].lastSeen = millis();
      knownRaw[i].dataHex = dataHex;
      return;
    }
  }
  if (knownRawCount < 30) {
    knownRaw[knownRawCount].id = id;
    knownRaw[knownRawCount].description = description;
    knownRaw[knownRawCount].dataHex = dataHex;
    knownRaw[knownRawCount].count = 1;
    knownRaw[knownRawCount].lastSeen = millis();
    knownRawCount++;
  }
}

void logUnknownMessage(twai_message_t& msg) {
  String dataHex = "";
  for (int i = 0; i < msg.data_length_code; i++) {
    char buf[3];
    sprintf(buf, "%02X", msg.data[i]);
    dataHex += buf;
    if (i < msg.data_length_code-1) dataHex += " ";
  }
  for (int i = 0; i < unknownRawCount; i++) {
    if (unknownRaw[i].id == msg.identifier) {
      unknownRaw[i].count++;
      unknownRaw[i].lastSeen = millis();
      unknownRaw[i].dataHex = dataHex;
      return;
    }
  }
  if (unknownRawCount < 40) {
    unknownRaw[unknownRawCount].id = msg.identifier;
    unknownRaw[unknownRawCount].description = "Unknown";
    unknownRaw[unknownRawCount].dataHex = dataHex;
    unknownRaw[unknownRawCount].count = 1;
    unknownRaw[unknownRawCount].lastSeen = millis();
    unknownRawCount++;
  }
}

int getModuleIndex(uint8_t nodeId) {
  switch(nodeId) {
    case 0xAA: return 0; case 0xAB: return 1; case 0xAC: return 2;
    case 0xAD: return 3; case 0xAE: return 4; case 0xAF: return 5;
    case 0xB0: return 6; case 0xB1: return 7; default: return -1;
  }
}

void procesCANMessage(twai_message_t& msg) {
  uint32_t id = msg.identifier;
  bool isKnown = false;

  if ((id & 0xFFFFFF00) == 0x18001300) {
    isKnown = true;
    trackKnownMessage(id, "Voltage Data (Cells 1-4)", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      bmsData.nodeStatus[module] = true;
      for (int i = 0; i < 4; i++) {
        uint16_t v = (msg.data[i*2+1] << 8) | msg.data[i*2];
        bmsData.cellVoltages[module*8 + i] = v / 1000.0;
      }
      moduleLastSeen[module] = millis();
    }
  }
  else if ((id & 0xFFFFFF00) == 0x18011300) {
    isKnown = true;
    trackKnownMessage(id, "Voltage Data (Cells 5-8)", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      for (int i = 0; i < 4; i++) {
        uint16_t v = (msg.data[i*2+1] << 8) | msg.data[i*2];
        bmsData.cellVoltages[module*8 + i + 4] = v / 1000.0;
      }
      moduleLastSeen[module] = millis();
    }
  }
  else if ((id & 0xFFFFFF00) == 0x18001400) {
    isKnown = true;
    trackKnownMessage(id, "Temperature Data", msg);
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t t1 = (msg.data[1] << 8) | msg.data[0];
      uint16_t t2 = (msg.data[3] << 8) | msg.data[2];
      bmsData.temperatures[module*2] = t1 / 10.0;
      bmsData.temperatures[module*2 + 1] = t2 / 10.0;
    }
  }



  if (id == 0x180011F4) {
    String dataHex = "";
    for (int i = 0; i < msg.data_length_code; i++) {
      char buf[3];
      sprintf(buf, "%02X", msg.data[i]);
      dataHex += buf;
      if (i < msg.data_length_code-1) dataHex += " ";
    }
    f4History[f4HistoryIndex] = dataHex;
    f4LastSeen[f4HistoryIndex] = millis();
    f4HistoryIndex = (f4HistoryIndex + 1) % 12;
  }

  if (!isKnown) logUnknownMessage(msg);
  calculatePackStats();
}

// ====================== CALCULATE STATS (with 5-min stale tolerance) ======================

void calculatePackStats() {
  unsigned long now = millis();

  // ---------- Cell Pack Stats ----------
  float globalMax = 0.0;
  float globalMin = 5.0;
  int maxIdx = -1, minIdx = -1;
  float total = 0.0;
  int validCells = 0;

  for (int i = 0; i < 64; i++) {
    float v = bmsData.cellVoltages[i];
    if (v > 0.01) {
      total += v;
      validCells++;
      if (v > globalMax) { globalMax = v; maxIdx = i; }
      if (v < globalMin) { globalMin = v; minIdx = i; }
    }
  }

  bmsData.packVoltage = (validCells > 0) ? total : 0.0;
  bmsData.cellSpread = (globalMax - globalMin) * 1000.0;
  bmsData.globalMaxCell = globalMax;
  bmsData.globalMinCell = globalMin;
  bmsData.globalMaxModule = (maxIdx >= 0) ? (maxIdx / 8) : -1;
  bmsData.globalMaxCellIndex = (maxIdx >= 0) ? (maxIdx % 8) + 1 : -1;
  bmsData.globalMinModule = (minIdx >= 0) ? (minIdx / 8) : -1;
  bmsData.globalMinCellIndex = (minIdx >= 0) ? (minIdx % 8) + 1 : -1;

  // ---------- Module Voltages ----------
  float maxModuleV = 0.0;
  float minModuleV = 999.0;
  bmsData.worstModuleIndex = 0;
  bmsData.worstModuleInternalSpread = 0.0;

  for (int m = 0; m < configNumModules; m++) {
    float sum = 0.0;
    int count = 0;
    float moduleMax = 0.0;
    float moduleMin = 5.0;

    for (int c = 0; c < 8; c++) {
      float v = bmsData.cellVoltages[m*8 + c];
      if (v > 0.01) {
        sum += v;
        count++;
        if (v > moduleMax) moduleMax = v;
        if (v < moduleMin) moduleMin = v;
      }
    }

    bool isRecent = (moduleLastSeen[m] > 0 && (now - moduleLastSeen[m] < 300000UL)); // 5 min

    if (count >= 4 || isRecent) {
      bmsData.moduleVoltages[m] = sum;   // sum of 8 cells
      if (sum > maxModuleV) maxModuleV = sum;
      if (sum < minModuleV) minModuleV = sum;

      float internalSpread = (moduleMax - moduleMin) * 1000.0;
      if (internalSpread > bmsData.worstModuleInternalSpread) {
        bmsData.worstModuleInternalSpread = internalSpread;
        bmsData.worstModuleIndex = m;
      }
    } else {
      bmsData.moduleVoltages[m] = 0.0;
    }
  }

  bmsData.moduleSpread = (maxModuleV > 0 && minModuleV < 999) ? (maxModuleV - minModuleV) * 1000.0 : 0.0;

  // ---------- Temperatures (Fixed) ----------
  float maxT = -50.0;
  float minT = 999.0;
  int validTemps = 0;

  for (int i = 0; i < 16; i++) {
    float t = bmsData.temperatures[i];
    if (t > -40 && t < 200) {        // reasonable range filter
      validTemps++;
      if (t > maxT) maxT = t;
      if (t < minT) minT = t;
    }
  }

  bmsData.maxTemp = (validTemps > 0) ? maxT : 0.0;
  bmsData.minTemp = (validTemps > 0) ? minT : 0.0;

  bmsData.lastUpdate = now;
}

String getBMSDataJSON() {
  DynamicJsonDocument doc(4096);
  doc["packVoltage"] = bmsData.packVoltage;
  doc["cellSpread"] = bmsData.cellSpread;
  doc["moduleSpread"] = bmsData.moduleSpread;
  doc["worstModuleIndex"] = bmsData.worstModuleIndex;
  doc["worstModuleInternalSpread"] = bmsData.worstModuleInternalSpread;
  doc["globalMaxCell"] = bmsData.globalMaxCell;
  doc["globalMinCell"] = bmsData.globalMinCell;
  doc["globalMaxModule"] = bmsData.globalMaxModule;
  doc["globalMaxCellIndex"] = bmsData.globalMaxCellIndex;
  doc["globalMinModule"] = bmsData.globalMinModule;
  doc["globalMinCellIndex"] = bmsData.globalMinCellIndex;
  doc["maxTemp"] = bmsData.maxTemp;
  doc["minTemp"] = bmsData.minTemp;
  doc["lastUpdate"] = millis();
  doc["ntpSynced"] = ntpSynced;

  JsonArray modules = doc.createNestedArray("modules");
  for (int m = 0; m < configNumModules; m++) {
    JsonObject mod = modules.createNestedObject();
    mod["name"] = moduleNames[m];
    mod["connected"] = bmsData.nodeStatus[m];
    mod["voltage"] = bmsData.moduleVoltages[m];
    JsonArray cells = mod.createNestedArray("cells");
    for (int c = 0; c < 8; c++) cells.add(bmsData.cellVoltages[m*8 + c]);
    JsonArray temps = mod.createNestedArray("temperatures");
    temps.add(bmsData.temperatures[m*2]);
    temps.add(bmsData.temperatures[m*2 + 1]);
  }

  JsonArray known = doc.createNestedArray("knownMessages");
  for (int i = 0; i < knownRawCount; i++) {
    JsonObject msg = known.createNestedObject();
    msg["id"] = String(knownRaw[i].id, HEX);
    msg["desc"] = knownRaw[i].description;
    msg["data"] = knownRaw[i].dataHex;
    msg["count"] = knownRaw[i].count;
    msg["age"] = (millis() - knownRaw[i].lastSeen) / 1000;
  }

  JsonArray unknown = doc.createNestedArray("unknownMessages");
  for (int i = 0; i < unknownRawCount && i < 20; i++) {
    JsonObject msg = unknown.createNestedObject();
    msg["id"] = String(unknownRaw[i].id, HEX);
    msg["data"] = unknownRaw[i].dataHex;
    msg["count"] = unknownRaw[i].count;
    msg["age"] = (millis() - unknownRaw[i].lastSeen) / 1000;
  }

  JsonArray f4 = doc.createNestedArray("f4History");
  for (int i = 0; i < 12; i++) {
    int idx = (f4HistoryIndex + i) % 12;
    if (f4History[idx].length() > 0) {
      JsonObject entry = f4.createNestedObject();
      entry["data"] = f4History[idx];
      entry["age"] = (millis() - f4LastSeen[idx]) / 1000;
    }
  }

  String output;
  serializeJson(doc, output);
  return output;
}

void broadcastUpdate() {
  ws.textAll(getBMSDataJSON());
}

// ====================== HTML ======================
String getDashboardHTML() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>BMS Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta charset="UTF-8">
    <style>
        body {font-family: Arial, sans-serif; margin: 20px; background: #f0f0f0;}
        .header {background: #2c3e50; color: white; padding: 15px 20px; border-radius: 8px; margin-bottom: 15px; display:flex; justify-content:space-between; align-items:center;}
        .tab {overflow: hidden; border-bottom: 1px solid #ccc;}
        .tab button {background: inherit; float: left; border: none; outline: none; cursor: pointer; padding: 14px 16px; transition: 0.3s; font-size: 16px;}
        .tab button:hover {background: #ddd;}
        .tab button.active {background: #ccc; font-weight: bold;}
        .tabcontent {display: none; padding: 20px 0;}
        .stat {background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); text-align: center; margin:5px;}
        .stat-value {font-size: 24px; font-weight: bold;}
        .modules {display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; margin-bottom: 30px;}
        .module {background: white; padding: 15px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); position: relative;}
        .worst-dot {position: absolute; top: 12px; right: 12px; width: 16px; height: 16px; background: #e74c3c; border-radius: 50%; box-shadow: 0 0 10px #e74c3c;}
        .module-header {font-size: 18px; font-weight: bold; margin-bottom: 10px; display: flex; justify-content: space-between; align-items: center;}
        .module-voltage {font-size: 16px; color: #2c3e50; font-weight: bold; margin-bottom: 8px; text-align: center; background: #ecf0f1; padding: 8px; border-radius: 4px;}
        .status-dot {width: 12px; height: 12px; border-radius: 50%; background: #e74c3c;}
        .status-dot.connected {background: #27ae60}
        .cells {display: grid; grid-template-columns: repeat(4, 1fr); gap: 5px; margin-bottom: 10px;}
        .cell {padding: 8px; text-align: center; border-radius: 4px; font-size: 12px; color: white; background: #95a5a6;}
        .cell.good { background: #27ae60; }
        .cell.warning { background: #f39c12; }
        .cell.danger { background: #e74c3c; }
        .cell.max { background: #2ecc71 !important; color: black !important; font-weight: bold; box-shadow: 0 0 8px #27ae60; }
        .cell.min { background: #e67e22 !important; color: black !important; font-weight: bold; box-shadow: 0 0 8px #e67e22; }
        .temps {font-size: 14px; color: #7f8c8d;}
        .ntp-banner {padding:8px 16px; border-radius:30px; font-size:0.9em;}
        .spread-green {color:#27ae60; font-weight:bold;}
        .spread-yellow {color:#f39c12; font-weight:bold;}
        .spread-red {color:#e74c3c; font-weight:bold;}
    </style>
</head>
<body>
    <div class="header">
        <h1>BMS Dashboard - )rawliteral";
  html += deviceSerial;
  html += R"rawliteral( v2.26</h1>
        <div class="ntp-banner" style="background:#607d8b;color:white;">NTP: )rawliteral";
  html += (ntpSynced ? "✅ Synced" : "⚠️ Not synced");
  html += R"rawliteral(</div>
    <!-- LOGGING MODE INDICATOR -->
            <div style="background:#34495e; color:white; padding:6px 12px; border-radius:20px; font-size:0.9em;">
              Log: <strong>)rawliteral";
      html += (logFullCells ? "FULL" : "LIGHT");
      html += R"rawliteral(</strong>
            </div>


        <button onclick="window.location.href='/config'" style="width:56px;height:56px;background:#607d8b;color:white;border:none;border-radius:50%;font-size:26px;cursor:pointer;">⚙️</button>
    </div>
    <div class="tab">
      <button class="tablinks active" onclick="openTab(event, 'Overview')">Overview</button>
      <button class="tablinks" onclick="openTab(event, 'RawData')">Raw CAN Data</button>
      <button onclick="window.location.href='/history'">History Table</button>
      <button onclick="window.location.href='/graphs'">Graphs</button>
      <button onclick="window.location.href='/reference'">LiFePO4 Reference</button>
    </div>
    <div id="Overview" class="tabcontent" style="display:block;">
      <div style="display:flex; gap:15px; flex-wrap:wrap;">
        <div class="stat"><div>Pack Voltage</div><div class="stat-value" id="pack-voltage">---</div></div>
        <div class="stat"><div>Global Cell Spread</div><div class="stat-value" id="cell-spread">---</div></div>
        <div class="stat"><div>Module External Spread</div><div class="stat-value" id="module-spread">---</div></div>
        <div class="stat"><div>Worst Internal Module</div><div class="stat-value" id="worst-module">---</div></div>
        <div class="stat"><div>Temperature Range</div><div class="stat-value" id="temp-range">---</div></div>
      </div>

      <!-- CONTROL BUTTONS -->
      <div style="margin-top:20px; display:flex; gap:12px; flex-wrap:wrap; align-items:center;">
        <button onclick="eraseAllData()" 
                style="background:#e67e22; color:white; padding:9px 18px; border:none; border-radius:6px; font-size:0.95em; cursor:pointer;">
          🗑️ Clear All History
        </button>
      
        <small style="color:#777; font-size:0.9em;">
          (Clear = delete all history points )
        </small>
      </div>
      <!-- MODULES GRID -->
      <div class="modules" id="modules"></div>
    </div>   <!-- Close Overview -->

    <div id="RawData" class="tabcontent">
      <h2>Known Messages</h2>
      <table id="knownTable"><thead><tr><th>ID</th><th>Description</th><th>Data (Hex)</th><th>Count</th><th>Age (s)</th></tr></thead><tbody></tbody></table>
      <h2>Unknown Messages</h2>
      <table id="unknownTable"><thead><tr><th>ID</th><th>Data (Hex)</th><th>Count</th><th>Age (s)</th></tr></thead><tbody></tbody></table>
      <h2>0x180011F4 History (last 12)</h2>
      <table id="f4Table"><thead><tr><th>Data (Hex)</th><th>Age (s)</th></tr></thead><tbody></tbody></table>
    </div>

    <script>
        let currentLogMode = true; // full by default
        let ws;

        function eraseAllData() {
            if (!confirm("⚠️ Delete ALL stored history data?\n\nThis cannot be undone!")) return;
            fetch('/api/erase', { method: 'POST' })
                .then(() => { alert("✅ History erased!"); location.reload(); })
                .catch(err => { alert("Error"); console.error(err); });
        }

        function trimTo100() {
          if (!confirm("Trim history to last 100 entries?\nOlder data will be deleted.")) return;
          fetch('/api/trim?keep=100', { method: 'GET' })
            .then(r => r.text())
            .then(text => {
              alert(text);
              location.reload();
            })
            .catch(err => alert("Trim failed"));
        } 

        async function toggleLoggingMode() {
          const newMode = !currentLogMode;
          const modeName = newMode ? "FULL (with all cell data)" : "LIGHT (smaller logs)";
          if (!confirm(`Switch logging to ${modeName}?\n\nThe device will restart automatically.`)) return;
          try {
            const res = await fetch('/api/config', {
              method: 'POST',
              headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
              body: `log_full_cells=${newMode}`
            });
            const text = await res.text();
            alert(text);
          } catch(e) {
            alert("Failed to save config");
          }
        }

        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.host + '/ws');
            ws.onmessage = function(event) {
                const data = JSON.parse(event.data);
                updateDashboard(data);
                updateRawData(data);
            };
        }

        function updateDashboard(data) {
            const modulesDiv = document.getElementById('modules');
            // if (!modulesDiv) return;

            document.getElementById('pack-voltage').textContent = data.packVoltage.toFixed(2) + 'V';
            const cellSpread = data.cellSpread || 0;
            let cellColor = cellSpread > 50 ? 'spread-red' : (cellSpread > 30 ? 'spread-yellow' : 'spread-green');
            document.getElementById('cell-spread').innerHTML = `<span class="${cellColor}">${cellSpread.toFixed(0)} mV</span>`;

            const moduleSpread = data.moduleSpread || 0;
            let modColor = moduleSpread > 250 ? 'spread-red' : (moduleSpread > 150 ? 'spread-yellow' : 'spread-green');
            document.getElementById('module-spread').innerHTML = `<span class="${modColor}">${moduleSpread.toFixed(0)} mV</span>`;

            const minT = data.minTemp || 0;
            const maxT = data.maxTemp || 0;
            document.getElementById('temp-range').textContent = minT.toFixed(1) + '°F - ' + maxT.toFixed(1) + '°F';

            const worstIdx = data.worstModuleIndex || 0;
            const worstSpread = data.worstModuleInternalSpread || 0;
            let worstColor = worstSpread > 300 ? 'spread-red' : (worstSpread > 150 ? 'spread-yellow' : 'spread-green');
            document.getElementById('worst-module').innerHTML = `<span class="${worstColor}">Module ${data.modules[worstIdx].name} (${worstSpread.toFixed(0)} mV)</span>`;

            modulesDiv.innerHTML = '';
            if (data.modules) {
              data.modules.forEach(function(module, mIdx) {
                let div = document.createElement('div');
                div.className = 'module';
                if (mIdx === worstIdx) div.innerHTML += '<div class="worst-dot"></div>';
                let cellsHTML = '';
                module.cells.forEach(function(v, cIdx) {
                  let cls = (v < 3.0 || v > 3.4) ? 'danger' : ((v < 3.1 || v > 3.35) ? 'warning' : 'good');
                  if (Math.abs(v - data.globalMaxCell) < 0.001) cls += ' max';
                  if (Math.abs(v - data.globalMinCell) < 0.001) cls += ' min';
                  cellsHTML += `<div class="cell ${cls}">Cell ${cIdx+1}<br>${v.toFixed(3)}V</div>`;
                });
                div.innerHTML += `
                  <div class="module-header">Module ${module.name} <div class="status-dot ${module.connected ? 'connected' : ''}"></div></div>
                  <div class="module-voltage">Module Voltage: ${module.voltage.toFixed(2)}V</div>
                  <div class="cells">${cellsHTML}</div>
                  <div class="temps">Temps: ${module.temperatures[0].toFixed(1)}°F, ${module.temperatures[1].toFixed(1)}°F</div>
                `;
                modulesDiv.appendChild(div);
              });
            }
        }

        function updateRawData(data) {
          // Known Messages
          let html = '';
          if (data.knownMessages) {
            data.knownMessages.forEach(function(msg) {
              html += `<tr><td>0x${msg.id}</td><td>${msg.desc}</td><td>${msg.data}</td><td>${msg.count}</td><td>${msg.age}</td></tr>`;
            });
          }
          document.getElementById('knownTable').querySelector('tbody').innerHTML = html || '<tr><td colspan="5">No known messages yet</td></tr>';

          // Unknown Messages (add similar code if needed)
          // F4 History (add similar code if needed)
        }

        function openTab(evt, tabName) {
          var tabcontent = document.getElementsByClassName("tabcontent");
          for (var i = 0; i < tabcontent.length; i++) tabcontent[i].style.display = "none";
          var tablinks = document.getElementsByClassName("tablinks");
          for (var i = 0; i < tablinks.length; i++) tablinks[i].className = tablinks[i].className.replace(" active", "");
          document.getElementById(tabName).style.display = "block";
          evt.currentTarget.className += " active";
        }

        window.onload = function() {
            connectWebSocket();
        };
    </script>
</body>
</html>
)rawliteral";
  return html;
}

// end getDashboardHTML

String getConfigHTML() {
  uint8_t savedOctet = preferences.getUChar("static_octet", 100);
  bool useDHCP = preferences.getBool("use_dhcp", false);
  String dots = (homePass.length() > 0) ? String(homePass.length(), '•') : "none";
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>ESP32 CAN Config</title>";
  html += "<style>body{font-family:Arial;text-align:center;margin:40px;background:#f4f4f9;} h1{color:#2c3e50;} .box{background:white;padding:30px;margin:20px auto;max-width:600px;border-radius:12px;box-shadow:0 4px 15px rgba(0,0,0,0.1);text-align:left;} .form-group{margin:20px 0;} .form-label{display:block;font-weight:bold;margin-bottom:8px;color:#2c3e50;} input, select{width:100%;padding:12px;font-size:1em;border:1px solid #ccc;border-radius:6px;box-sizing:border-box;} button{padding:15px 30px;margin:20px 0;font-size:1.1em;border:none;border-radius:8px;color:white;cursor:pointer;background:#607d8b;} button:hover{background:#219653;}</style></head><body>";
  html += "<h1>BMS CAN Bus Config</h1><div class='box'>";
  html += "<strong>AP SSID:</strong> " + String(apSSID) + "<br>";
  html += "<strong>AP IP:</strong> " + apIP.toString() + "<br><br>";
  if (WiFi.status() == WL_CONNECTED) html += "<strong>Connected to:</strong> " + WiFi.SSID() + "<br><strong>Home IP:</strong> " + WiFi.localIP().toString() + "<br><br>";
  else html += "<strong>Not connected to home WiFi</strong><br><br>";
  html += "<button onclick=\"document.getElementById('form').style.display='block';this.style.display='none';\">Configure WiFi & Modules</button>";
  html += "<div id='form' style='display:none;'><form action='/save' method='POST'>";
  html += "<div class='form-group'><label class='form-label'>WiFi SSID:</label><input type='text' name='ssid' value='" + homeSSID + "'></div>";
  html += "<div class='form-group'><label class='form-label'>Password:</label><input type='password' name='pass' placeholder='Leave blank to keep current'><small>Current: " + dots + "</small></div>";
  html += "<div class='form-group'><label class='form-label'>IP Mode:</label>";
  html += "<select name='ip_mode'><option value='dhcp'" + String(useDHCP ? " selected" : "") + ">DHCP</option><option value='static'" + String(!useDHCP ? " selected" : "") + ">Static IP</option></select></div>";
  html += "<div class='form-group'><label class='form-label'>Static IP Last Octet (192.168.1.xxx):</label><input type='number' name='static_octet' value='" + String(savedOctet) + "'></div>";
  html += "<div class='form-group'><label class='form-label'>Number of Modules:</label><select name='modules'>";
  html += "<option value='4'" + String(configNumModules==4?" selected":"") + ">4</option>";
  html += "<option value='6'" + String(configNumModules==6?" selected":"") + ">6</option>";
  html += "<option value='8'" + String(configNumModules==8?" selected":"") + ">8</option>";
  html += "</select></div>";
  html += "<button type='submit'>Save & Restart</button></form></div></div></body></html>";
  return html;
}

// getHistoryHTML
String getHistoryHTML() {
  String status = ntpSynced ? "✅ NTP Synced" : "⚠️ NTP Not Synced";
  String htmlClass = ntpSynced ? "good" : "warn";

  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>BMS History</title>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <meta charset="UTF-8">
  <style>
    body{font-family:Arial,sans-serif;margin:20px;background:#f0f0f0;}
    .status{padding:12px;border-radius:8px;font-size:1.1em;margin:10px 0;}
    .status.good{background:#e8f5e9;color:#2e7d32;}
    .status.warn{background:#fff3e0;color:#ef6c00;}
    table{width:100%;border-collapse:collapse;margin-top:10px;}
    th,td{padding:10px;border:1px solid #ddd;text-align:center;}
    th{background:#2c3e50;color:white;}
    td.module-v {text-align:left; font-family:monospace; font-size:0.95em; max-width:420px; white-space:nowrap; overflow:auto;}
    button{margin:5px;padding:10px 16px;font-size:1em;}
  </style>
</head>
<body>
  <h1>BMS History + CSV Export</h1>
  <div class="status )rawliteral" + htmlClass + R"rawliteral(">)rawliteral" + status + R"rawliteral(</div>
  <p>
    <button onclick="fetchHistory()">Refresh Table</button>
    <!-- <button onclick="downloadCSV()">Download Full CSV</button> -->
    <a href="/">← Dashboard</a> | <a href="/graphs">Graphs</a>
  </p>
  <table id="historyTable">
    <thead>
      <tr>
        <th>Time</th>
        <th>Pack Voltage</th>
        <th>Cell Spread (mV)</th>
        <th>Module Spread (mV)</th>
        <th>Module Voltages (AA to B1)</th>
      </tr>
    </thead>
    <tbody></tbody>
  </table>

  <script>
  async function fetchHistory(){
    const limit = 50; 
    const res = await fetch(`/api/history?limit=${limit}&light=1`);
    if(!res.ok) return;
    let data = await res.json();
    data = data.filter(e => e.packV > 0.1);
    const tbody = document.querySelector('#historyTable tbody');
    tbody.innerHTML = '';
    data.forEach(entry => {
      const d = new Date(entry.ts * 1000);
      let modVStr = '';
      if (entry.moduleV && Array.isArray(entry.moduleV)) {
        modVStr = entry.moduleV.map(v => v.toFixed(2)).join(', ');
      }
      tbody.innerHTML += `<tr>
        <td>${d.toLocaleString()}</td>
        <td>${entry.packV.toFixed(3)} V</td>
        <td>${Math.round(entry.spread || 0)}</td>
        <td>${entry.moduleSpread ? Math.round(entry.moduleSpread) : '---'}</td>
        <td class="module-v">${modVStr}</td>
      </tr>`;
    });
  }
  function downloadCSV(){ 
    window.location.href = '/api/history/csv'; 
  }
  window.onload = fetchHistory;
  </script>
</body>
</html>
)rawliteral";

} // End of getHistoryHTML()




String getGraphsHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>BMS Graphs v2.26</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.1/dist/chart.umd.min.js"></script>
  <style>
    body {font-family: Arial, sans-serif; margin:20px; background:#f0f0f0;}
    .container {max-width:1450px; margin:auto; background:white; padding:20px; border-radius:8px;}
    canvas {margin:25px 0; border:1px solid #ddd;}
    .module-title {background:#34495e; color:white; padding:12px; border-radius:6px; margin:30px 0 10px;}
    .controls {margin:15px 0; display:flex; gap:12px; flex-wrap:wrap; align-items:center; background:#f8f9fa; padding:15px; border-radius:8px;}
    .btn {padding:10px 22px; font-size:1.05em; border:none; border-radius:6px; cursor:pointer; font-weight:bold;}
    .load-data-btn {background:#2980b9; color:white;}
    .cell-load-btn {background:#27ae60; color:white;}
    .debug-box {background:#f8f8f8; border:1px solid #ddd; padding:10px; margin:10px 0; font-family:monospace; font-size:13px; max-height:400px; overflow:auto;}
  </style>
</head>
<body>
  <div class="container">
    <h1>BMS Graphs v2.26 - Auto Loading (Cells on demand)</h1>
    
    <div class="controls">
      <button onclick="loadAndDisplayAllGraphs()" class="btn load-data-btn">🔄 Load/ Refresh Graphs</button>
      Show last 
      <select id="limit" onchange="loadAndDisplayAllGraphs()">
        <option value="30">30</option>
        <option value="60" selected>60</option>
        <option value="80" selected>80</option>

      </select>
      <a href="/">← Dashboard</a> | <a href="/history">History</a>
    </div>

    <div id="debugInfo" class="debug-box" style="display:none;"></div>

    <!-- Graph 1 -->
    <div class="module-title">1. Global Cell Spread Over Time</div>
    <canvas id="cellSpreadChart" height="280"></canvas>

    <!-- Graph 2 -->
    <div class="module-title">2. Module Spread Over Time</div>
    <canvas id="moduleSpreadChart" height="280"></canvas>

    <!-- Graph 3 -->
    <div class="module-title">3. Module Voltages + Pack Voltage</div>
    <canvas id="unifiedChart" height="320"></canvas>

    <!-- Graph 4 - Lazy -->
    <div class="module-title" id="cellGraphTitle">4. Module AA — 8 Individual Cells</div>
    
    <div style="margin:10px 0;">
      <select id="moduleSelect" style="padding:8px; margin-right:10px;">
        <option value="0" selected>Module AA</option>
        <option value="1">Module AB</option>
        <option value="2">Module AC</option>
        <option value="3">Module AD</option>
        <option value="4">Module AE</option>
        <option value="5">Module AF</option>
        <option value="6">Module B0</option>
        <option value="7">Module B1</option>
      </select>
      <button onclick="loadSelectedCellGraph()" class="btn cell-load-btn">📊 Load / Refresh Cell Graph</button>
    </div>
    
    <canvas id="moduleCellsChart" height="380"></canvas>
  </div>

  <script>
    let charts = [];
    let cachedHistoryData = null;

    function debug(msg) { 
      const box = document.getElementById('debugInfo');
      box.style.display = 'block';
      box.innerHTML += new Date().toLocaleTimeString() + " | " + msg + "<br>";
      box.scrollTop = box.scrollHeight;
    }

    function createDateAwareLabels(data) {
      return data.map((e,i) => {
        const d = new Date(e.ts*1000);
        const time = d.toLocaleTimeString([],{hour:'2-digit',minute:'2-digit'});
        if (i===0 || d.getDate() !== new Date(data[i-1].ts*1000).getDate())
          return d.toLocaleDateString([],{month:'short',day:'numeric'}) + "\n" + time;
        return time;
      });
    }

  async function loadDataCache() {
  const limit = document.getElementById('limit').value;
  debug(`Fetching ${limit} light records...`);

  try {
    const res = await fetch(`/api/history?limit=${limit}&light=1`);
    if (!res.ok) throw new Error(`HTTP ${res.status}`);

    const text = await res.text();           // ← Read as raw text first
    // === DEBUG: Show raw response ===
    console.log("=== RAW JSON RECEIVED ===");
    console.log(text);
    //debug(`Raw response size: ${text.length} characters`);


    if (text.length < 10) throw new Error("Empty response");
    // debug(`✅ received ${text.length} text`);

    const raw = JSON.parse(text);            // ← Parse manually
    cachedHistoryData = raw.filter(e => e && e.packV && e.packV > 0.5);
    
    debug(`✅ Loaded ${cachedHistoryData.length} records`);
    return true;
  } catch(e) {
    debug("❌ Error: " + e.message);
    return false;
  }
}

    async function renderAllGraphs() {
      if (!cachedHistoryData || cachedHistoryData.length === 0) {
        debug("No data available");
        return;
      }

      const labels = createDateAwareLabels(cachedHistoryData);

      charts.forEach(chart => { if(chart) chart.destroy(); });
      charts = [];

      // Graph 1-3 (unchanged)
      charts[0] = new Chart(document.getElementById('cellSpreadChart'), {
        type:'line', data:{labels, datasets:[{label:'Cell Spread (mV)', data:cachedHistoryData.map(e=>e.spread||0), borderColor:'#e74c3c', tension:0.2, borderWidth:3}]},
        options:{responsive:true, scales:{y:{min:0,max:400}}}
      });

      charts[1] = new Chart(document.getElementById('moduleSpreadChart'), {
        type:'line', data:{labels, datasets:[{label:'Module Spread (mV)', data:cachedHistoryData.map(e=>e.moduleSpread||0), borderColor:'#3498db', tension:0.2, borderWidth:3}]},
        options:{responsive:true, scales:{y:{min:0,max:1300}}}
      });

      const colors = ['#e74c3c','#3498db','#2ecc71','#f39c12','#9b59b6','#1abc9c','#34495e','#95a5a6'];
      const ds = [];
      for(let m=0;m<8;m++) {
        ds.push({label:`M${["AA","AB","AC","AD","AE","AF","B0","B1"][m]}`, data:cachedHistoryData.map(e=>e.moduleV?e.moduleV[m]:null), borderColor:colors[m], borderWidth:2.2, tension:0.3});
      }
      ds.push({label:'Total Pack Voltage (Sum)', data:cachedHistoryData.map(e=>e.packV), borderColor:'#111', borderWidth:3.5, tension:0.1, yAxisID:'y1'});
      
      charts[2] = new Chart(document.getElementById('unifiedChart'), {
        type: 'line', data: {labels, datasets: ds}, 
        options: { responsive:true, scales:{ y:{position:'left', min:24, max:28.5}, y1:{position:'right', min:190, max:230, grid:{drawOnChartArea:false}} } }
      });
    }

    // === NEW: Load cell graph ONLY when button clicked ===
    async function loadSelectedCellGraph() {
      const modIdx = parseInt(document.getElementById('moduleSelect').value);
      const modName = ["AA","AB","AC","AD","AE","AF","B0","B1"][modIdx];
      document.getElementById('cellGraphTitle').textContent = `4. Module ${modName} — 8 Individual Cells`;

      if (charts[3]) charts[3].destroy();

      try {
        const limit = document.getElementById('limit').value;
        debug(`Loading cell data for module ${modName}...`);
        const res = await fetch(`/api/moduleCells?module=${modIdx}&limit=${limit}`);
        const cellData = await res.json();

        if (!cellData || cellData.length === 0) {
          debug(`No cell data for module ${modName}`);
          return;
        }

        const labels = createDateAwareLabels(cellData);
        const colors = ['#e74c3c','#3498db','#2ecc71','#f39c12','#9b59b6','#1abc9c','#34495e','#95a5a6'];
        const datasets = [];

        for(let c = 0; c < 8; c++) {
          datasets.push({
            label: `Cell ${c+1}`,
            data: cellData.map(e => e.cells ? e.cells[c] : null),
            borderColor: colors[c],
            borderWidth: 2.5,
            tension: 0.2
          });
        }

        charts[3] = new Chart(document.getElementById('moduleCellsChart'), {
          type: 'line',
          data: { labels, datasets },
          options: { responsive: true, scales: { y: { min: 2.8, max: 3.6 } }}
        });
        debug(`✅ Cell graph for ${modName} loaded`);
      } catch(e) {
        debug("❌ Cell graph error: " + e.message);
        console.error(e);
      }
    }

    async function loadAndDisplayAllGraphs() {
      const success = await loadDataCache();
      if (success) {
        await renderAllGraphs();
      }
    }

   // window.onload = loadAndDisplayAllGraphs;
   // No auto-load on page open
    window.onload = function() {
      console.log("✅ Graphs page ready. Click 'Refresh Main Graphs' to load data.");
};
  </script>
</body>
</html>
)rawliteral";
}


String getReferenceHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>BMS Reference - 8-Cell & 64-Cell LiFePO4</title>
    <style>
        body {font-family: Arial, Helvetica, sans-serif; margin: 20px; background: #f4f7f9; color: #333;}
        .container {max-width: 1200px; margin: auto; background: white; padding: 30px; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.1);}
        h1, h2 {color: #2c3e50;}
        table {width: 100%; border-collapse: collapse; margin: 25px 0;}
        th, td {padding: 14px; text-align: center; border: 1px solid #ddd;}
        th {background: #2c3e50; color: white;}
        tr:nth-child(even) {background: #f9f9f9;}
        .note {background: #e8f4fc; padding: 18px; border-left: 6px solid #3498db; margin: 25px 0; border-radius: 4px;}
        .warning {background: #fff3e0; padding: 18px; border-left: 6px solid #f39c12;}
        .good {color: #27ae60; font-weight: bold;}
        .yellow {color: #f39c12; font-weight: bold;}
        .red {color: #e74c3c; font-weight: bold;}
        .header {background: #2c3e50; color: white; padding: 20px; border-radius: 8px; margin-bottom: 30px; text-align: center;}
        .back-btn {display: inline-block; margin: 20px 0; padding: 14px 28px; background: #607d8b; color: white; text-decoration: none; border-radius: 6px;}
        .back-btn:hover {background: #34495e;}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>LiFePO4 Reference Guide</h1>
            <p>8-Cell Module • 64-Cell Stack • Spread & Balance Limits</p>
        </div>
        <a href="/" class="back-btn">← Back to Dashboard</a>

        <h2>1. Single Cell Reference (3.2V Nominal)</h2>
        <table>
            <thead><tr><th>SOC (%)</th><th>Resting Voltage</th><th>Charging Voltage</th><th>Status</th></tr></thead>
            <tbody>
                <tr><td>100%</td><td>3.40 – 3.45 V</td><td>3.65 V</td><td class="red">Fully charged – stop here for max life</td></tr>
                <tr><td>90%</td><td>3.34 – 3.36 V</td><td>3.50 – 3.55 V</td><td>Good upper daily limit</td></tr>
                <tr><td>80%</td><td>3.32 – 3.34 V</td><td>~3.45 V</td><td class="good">Excellent daily target (20-80% rule)</td></tr>
                <tr><td>50%</td><td>3.27 – 3.28 V</td><td>—</td><td>Flat curve region</td></tr>
                <tr><td>20%</td><td>3.20 – 3.22 V</td><td>—</td><td class="good">Recommended lower daily limit</td></tr>
                <tr><td>0%</td><td>~2.50 V</td><td>—</td><td class="red">Do not go below</td></tr>
            </tbody>
        </table>

        <h2>2. 8-Cell Module Reference (25.6V Nominal)</h2>
        <table>
            <thead><tr><th>SOC (%)</th><th>Resting Voltage</th><th>Charging Voltage</th><th>Status</th></tr></thead>
            <tbody>
                <tr><td>100%</td><td>27.20 – 27.60 V</td><td>29.20 V</td><td class="red">Fully charged</td></tr>
                <tr><td>90%</td><td>26.72 – 26.88 V</td><td>28.00 – 28.40 V</td><td>Good upper limit</td></tr>
                <tr><td>80%</td><td>26.56 – 26.72 V</td><td>27.60 V</td><td class="good">Excellent daily target</td></tr>
                <tr><td>50%</td><td>26.16 – 26.24 V</td><td>—</td><td>Flat curve</td></tr>
                <tr><td>20%</td><td>25.60 – 25.76 V</td><td>—</td><td class="good">Recommended lower limit</td></tr>
                <tr><td>0%</td><td>~20.00 V</td><td>—</td><td class="red">Do not go below</td></tr>
            </tbody>
        </table>

        <h2>3. 64-Cell Full Pack Reference (204.8V Nominal)</h2>
        <table>
            <thead><tr><th>SOC (%)</th><th>Resting Voltage</th><th>Charging Voltage</th><th>Status</th></tr></thead>
            <tbody>
                <tr><td>100%</td><td>217.6 – 220.8 V</td><td>233.6 V</td><td class="red">Fully charged</td></tr>
                <tr><td>90%</td><td>213.8 – 215.0 V</td><td>224.0 – 227.2 V</td><td>Good upper limit</td></tr>
                <tr><td>80%</td><td>212.5 – 213.8 V</td><td>220.8 V</td><td class="good">Excellent daily target</td></tr>
                <tr><td>50%</td><td>209.3 – 209.9 V</td><td>—</td><td>Flat curve</td></tr>
                <tr><td>20%</td><td>204.8 – 206.1 V</td><td>—</td><td class="good">Recommended lower limit</td></tr>
                <tr><td>0%</td><td>~160 V</td><td>—</td><td class="red">Do not go below</td></tr>
            </tbody>
        </table>

        <h2>4. Spread & Balance Limits (Your 64-Cell Stack)</h2>
        <table>
            <thead><tr><th>Parameter</th><th>Excellent</th><th>Good / Normal</th><th>Watch</th><th>Problem (Action Needed)</th></tr></thead>
            <tbody>
                <tr><td><strong>Global Cell Spread</strong></td><td>&lt; 30 mV</td><td class="good">30 – 80 mV</td><td class="yellow">80 – 150 mV</td><td class="red">&gt; 150 mV</td></tr>
                <tr><td><strong>Module External Spread</strong> (between modules)</td><td>&lt; 30 mV</td><td class="good">30 – 100 mV</td><td class="yellow">100 – 250 mV</td><td class="red">&gt; 250 mV</td></tr>
                <tr><td><strong>Worst Module Internal Spread</strong> (inside one module)</td><td>&lt; 50 mV</td><td class="good">50 – 120 mV</td><td class="yellow">120 – 300 mV</td><td class="red">&gt; 300 mV</td></tr>
            </tbody>
        </table>

        <div class="note">
            <strong>Key Rules for Your System:</strong><br>
            • Daily operation: Global spread < 80 mV and Module External < 100 mV is ideal<br>
            • If your module internal spread is in the <span class="red">Problem</span> zone — continue full charging to let the BMS balance it.<br>
            • Regular full charges (to 3.55–3.65 V per cell) are the best way to keep spreads low.
        </div>

        <p style="text-align:center; color:#7f8c8d; margin-top:40px;">
            Data based on common LiFePO4 datasheets and real-world 8S8P experience.
        </p>
    </div>
</body>
</html>
)rawliteral";
}



// end getReference 
  


// ====================== WEB SERVER ======================
void setupWebServer() {
  // WebSocket Handler
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
      client->text(getBMSDataJSON());
    }
  });
  server.addHandler(&ws);

  // ====================== HTML PAGES ======================
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getDashboardHTML());
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getConfigHTML());
  });
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getHistoryHTML());
  });
  server.on("/graphs", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getGraphsHTML());
  });
  server.on("/reference", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getReferenceHTML());
  });

  // ====================== API ENDPOINTS ======================
  server.on("/api/bms", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getBMSDataJSON());
  });

 

Serial.printf("[HISTORY] Free Heap before response: %d bytes\n", ESP.getFreeHeap());


// ====================== IMPROVED /api/history - Supports ?limit=X (LATEST entries) ======================
// v2.28 - Safe single-pass history (no std::vector, lower risk of crash)
// v2.29 - Lightweight history endpoint for fast 200-point graphs

// ====================== IMPROVED /api/history ======================
server.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request){
  int limit = request->hasParam("limit") ? request->getParam("limit")->value().toInt() : 100;
  if (limit < 20) limit = 20;
  if (limit > 80) limit = 80;

  bool lightMode = request->hasParam("light") && request->getParam("light")->value() == "1";

  Serial.printf("[HISTORY] API - limit=%d light=%d | Heap=%d\n", limit, lightMode, ESP.getFreeHeap());

  File file = LittleFS.open("/cell_log.jsonl", "r");
  if (!file) {
    Serial.println("[HISTORY] ❌ No log file");
    request->send(200, "application/json", "[]");
    return;
  }

  DynamicJsonDocument responseDoc(32768);
  JsonArray arr = responseDoc.to<JsonArray>();

  String line;
  String lastLines[81];   // circular buffer
  int count = 0;

  // First pass: collect last N valid lines
  while (file.available()) {
    line = file.readStringUntil('\n');
    if (line.length() < 80) continue;

    DynamicJsonDocument doc(900);
    if (deserializeJson(doc, line) == DeserializationError::Ok && doc["packV"].as<float>() > 0.5) {
      lastLines[count % 80] = line;   // store raw line
      count++;
    }
  }
  file.close();

  Serial.printf("[HISTORY] Found %d valid entries in log\n", count);

  int numToSend = min(count, limit);
  int start = count > numToSend ? (count - numToSend) % 80 : 0;
  for (int i = 0; i < numToSend; i++) {
    int idx = (start + i) % 80;
    if (lastLines[idx].length() == 0) continue;

    DynamicJsonDocument doc(900);
    if (deserializeJson(doc, lastLines[idx]) == DeserializationError::Ok) {
      JsonObject obj = arr.createNestedObject();
      obj["ts"] = doc["ts"].as<long long>();
      obj["packV"] = doc["packV"].as<float>();
      obj["spread"] = doc["spread"] | 0;
      obj["moduleSpread"] = doc["moduleSpread"] | 0;

      if (doc.containsKey("moduleV")) obj["moduleV"] = doc["moduleV"];
      if (!lightMode && doc.containsKey("cells")) obj["cells"] = doc["cells"];
    }
  }

  String output = "";
  serializeJson(responseDoc, output);
  if (output.length() < 10) output = "[]";

  request->send(200, "application/json", output);
  Serial.printf("[HISTORY] ✅ Sent %d most recent records (of %d total) | JSON size %d\n", 
              numToSend, count, output.length());
});


// END FIXED HANDLER ======================

// cell handler 
// ====================== NEW: SEPARATE LIGHTWEIGHT ENDPOINT FOR MODULE CELLS ======================
server.on("/api/moduleCells", HTTP_GET, [](AsyncWebServerRequest *request){
  int moduleFilter = -1;
  if (request->hasParam("module")) {
    moduleFilter = request->getParam("module")->value().toInt();
  }
  if (moduleFilter < 0 || moduleFilter > 7) moduleFilter = 0;

  int limit = request->hasParam("limit") ? request->getParam("limit")->value().toInt() : 80;
  if (limit < 20) limit = 20;
  if (limit > 120) limit = 120;

  Serial.printf("[MODULECELLS] Requested module=%d limit=%d | Heap=%d\n", 
                moduleFilter, limit, ESP.getFreeHeap());

  File file = LittleFS.open("/cell_log.jsonl", "r");
  if (!file) {
    request->send(200, "application/json", "[]");
    return;
  }

  DynamicJsonDocument responseDoc(16384);
  JsonArray arr = responseDoc.to<JsonArray>();

  String line;
  int sent = 0;

  while (file.available() && sent < limit) {
    line = file.readStringUntil('\n');
    if (line.length() < 80) continue;

    DynamicJsonDocument doc(700);
    if (deserializeJson(doc, line) == DeserializationError::Ok && doc["packV"].as<float>() > 0.5) {
      
      JsonObject obj = arr.createNestedObject();
      obj["ts"] = doc["ts"].as<long long>();
      obj["packV"] = doc["packV"].as<float>();

      // Only return the 8 cells for the requested module
      if (doc.containsKey("cells")) {
        JsonArray allCells = doc["cells"];
        JsonArray modCells = obj.createNestedArray("cells");
        for (int c = 0; c < 8; c++) {
          int idx = moduleFilter * 8 + c;
          modCells.add((idx < allCells.size()) ? allCells[idx] : 0.0);
        }
      }

      sent++;
    }
  }
  file.close();

  String output = "";
  serializeJson(responseDoc, output);
  if (output.length() < 10) output = "[]";

  request->send(200, "application/json", output);

  Serial.printf("[MODULECELLS] ✅ Sent %d records for module %d\n", sent, moduleFilter);
});
// ====================== END NEW ENDPOINT ======================

// new config endpoint 
// ====================== CONFIG API ======================
server.on("/api/config", HTTP_POST, [](AsyncWebServerRequest *request){
  if (request->hasParam("log_full_cells", true)) {
    bool newMode = request->getParam("log_full_cells", true)->value() == "true";
    preferences.putBool("log_full_cells", newMode);
    logFullCells = newMode;
    Serial.printf("[CONFIG] Logging mode set to: %s\n", newMode ? "FULL" : "LIGHT");
  }

  request->send(200, "text/plain", "Configuration saved. Restarting...");
  delay(800);
  ESP.restart();
});


// end new config endpoint 


// end cell handle 


  // ====================== CSV EXPORT (unchanged - still works) ======================
 
  server.on("/api/history/csv", HTTP_GET, [](AsyncWebServerRequest *request){
    File file = LittleFS.open("/cell_log.jsonl", "r");
    if (!file) {
      request->send(200, "text/csv", "No data yet\n");
      return;
    }

    String csv = "Timestamp,Pack Voltage (V),Cell Spread (mV),Module Spread (mV)\n";
    String line;
    while (file.available()) {
      line = file.readStringUntil('\n');
      if (line.length() < 50) continue;

      DynamicJsonDocument doc(1800);
      if (deserializeJson(doc, line) == DeserializationError::Ok) {
        time_t ts = doc["ts"];
        struct tm *tm = localtime(&ts);
        char timeStr[32];
        strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", tm);

        csv += String(timeStr) + "," 
             + String(doc["packV"].as<float>(), 3) + "," 
             + String(doc["spread"] | 0) + "," 
             + String(doc["moduleSpread"] | 0) + "\n";
      }
    }
    file.close();
    request->send(200, "text/csv", csv);
  });
  // ====================== END CSV EXPORT ======================


  // Erase all history data
  server.on("/api/erase", HTTP_POST, [](AsyncWebServerRequest *request){
    Serial.println("[ERASE] Received erase request");
    if (LittleFS.exists("/cell_log.jsonl")) {
      LittleFS.remove("/cell_log.jsonl");
      Serial.println("[ERASE] ✅ cell_log.jsonl deleted");
    } else {
      Serial.println("[ERASE] No log file found");
    }
    request->send(200, "text/plain", "Erased");
  });


// ====================== TRIM ENDPOINT ======================
// Supports both GET ?keep=100 and POST
server.on("/api/trim", HTTP_GET, [](AsyncWebServerRequest *request){
  int keep = 200;
  
  if (request->hasParam("keep")) {
    keep = request->getParam("keep")->value().toInt();
  }

  if (keep < 50) keep = 50;
  if (keep > 1000) keep = 1000;

  trimToLastN(keep);
  
  String msg = "✅ Trimmed to last " + String(keep) + " entries";
  request->send(200, "text/plain", msg);
  Serial.println("[TRIM] " + msg);
});

// POST fallback (for the JS button)
server.on("/api/trim", HTTP_POST, [](AsyncWebServerRequest *request){
  int keep = request->hasParam("keep", true) ? request->getParam("keep", true)->value().toInt() : 200;
  if (keep < 50) keep = 50;
  if (keep > 1000) keep = 1000;
  
  trimToLastN(keep);
  
  String msg = "✅ Trimmed to last " + String(keep) + " entries";
  request->send(200, "text/plain", msg);
  Serial.println("[TRIM] " + msg);
});






  // Save Config
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("ssid", true)) preferences.putString("ssid", request->getParam("ssid", true)->value());
    if (request->hasParam("pass", true)) {
      String p = request->getParam("pass", true)->value();
      if (p.length() > 0) preferences.putString("pass", p);
    }
    bool dhcp = request->hasParam("ip_mode", true) && request->getParam("ip_mode", true)->value() == "dhcp";
    preferences.putBool("use_dhcp", dhcp);
    if (!dhcp && request->hasParam("static_octet", true)) 
      preferences.putUChar("static_octet", request->getParam("static_octet", true)->value().toInt());
    if (request->hasParam("modules", true)) 
      preferences.putUChar("modules", request->getParam("modules", true)->value().toInt());

    request->send(200, "text/html", "<h3>✅ Saved! Restarting in 2 seconds...</h3><script>setTimeout(()=>{location.href='/';},1800);</script>");
    delay(2000);
    ESP.restart();
  });

  server.begin();
  Serial.println("✅ Web server started with full API + Graphs support");
} // End of setupWebServer()