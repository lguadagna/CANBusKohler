// lhamilton 12/20/25 version 1.4
// AP or WiFi mode only to avoid channel switching
// press 'boot' button for 4 seconds to erase all configs
// static IP of 192.168.1.100 or choose last octet on configs page
// added config for WiFi and modules reported. 
// SBChanged from driver/can.h to driver/twai.h to work with the DOIT Devkit boaard.
//Also changed ESPAsyncWebServer etc  to fix compiler errors
// Note:  must roll ESP32 by Espresif board version back to 2.0.11 per Clint
//Version 1.2 changes back to ESPAsyncWebServer by lacamera and AsyncTCP by dvarrel
//to see if it still gives compiler errors.  It compiles okay and runs okay!
//Version 1.3 toggles the builtin LED once per Wifi update = once/second.
//

// From SB instructions on board specific libraries needed
//You will need the following libraries:
//Arduinojson by Benoit Blanchon ver 7.4.2
//AsynchTCP by dvarrel  ver 1.1.4
//ESP32-TWAI-CAN by sorek.uk  ver 1.0.1
//ESPAsychTCP by dvarrel  ver 1.2.4  (this may be the same thing as above, but I have both libraries installed)
//ESPAsych Web Server  by lacamera  ver 3.1.0
// Board1- DOIT ESP32 DEVKIT V1https://www.silabs.com/software-and-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads
// downloaded cp210 driver for usbc port 
///  
// github for eesp32 by espressif 

#include <AsyncUDP.h>
#include "driver/twai.h"
#include "WiFi.h"
#include <AsyncTCP.h>
#include "ESPAsyncWebServer.h"
#include "AsyncWebSocket.h"
#include "ArduinoJson.h"

// #include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>


bool ledState = false;

Preferences preferences;

// ───── FORWARD DECLARATION  ─────
String getBMSDataJSON();
String getDashboardHTML();
String getConfigHTML(); 
void setupWebServer(); 

// these are for boot press
unsigned long bootButtonPressTime = 0;
const unsigned long BOOT_ERASE_HOLD_TIME = 3000;  // Hold for 3 seconds to erase

const char* apSSID      = "ESP32_CANBus";   // Always-on Access Point
const char* apPassword  = "12345678";

String homeSSID = "";
String homePass = "";

uint8_t configNumModules = 8;  // Default = 8 modules

// set AP host IP 
IPAddress apIP(192, 168, 4, 1);      // ← Your new AP IP
IPAddress apGateway(192, 168, 4, 1);
IPAddress apSubnet(255, 255, 255, 0);


// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void setupWiFi() {
  Serial.begin(115200);
  delay(1000);
  preferences.begin("wifi-config", false);

  homeSSID = preferences.getString("ssid", "");
  homePass = preferences.getString("pass", "");
  configNumModules = preferences.getUChar("modules", 8);  // Load saved value, default 8

  // Get the unique MAC address (chip ID)
  uint64_t chipid = ESP.getEfuseMac();

  // Extract the last byte (last two hex digits)
  uint8_t last_byte = (uint8_t)chipid;  // Lowest byte of the MAC

  // Format as two uppercase hex digits (e.g., "A3")
  char unique_suffix[3];
  snprintf(unique_suffix, sizeof(unique_suffix), "%02X", last_byte);

  // Build the full SSID (base + "-" + last two digits)
  char full_ssid[32];  // Max SSID length is 32 chars
  snprintf(full_ssid, sizeof(full_ssid), "%s-%s", apSSID, unique_suffix);
  // Optional: Print for debugging
  Serial.printf("AP SSID: %s\n", full_ssid);

  // Always start our own Access Point (so you can never lose access)
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(full_ssid, apPassword);
  Serial.println("AP started → " + String(full_ssid));
  Serial.println("AP URL      → http://" + WiFi.softAPIP().toString());

  // === mDNS setup - this is the magic part ===
  if (MDNS.begin("ESP32_CANBus")) {  // http://ESP32_CANBus
    Serial.println("mDNS responder started: http://ESP32_CANBus");
  } else {
    Serial.println("Error setting up mDNS");
  }

  if (homeSSID.length() > 0) {
    Serial.println("Found saved WiFi credentials — switching to STA mode");  
    WiFi.mode(WIFI_STA);  // Only station mode if we have credentials
    WiFi.begin(homeSSID.c_str(), homePass.c_str());

    Serial.print("Connecting to home WiFi: " + homeSSID);
    uint16_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 60) {
      delay(1000);
      Serial.print(".");
      attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
      // Apply saved static IP
      uint8_t octet = preferences.getUChar("static_octet", 100);
      IPAddress staticIP(192, 168, 1, octet);
      IPAddress gateway(192, 168, 1, 1);
      IPAddress subnet(255, 255, 255, 0);

      if (WiFi.config(staticIP, gateway, subnet)) {
        Serial.printf("\nStatic IP applied: 192.168.1.%d\n", octet);
      } else {
        Serial.println("\nFailed to apply static IP (using DHCP)");
      }

      Serial.println("\n✅ Connected to home WiFi (STA mode)");
      Serial.println("Home IP     → http://" + WiFi.localIP().toString());
    } else {
      Serial.println("\nFailed to connect — staying in AP mode");
      WiFi.mode(WIFI_AP);  // Fall back to AP-only
    }
  } else {
    Serial.println("No saved WiFi → running in AP-only mode");
  }
}  // ← This closes setupWiFi()

// Simple unknown message tracking
struct SimpleUnknownMessage {
  uint32_t id;
  uint8_t data[8];
  uint8_t length;
  uint32_t count;
  unsigned long lastSeen;
};

// Simple known message tracking - just count what we see
struct SimpleKnownMessage {
  uint32_t id;
  const char* description;
  uint32_t count;
  unsigned long lastSeen;
  uint8_t data[8];      // Store last seen data
  uint8_t length;       // Store data length
};

SimpleUnknownMessage unknownMessages[20];  // Track last 20 unknown messages
int unknownCount = 0;

SimpleKnownMessage knownMessages[30];  // Track known message types
int knownCount = 0;

// BMS Data Structure
struct BMSData {
  float cellVoltages[64];  // 8 modules x 8 cells
  float moduleVoltages[8]; // Total voltages
  float temperatures[16];   // 2 per module
  float packVoltage;
  float maxTemp;
  float minTemp;
  float cellSpread;
  unsigned long lastUpdate;
  bool nodeStatus[8];      // AA, AB, AC, AD, AE, AF, B0, B1 connectivity
} bmsData;

// Track which cells belong to which module
const char* moduleNames[] = {"AA", "AB", "AC", "AD", "AE", "AF", "B0", "B1" };

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode (GPIO_NUM_2, OUTPUT); //setup built-in LED
  digitalWrite (GPIO_NUM_2, ledState); //initialize off

  // === ERASE CONFIGS IF BOOT BUTTON IS HELD ===
  pinMode(0, INPUT_PULLUP);  // GPIO 0 = BOOT button
  delay(100);  // Small stabilize time

  if (digitalRead(0) == LOW) {  // Button is pressed (pulled to GND)
    Serial.println("BOOT button held — ERASING ALL SAVED CONFIGS!");
    // Fast LED blink to confirm
    for (int i = 0; i < 10; i++) {
      digitalWrite(GPIO_NUM_2, !digitalRead(GPIO_NUM_2));
      delay(150);
    }
    preferences.begin("wifi-config", false);
    preferences.clear();  // Deletes all keys: ssid, pass, static_octet, modules
    preferences.end();
    Serial.println("All configs erased. Rebooting to AP mode...");
    delay(1000);
    ESP.restart();
  }
  // =========================================

  
  // Initialize BMS data
  memset(&bmsData, 0, sizeof(bmsData));
  memset(unknownMessages, 0, sizeof(unknownMessages));
  memset(knownMessages, 0, sizeof(knownMessages));
  
  Serial.println("=== BMS Network Dashboard ===");
  
  // Initialize CAN (your existing code)
  setupTWAI();
  
  // Connect to WiFi
  setupWiFi();
  
  // Setup web server
  setupWebServer();
  
  Serial.println("Dashboard ready!");
  Serial.printf("Access dashboard at: http://%s\n", WiFi.localIP().toString().c_str());
}

void setupTWAI() {
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_general_config_t g_config = {
    .mode = TWAI_MODE_LISTEN_ONLY,
    .tx_io = GPIO_NUM_17,
    .rx_io = GPIO_NUM_16,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 30,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0
  };
  
  esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
  if (result == ESP_OK) {
    Serial.println("✅ CAN driver installed");
    twai_start();
    Serial.println("✅ CAN started");
  } else {
    Serial.printf("❌ CAN failed: %s\n", esp_err_to_name(result));
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
                     AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("WebSocket client connected: %u\n", client->id());
    // Send current data to new client
    client->text(getBMSDataJSON());
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("WebSocket client disconnected: %u\n", client->id());
  }
}

void setupWebServer() {
  // Handle WebSocket connections
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // Serve the dashboard HTML
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getDashboardHTML());
  });

  // Serve the wifi config HTML 
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/html", getConfigHTML());
  });

  // API endpoint for current data
  server.on("/api/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", getBMSDataJSON());
  });

  // Save new credentials
  server.on("/save", HTTP_POST, [](AsyncWebServerRequest *request) {
    if (request->hasParam("ssid", true)) {
      homeSSID = request->getParam("ssid", true)->value();
          preferences.putString("ssid", homeSSID);
    }
    if (request->hasParam("pass", true)) {
      homePass = request->getParam("pass", true)->value();   
      homePass.trim(); 

      if (homePass.length() > 0 ) {
        preferences.putString("pass", homePass);
        Serial.println("Password updated and saved");
    }
    }

    if (request->hasParam("static_octet", true)) {
      uint8_t octet = request->getParam("static_octet", true)->value().toInt();
    if (octet >= 2 && octet <= 254) {  // Valid range (avoid 0,1,255)
      preferences.putUChar("static_octet", octet);
      Serial.printf("Saved static octet: %d → IP will be 192.168.1.%d\n", octet, octet);
    }
    }

    if (request->hasParam("modules", true)) {
      configNumModules = request->getParam("modules", true)->value().toInt();
      preferences.putUChar("modules", configNumModules);
      if (configNumModules != 4 && configNumModules != 6 && configNumModules != 8) {
        configNumModules = 8; // safety
      }
    }

    Serial.printf("Saved: WiFi='%s'/'%s', Modules=%d\n", homeSSID.c_str(), homePass.c_str(), configNumModules);

    request->send(200, "text/html",
      "<h3>Saved!</h3><p>Rebooting in 3s...</p>"
      "<script>setTimeout(()=>{location='/';},3000);</script>");
    delay(3000);
    ESP.restart();
  });

  
  // Start server
  server.begin();
  Serial.println("✅ Web server started");
}


void loop(){

  twai_message_t rx_message;
  
  // Read CAN messages
  if (twai_receive(&rx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
    procesCANMessage(rx_message);
  }
  
  // Send updates to connected clients every 1 second
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    broadcastUpdate();
    lastUpdate = millis();

    if (ledState == false){
    ledState= true;
    }
    else{
    ledState= false;
  }
  digitalWrite (GPIO_NUM_2, ledState);
  }
  
// === CHECK BOOT BUTTON FOR CONFIG ERASE ===
  if (digitalRead(0) == LOW) {  // Button pressed
    if (bootButtonPressTime == 0) {
      bootButtonPressTime = millis();  // Start timing
      Serial.println("BOOT button pressed — hold for 3 seconds to erase configs");
      // Optional: fast blink LED to show detection
      for (int i = 0; i < 5; i++) {
        digitalWrite(GPIO_NUM_2, HIGH);
        delay(100);
        digitalWrite(GPIO_NUM_2, LOW);
        delay(100);
      }
    }

    // Check if held long enough
    if (millis() - bootButtonPressTime >= BOOT_ERASE_HOLD_TIME) {
      Serial.println("BOOT button held long enough — ERASING ALL CONFIGS!");
      // Fast blink confirmation
      for (int i = 0; i < 10; i++) {
        digitalWrite(GPIO_NUM_2, !digitalRead(GPIO_NUM_2));
        delay(150);
      }

      preferences.begin("wifi-config", false);
      preferences.clear();
      preferences.end();

      Serial.println("Configs erased! Rebooting to AP mode...");
      delay(1000);
      ESP.restart();
    }
  } else {
    if (bootButtonPressTime != 0) {
      Serial.println("BOOT button released — not long enough to erase");
      bootButtonPressTime = 0;  // Reset timer
    }
  }
  // ======================================

  // Clean up WebSocket clients
  ws.cleanupClients();
}  // end loop ]\

// Simple helper to track known messages without disrupting existing logic
void trackKnownMessage(uint32_t id, const char* description, twai_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < knownCount; i++) {
    if (knownMessages[i].id == id) {
      knownMessages[i].count++;
      knownMessages[i].lastSeen = millis();
      // Update with latest data
      memcpy(knownMessages[i].data, msg.data, msg.data_length_code);
      knownMessages[i].length = msg.data_length_code;
      return;
    }
  }
  
  // Add new known message if we have space
  if (knownCount < 30) {
    knownMessages[knownCount].id = id;
    knownMessages[knownCount].description = description;
    knownMessages[knownCount].count = 1;
    knownMessages[knownCount].lastSeen = millis();
    memcpy(knownMessages[knownCount].data, msg.data, msg.data_length_code);
    knownMessages[knownCount].length = msg.data_length_code;
    knownCount++;
  }
}

void procesCANMessage(twai_message_t& msg) {
  uint32_t id = msg.identifier;
  uint8_t* data = msg.data;
  bool isKnownMessage = false;
  
  // Update last seen time
  bmsData.lastUpdate = millis();
  
  // Parse voltage messages (cells 1-4) - ORIGINAL CODE UNCHANGED
  if ((id & 0xFFFFFF00) == 0x18001300) {
    isKnownMessage = true;
    trackKnownMessage(id, "Voltage Data (Cells 1-4)", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      bmsData.nodeStatus[module] = true;
      for (int i = 0; i < 4; i++) {
        uint16_t voltage = (data[i*2+1] << 8) | data[i*2];
        bmsData.cellVoltages[module*8 + i] = voltage / 1000.0;
      }
    }
  }
  
  // Parse voltage messages (cells 5-8) - ORIGINAL CODE UNCHANGED
  else if ((id & 0xFFFFFF00) == 0x18011300) {
    isKnownMessage = true;
    trackKnownMessage(id, "Voltage Data (Cells 5-8)", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      for (int i = 0; i < 4; i++) {
        uint16_t voltage = (data[i*2+1] << 8) | data[i*2];
        bmsData.cellVoltages[module*8 + i + 4] = voltage / 1000.0;
      }
    }
  }
  
  // Parse temperature messages - ORIGINAL CODE UNCHANGED
  else if ((id & 0xFFFFFF00) == 0x18001400) {
    isKnownMessage = true;
    trackKnownMessage(id, "Temperature Data", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      uint16_t temp1 = (data[1] << 8) | data[0];
      uint16_t temp2 = (data[3] << 8) | data[2];
      bmsData.temperatures[module*2] = temp1/10.0;     // Keep in Fahrenheit
      bmsData.temperatures[module*2 + 1] = temp2/10.0;
    }
  }
  
  // Check for the new message types we discovered - ORIGINAL CODE UNCHANGED
  else if (id == 0x180011F4) {
    isKnownMessage = true;
    trackKnownMessage(id, "Master Request", msg);  // Pass msg for data
    Serial.printf("Master Request: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  else if ((id & 0xFFFFFF00) == 0x18001200) {
    isKnownMessage = true;  // Basic ACK messages
    trackKnownMessage(id, "Basic ACK Messages", msg);  // Pass msg for data
  }
  else if ((id & 0xFFFFFF00) == 0x18021200) {
    isKnownMessage = true;  // Heartbeat messages
    trackKnownMessage(id, "Heartbeat Messages", msg);  // Pass msg for data
  }
  else if ((id & 0xFFFFFF00) == 0x18031200) {
    isKnownMessage = true;  // Node health messages - ORIGINAL CODE UNCHANGED
    trackKnownMessage(id, "Node Health/Status", msg);  // Pass msg for data
    int module = getModuleIndex(id & 0xFF);
    if (module >= 0) {
      Serial.printf("Node %s Health: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                   moduleNames[module], data[0], data[1], data[2], data[3], 
                   data[4], data[5], data[6], data[7]);
    }
  }
  else if ((id & 0xFFFFFF00) == 0x18041200) {
    isKnownMessage = true;  // System config messages - ORIGINAL CODE UNCHANGED
    trackKnownMessage(id, "System Configuration", msg);  // Pass msg for data
    Serial.printf("System Config: %02X %02X %02X %02X %02X %02X %02X %02X\n", 
                 data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  
  // If unknown, log it - ORIGINAL CODE UNCHANGED
  if (!isKnownMessage) {
    logUnknownMessage(msg);
  }
  
  // Calculate derived values - ORIGINAL CODE UNCHANGED
  calculatePackStats();
}

void calculateModuleVoltages() {
  for (int m = 0; m < configNumModules; m++) {
    float sum = 0;
    bool valid = false;
    for (int c = 0; c < 8; c++) {
      float v = bmsData.cellVoltages[m * 8 + c];
      if (v > 0.1) {
        sum += v;
        valid = true;
      }
    }
    bmsData.moduleVoltages[m] = valid ? sum : 0.0;
  }
}


void logUnknownMessage(twai_message_t& msg) {
  // Check if we already have this message ID
  for (int i = 0; i < unknownCount; i++) {
    if (unknownMessages[i].id == msg.identifier) {
      // Update existing entry
      memcpy(unknownMessages[i].data, msg.data, msg.data_length_code);
      unknownMessages[i].length = msg.data_length_code;
      unknownMessages[i].count++;
      unknownMessages[i].lastSeen = millis();
      return;
    }
  }
  
  // Add new message if we have space
  if (unknownCount < 20) {
    unknownMessages[unknownCount].id = msg.identifier;
    memcpy(unknownMessages[unknownCount].data, msg.data, msg.data_length_code);
    unknownMessages[unknownCount].length = msg.data_length_code;
    unknownMessages[unknownCount].count = 1;
    unknownMessages[unknownCount].lastSeen = millis();
    unknownCount++;
    
    Serial.printf("NEW Unknown CAN ID: 0x%08X\n", msg.identifier);
  }
}

int getModuleIndex(uint8_t nodeId) {
  switch(nodeId) {
    case 0xAA: return 0;
    case 0xAB: return 1;
    case 0xAC: return 2;
    case 0xAD: return 3;
    case 0xAE: return 4;
    case 0xAF: return 5;
    case 0xB0: return 6;
    case 0xB1: return 7;
    default: return -1;
  }
}

void calculatePackStats() {
  float total = 0;
  float maxVoltage = 0;
  float minVoltage = 5.0;
  float maxTemp = -50;
  float minTemp = 100;
  
  // Calculate pack voltage and cell spread
  for (int i = 0; i < 64; i++) {
    if (bmsData.cellVoltages[i] > 0) {
      total += bmsData.cellVoltages[i];
      if (bmsData.cellVoltages[i] > maxVoltage) maxVoltage = bmsData.cellVoltages[i];
      if (bmsData.cellVoltages[i] < minVoltage) minVoltage = bmsData.cellVoltages[i];
    }
  }

  calculateModuleVoltages();

  
  for (int i = 0; i < configNumModules; i++) {
    if (bmsData.temperatures[i*2] > 0) {  // only if we have data
      if (bmsData.temperatures[i*2] > maxTemp) maxTemp = bmsData.temperatures[i*2];
      if (bmsData.temperatures[i*2 + 1] > maxTemp) maxTemp = bmsData.temperatures[i*2 + 1];
      if (bmsData.temperatures[i*2] < minTemp) minTemp = bmsData.temperatures[i*2];
      if (bmsData.temperatures[i*2 + 1] < minTemp) minTemp = bmsData.temperatures[i*2 + 1];
    }
  }

  
  // Calculate temperature range  // lmg i think this should be larger? check later 
  for (int i = 0; i < 8; i++) {
    if (bmsData.temperatures[i] > maxTemp) maxTemp = bmsData.temperatures[i];
    if (bmsData.temperatures[i] < minTemp) minTemp = bmsData.temperatures[i];
  }
  
  bmsData.packVoltage = total;
  bmsData.cellSpread = (maxVoltage - minVoltage) * 1000; // mV
  bmsData.maxTemp = maxTemp;
  bmsData.minTemp = minTemp;

}

String getBMSDataJSON() {
  DynamicJsonDocument doc(3072);
  
  doc["packVoltage"] = bmsData.packVoltage;
  doc["cellSpread"] = bmsData.cellSpread;
  doc["maxTemp"] = bmsData.maxTemp;
  doc["minTemp"] = bmsData.minTemp;
  doc["lastUpdate"] = bmsData.lastUpdate;
  
  // Cell voltages by module
  JsonArray modules = doc.createNestedArray("modules");
  for (int m = 0; m < configNumModules; m++) {  // ←←← CHANGED FROM 8
    JsonObject module = modules.createNestedObject();
    module["name"] = moduleNames[m];
    module["connected"] = bmsData.nodeStatus[m];
    module["voltage"] = bmsData.moduleVoltages[m];

    JsonArray cells = module.createNestedArray("cells");
    for (int c = 0; c < 8; c++) {
      cells.add(bmsData.cellVoltages[m*8 + c]);
    }

    JsonArray temps = module.createNestedArray("temperatures");
    temps.add(bmsData.temperatures[m*2]);
    temps.add(bmsData.temperatures[m*2 + 1]);
  }

  
  // Add unknown messages
  JsonArray unknowns = doc.createNestedArray("unknownMessages");
  for (int i = 0; i < unknownCount; i++) {
    JsonObject unknown = unknowns.createNestedObject();
    unknown["id"] = String(unknownMessages[i].id, HEX);
    unknown["count"] = unknownMessages[i].count;
    unknown["length"] = unknownMessages[i].length;
    
    JsonArray dataArray = unknown.createNestedArray("data");
    for (int j = 0; j < unknownMessages[i].length; j++) {
      dataArray.add(unknownMessages[i].data[j]);
    }
  }
  
  // Add known messages - NEW
  JsonArray knowns = doc.createNestedArray("knownMessages");
  for (int i = 0; i < knownCount; i++) {
    JsonObject known = knowns.createNestedObject();
    known["id"] = String(knownMessages[i].id, HEX);
    known["description"] = knownMessages[i].description;
    known["count"] = knownMessages[i].count;
    known["lastSeen"] = knownMessages[i].lastSeen;
    known["length"] = knownMessages[i].length;
    
    JsonArray dataArray = known.createNestedArray("data");
    for (int j = 0; j < knownMessages[i].length; j++) {
      dataArray.add(knownMessages[i].data[j]);
    }
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

void broadcastUpdate() {
  String jsonData = getBMSDataJSON();
  ws.textAll(jsonData);
}

String getConfigHTML() {
  uint8_t savedOctet = preferences.getUChar("static_octet", 100);  // preferences is a global object

  String dots = (homePass.length() > 0) ? String(homePass.length(), '•') : "none";

  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>ESP32 CAN Config</title>";
  html += "<style>body{font-family:Arial;text-align:center;margin:40px;background:#f4f4f9;}";
  html += "h1{color:#2c3e50;}.box{background:white;padding:20px;margin:20px auto;max-width:600px;border-radius:12px;box-shadow:0 4px 15px rgba(0,0,0,0.1);}";
  html += "select, input{width:90%;padding:12px;margin:10px;font-size:1em;}";
  html += "button{padding:15px 30px;margin:15px;font-size:1.1em;border:none;border-radius:8px;color:white;cursor:pointer;background:#27ae60;}";
  html += "</style></head><body>";

  html += "<h1>BMS CAN Bus Config</h1>";
  html += "<div class='box'><strong>AP:</strong> " + String(apSSID) + "<br>";
  html += "<strong>AP URL:</strong> http://" + WiFi.softAPIP().toString() + "</div>";

  if (WiFi.status() == WL_CONNECTED) {
    html += "<div class='box'>Connected to: " + WiFi.SSID() + "<br>IP: " + WiFi.localIP().toString() + "</div>";
  } else {
    html += "<div class='box'>Not connected to home WiFi</div>";
  }

  html += "<button onclick=\"document.getElementById('form').style.display='block';this.style.display='none';\">";
  html += "Configure WiFi & Modules</button>";

  html += "<div id='form' class='box' style='display:none;'>";
  html += "<form action='/save' method='POST'>";

  html += "WiFi SSID:<br><input type='text' name='ssid' value='" + homeSSID + "'><br>";
  html += "Password:<br><input type='password' name='pass' placeholder='Leave blank to keep current'><br>";
  html += "<small>Current: " + dots + " (" + String(homePass.length()) + " chars)</small><br><br>";

  html += "<label>Static IP Last Octet (192.168.1.xxx):</label><br>" ;
  html +=  "<input type='number' name='static_octet' min='2' max='254' value='" + String(savedOctet) + "'><br>" ;
  html += "<small>Current static IP will be 192.168.1." + String(savedOctet) + " after reboot</small><br><br>" ;


  html += "<label>Number of Battery Modules:</label><br>";
  html += "<select name='modules'>";
  html += "<option value='4'" + String(configNumModules == 4 ? " selected" : "") + ">4 Modules</option>";
  html += "<option value='6'" + String(configNumModules == 6 ? " selected" : "") + ">6 Modules</option>";
  html += "<option value='8'" + String(configNumModules == 8 ? " selected" : "") + ">8 Modules (default)</option>";
  html += "</select><br><br>";

  html += "<button type='submit'>Save & Restart</button>";
  html += "</form></div></body></html>";
  return html;
}



String getDashboardHTML() {
  return R"(
<!DOCTYPE html>
<html>
<head>
    <title>BMS Dashboard</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { 
            font-family: Arial, sans-serif; 
            margin: 20px; 
            background: #f0f0f0;
        }
        .header {
            background: #2c3e50;
            color: white;
            padding: 10px 20px;
            border-radius: 8px;
            margin-bottom: 15px;
        }
        .stats {
            display: flex;
            justify-content: space-around;
            margin-bottom: 20px;
        }
        .stat {
            background: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            text-align: center;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #27ae60;
        }
        .modules {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 30px;
        }
        .module {
            background: white;
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        .module-header {
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 10px;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .module-voltage {
            font-size: 16px;
            color: #2c3e50;
            font-weight: bold;
            margin-bottom: 8px;
            text-align: center;
            background: #ecf0f1;
            padding: 8px;
            border-radius: 4px;
        }
        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #e74c3c;
        }
        .status-dot.connected {
            background: #27ae60;
        }
        .cells {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 5px;
            margin-bottom: 10px;
        }
        .cell {
            padding: 8px;
            text-align: center;
            border-radius: 4px;
            font-size: 12px;
            color: white;
            background: #95a5a6;
        }
        .cell.good { background: #27ae60; }
        .cell.warning { background: #f39c12; }
        .cell.danger { background: #e74c3c; }
        .temps {
            font-size: 14px;
            color: #7f8c8d;
        }
        .unknown-section {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            margin-bottom: 20px;
        }
        .unknown-table {
            width: 100%;
            border-collapse: collapse;
            margin-top: 15px;
        }
        .unknown-table th,
        .unknown-table td {
            border: 1px solid #ddd;
            padding: 8px;
            text-align: left;
        }
        .unknown-table th {
            background-color: #f2f2f2;
            font-weight: bold;
        }
        .unknown-table tr:nth-child(even) {
            background-color: #f9f9f9;
        }
        .data-hex {
            font-family: monospace;
            font-size: 12px;
        }
        #connection-status {
            position: fixed;
            top: 10px;
            right: 10px;
            padding: 10px;
            border-radius: 4px;
            color: white;
            font-weight: bold;
        }
        .connected { background: #27ae60; }
        .disconnected { background: #e74c3c; }
    </style>
</head>
<body>
    <div id="connection-status" class="disconnected">Connecting...</div>
    
    <div class="header">
        <h1>BMS Dashboard </h1>
        <!-- Config Button - Top Right -->
     <button onclick="window.location.href='/config'" 
            style="position:absolute; top:68px; right:15px; 
                   width:56px; height:56px; 
                   background:#607d8b;            /* beautiful blue-grey */
                   color:white; 
                   border:none; 
                   border-radius:50%; 
                   font-size:26px; 
                   cursor:pointer; 
                   box-shadow:0 6px 16px rgba(0,0,0,0.25);
                   transition:all 0.25s ease;
                   z-index:10;"
            onmouseover="this.style.background='#546e7a'; this.style.transform='translateY(-3px) rotate(30deg)'"
            onmouseout="this.style.background='#607d8b'; this.style.transform='translateY(0) rotate(0)'"
            title="Configuration">
            &#9881;
    </button>   


    </div>
    
    <div class="stats">
        <div class="stat">
            <div>Pack Voltage</div>
            <div class="stat-value" id="pack-voltage">---</div>
        </div>
        <div class="stat">
            <div>Cell Spread</div>
            <div class="stat-value" id="cell-spread">---</div>
        </div>
        <div class="stat">
            <div>Temperature Range</div>
            <div class="stat-value" id="temp-range">---</div>
        </div>
        <div class="stat">
            <div>Last Update</div>
            <div class="stat-value" id="last-update">---</div>
        </div>
    </div>
    
    <div class="modules" id="modules">
        <!-- Modules will be populated by JavaScript -->
    </div>

    <div class="unknown-section">
        <h2>Unknown CAN Messages</h2>
        <p>Messages that don't match known protocol patterns:</p>
        <table class="unknown-table">
            <thead>
                <tr>
                    <th>CAN ID</th>
                    <th>Count</th>
                    <th>Length</th>
                    <th>Data (Hex)</th>
                </tr>
            </thead>
            <tbody id="unknown-table-body">
                <tr><td colspan="4">No unknown messages detected</td></tr>
            </tbody>
        </table>
    </div>

    <div class="unknown-section">
        <h2>Known CAN Messages</h2>
        <p>Messages we've identified and understand:</p>
        <table class="unknown-table">
            <thead>
                <tr>
                    <th>CAN ID</th>
                    <th>Description</th>
                    <th>Count</th>
                    <th>Length</th>
                    <th>Data (Hex)</th>
                    <th>Last Seen</th>
                </tr>
            </thead>
            <tbody id="known-table-body">
                <tr><td colspan="6">No known messages tracked yet</td></tr>
            </tbody>
        </table>
    </div>

    <script>
        let ws;
        let reconnectTimer;
        
        function connectWebSocket() {
            ws = new WebSocket('ws://' + window.location.host + '/ws');
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                document.getElementById('connection-status').textContent = 'Connected';
                document.getElementById('connection-status').className = 'connected';
                clearInterval(reconnectTimer);
            };
            
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    updateDashboard(data);
                } catch (e) {
                    console.error('Failed to parse data:', e);
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                document.getElementById('connection-status').textContent = 'Disconnected';
                document.getElementById('connection-status').className = 'disconnected';
                
                // Reconnect after 3 seconds
                reconnectTimer = setTimeout(connectWebSocket, 3000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function updateDashboard(data) {
            // Update summary stats
            document.getElementById('pack-voltage').textContent = data.packVoltage.toFixed(2) + 'V';
            document.getElementById('cell-spread').textContent = data.cellSpread.toFixed(0) + 'mV';
            document.getElementById('temp-range').textContent = 
                data.minTemp.toFixed(1) + 'F - ' + data.maxTemp.toFixed(1) + 'F';
            document.getElementById('last-update').textContent = 
                new Date(data.lastUpdate + Date.now() - performance.now()).toLocaleTimeString();
            
            // Update modules
            const modulesDiv = document.getElementById('modules');
            modulesDiv.innerHTML = '';
            
            data.modules.forEach(module => {
                const moduleDiv = document.createElement('div');
                moduleDiv.className = 'module';
                
                moduleDiv.innerHTML = 
                    '<div class="module-header">Module ' + module.name + 
                    '<div class="status-dot ' + (module.connected ? 'connected' : '') + '"></div></div>' +
                    '<div class="module-voltage">Pack Voltage: ' + module.voltage.toFixed(2) + 'V</div>' +
                    '<div class="cells">' +
                    module.cells.map((voltage, index) => {
                        let cellClass = 'good';
                        if (voltage < 3.0 || voltage > 3.4) cellClass = 'danger';
                        else if (voltage < 3.1 || voltage > 3.35) cellClass = 'warning';
                        
                        return '<div class="cell ' + cellClass + '">Cell ' + (index + 1) + '<br>' + voltage.toFixed(3) + 'V</div>';
                    }).join('') +
                    '</div>' +
                    '<div class="temps">Temperatures: ' + module.temperatures[0].toFixed(1) + 'F, ' + module.temperatures[1].toFixed(1) + 'F</div>';
                
                modulesDiv.appendChild(moduleDiv);
            });
            
            // Update unknown messages table
            const tableBody = document.getElementById('unknown-table-body');
            if (data.unknownMessages && data.unknownMessages.length > 0) {
                tableBody.innerHTML = data.unknownMessages.map(msg => {
                    const dataHex = msg.data.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
                    return '<tr>' +
                        '<td>0x' + msg.id.toUpperCase() + '</td>' +
                        '<td>' + msg.count + '</td>' +
                        '<td>' + msg.length + '</td>' +
                        '<td class="data-hex">' + dataHex + '</td>' +
                        '</tr>';
                }).join('');
            } else {
                tableBody.innerHTML = '<tr><td colspan="4">No unknown messages detected</td></tr>';
            }
            
            // Update known messages table
            const knownTableBody = document.getElementById('known-table-body');
            if (data.knownMessages && data.knownMessages.length > 0) {
                // Sort by CAN ID to prevent jumping around
                const sortedKnown = data.knownMessages.sort((a, b) => {
                    const idA = parseInt(a.id, 16);
                    const idB = parseInt(b.id, 16);
                    return idA - idB;
                });
                knownTableBody.innerHTML = sortedKnown.map(msg => {
                    const timeSince = Math.floor((Date.now() - (msg.lastSeen + Date.now() - performance.now())) / 1000);
                    let timeText;
                    if (timeSince < 0 || timeSince > 86400) {
                        timeText = 'Just now';
                    } else if (timeSince < 60) {
                        timeText = timeSince + 's ago';
                    } else if (timeSince < 3600) {
                        timeText = Math.floor(timeSince/60) + 'm ago';
                    } else {
                        timeText = Math.floor(timeSince/3600) + 'h ago';
                    }
                    
                    const dataHex = msg.data.map(b => b.toString(16).padStart(2, '0').toUpperCase()).join(' ');
                    
                    return '<tr>' +
                        '<td>0x' + msg.id.toUpperCase() + '</td>' +
                        '<td>' + msg.description + '</td>' +
                        '<td>' + msg.count + '</td>' +
                        '<td>' + msg.length + '</td>' +
                        '<td class="data-hex">' + dataHex + '</td>' +
                        '<td>' + timeText + '</td>' +
                        '</tr>';
                }).join('');
            } else {
                knownTableBody.innerHTML = '<tr><td colspan="6">No known messages tracked yet</td></tr>';
            }
        }
        
        // Start connection
        connectWebSocket();
        
        // Refresh page if no data for 30 seconds
        let lastDataTime = Date.now();
        setInterval(() => {
            if (Date.now() - lastDataTime > 30000) {
                location.reload();
            }
        }, 5000);
    </script>
</body>
</html>
)";
}