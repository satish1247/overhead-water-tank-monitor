/*
 * Overhead Water Tank Monitor
 * Board  : ESP32-S3 (Arduino core 3.x or 2.x)
 * Sensor : Ultrasonic (HC-SR04 / HC-SR04P / JSN-SR04T) pointing DOWN at the water
 *
 * What it does
 *  - Measures the distance from the sensor to the water surface.
 *  - Converts that into level (cm), percentage, volume (litres) and flow (L/min).
 *  - Prints everything to the Serial Monitor once per second.
 *  - Serves a live web page on the WiFi network it joins (station mode only).
 *  - Tank height and shape are set from the web page and saved in flash (NVS),
 *    so they survive a power cut.
 *
 * Important note about "flow"
 *  An ultrasonic sensor only sees the water surface. It cannot measure real
 *  pipe flow. The flow shown here is the NET change of water volume per minute
 *  (inflow minus outflow):  + = tank filling,  - = tank draining.
 *
 * Wiring: see README.md (the Echo pin needs a 5V -> 3.3V divider on most sensors).
 */

#include <WiFi.h>
#include <atomic>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include "index_html.h"
#include "LevelFilter.h"

// ===================== USER SETTINGS =====================
const char* WIFI_SSID = "vivo T4x 5G";
const char* WIFI_PASS = "123456789";

const char* HOST_NAME = "watertank";       // opens as http://watertank.local

const uint8_t TRIG_PIN = 5;
const uint8_t ECHO_PIN = 18;
// =========================================================

// ---- Measurement tuning ----
const uint32_t PING_INTERVAL_MS   = 100;    // one ultrasonic reading every 100 ms
const uint32_t MEASURE_INTERVAL_MS = 250;   // recalculate level 4x a second so the
                                            // dashboard reacts to a pour in under a second
const uint32_t UPDATE_INTERVAL_MS = 1000;   // Serial print every 1 s
const uint8_t  SAMPLE_COUNT       = 9;      // median of the last 9 readings
const uint8_t  MIN_VALID_SAMPLES  = 5;      // need at least this many good readings
const uint32_t ECHO_TIMEOUT_US    = 30000;  // 30 ms = about 5 m
const float    MIN_VALID_CM       = 2.0f;
const float    MAX_VALID_CM       = 400.0f;
// ---- Tank limits accepted from the website ----
const float MIN_TANK_CM = 3.0f;
const float MAX_TANK_CM = 400.0f;

WebServer   server(80);
Preferences prefs;

struct TankConfig {
  float   heightCm;  // actual usable tank height
  float   emptyDistanceCm; // sensor face to bottom (includes mounting gap)
  bool    calibrated;
  uint8_t shape;     // 0 = cylinder, 1 = rectangle
  float   dim1Cm;    // cylinder: diameter, rectangle: length
  float   dim2Cm;    // rectangle: width (unused for cylinder)
};
TankConfig cfg;

// ---- Live measurement state ----
float samples[SAMPLE_COUNT];
uint8_t sampleIdx = 0;

bool  sensorOk  = false;
bool  haveLevel = false;
bool  flowReady = false;
float distanceCm = 0, levelCm = 0, percent = 0;
float volumeL = 0, capacityL = 0, flowLpm = 0, flowShown = 0;

LevelFilter levelFilter;
uint32_t measurementStarted = 0;

bool stationWasConnected = false;
uint32_t lastPing = 0, lastMeasure = 0, lastUpdate = 0, lastReconnect = 0;

// ---------------------------------------------------------
float tankAreaCm2() {
  if (cfg.shape == 0) {
    float r = cfg.dim1Cm / 2.0f;
    return PI * r * r;
  }
  return cfg.dim1Cm * cfg.dim2Cm;
}

void loadConfig() {
  prefs.begin("tank", true);
  cfg.heightCm = prefs.getFloat("h", 100.0f);
  cfg.shape    = prefs.getUChar("shape", 0);
  cfg.dim1Cm   = prefs.getFloat("d1", 50.0f);
  cfg.dim2Cm   = prefs.getFloat("d2", 0.0f);
  cfg.emptyDistanceCm = prefs.getFloat("empty", cfg.heightCm);
  cfg.calibrated = prefs.getBool("verified", false);
  prefs.end();

  // Safety net against corrupted values.
  if (!(cfg.heightCm >= MIN_TANK_CM && cfg.heightCm <= MAX_TANK_CM)) cfg.heightCm = 100.0f;
  if (cfg.shape > 1) cfg.shape = 0;
  if (!(cfg.dim1Cm >= 1.0f)) cfg.dim1Cm = 50.0f;
  if (cfg.shape == 1 && !(cfg.dim2Cm >= 1.0f)) cfg.dim2Cm = 50.0f;
  if (!isfinite(cfg.emptyDistanceCm) || cfg.emptyDistanceCm < cfg.heightCm || cfg.emptyDistanceCm > MAX_TANK_CM) {
    cfg.emptyDistanceCm = cfg.heightCm;
    cfg.calibrated = false;
  }
}

void saveConfig() {
  prefs.begin("tank", false);
  prefs.putFloat("h", cfg.heightCm);
  prefs.putUChar("shape", cfg.shape);
  prefs.putFloat("d1", cfg.dim1Cm);
  prefs.putFloat("d2", cfg.dim2Cm);
  prefs.putFloat("empty", cfg.emptyDistanceCm);
  prefs.putBool("verified", cfg.calibrated);
  prefs.end();
}

// Clears derived state only. The ping ring buffer is deliberately left alone:
// it refills on its own, and wiping it here would starve medianDistance() of
// the MIN_VALID_SAMPLES it needs, since updateMeasurement() runs faster than
// the buffer takes to fill.
void resetDerived() {
  sensorOk = haveLevel = flowReady = false;
  flowLpm = flowShown = 0;
  levelFilter.reset();
}

// Full reset, including the raw echoes. Only for a calibration change, where
// readings taken against the old dimensions must not survive.
void resetMeasurement() {
  resetDerived();
  for (uint8_t i = 0; i < SAMPLE_COUNT; ++i) samples[i] = -1.0f;
}

// ---------------------------------------------------------
// One ultrasonic reading in cm, or -1 if there was no valid echo.
float pingCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1.0f;

  float cm = duration * 0.01715f;   // speed of sound ~343 m/s, round trip
  if (cm < MIN_VALID_CM || cm > MAX_VALID_CM) return -1.0f;
  return cm;
}

// Median of the valid readings: ignores ripple spikes and failed pings.
bool medianDistance(float &out) {
  float tmp[SAMPLE_COUNT];
  uint8_t n = 0;
  for (uint8_t i = 0; i < SAMPLE_COUNT; i++) {
    if (samples[i] > 0) tmp[n++] = samples[i];
  }
  if (n < MIN_VALID_SAMPLES) return false;

  for (uint8_t i = 1; i < n; i++) {           // insertion sort
    float v = tmp[i];
    int8_t j = i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  out = tmp[n / 2];
  return true;
}

void updateMeasurement() {
  float d;
  if (!medianDistance(d) || (cfg.calibrated && d > cfg.emptyDistanceCm + 1.0f)) {
    resetDerived();
    return;
  }
  sensorOk = true;
  distanceCm = d; // Keep the actual median echo visible for calibration.
  if (!cfg.calibrated) return;
  if (!haveLevel) { measurementStarted = millis(); haveLevel = true; }
  const float area = tankAreaCm2();
  const float rawLevel = constrain(cfg.emptyDistanceCm - d, 0.0f, cfg.heightCm);
  levelFilter.update(rawLevel, millis(), area);
  levelCm = constrain(levelFilter.height, 0.0f, cfg.heightCm);
  percent = levelCm / cfg.heightCm * 100.0f;
  volumeL = area * levelCm / 1000.0f;
  capacityL = area * cfg.heightCm / 1000.0f;
  flowReady = millis() - measurementStarted >= 2000;
  flowLpm = flowShown = flowReady ? levelFilter.flow : 0;
}

void printSerial() {
  if (!sensorOk) {
    Serial.println("Sensor error: no valid echo. Check wiring and sensor position.");
    return;
  }
  if (!cfg.calibrated) {
    Serial.printf("CALIBRATION REQUIRED. Sensor distance: %.2f cm. Confirm dimensions on dashboard.\n", distanceCm);
    return;
  }
  char flowText[40];
  if (flowReady) {
    const char* trend = flowShown > 0 ? "filling" : (flowShown < 0 ? "draining" : "steady");
    snprintf(flowText, sizeof flowText, "%+.2f L/min (%s)", flowShown, trend);
  } else {
    snprintf(flowText, sizeof flowText, "measuring...");
  }
  Serial.printf("Distance: %.1f cm | Level: %.1f cm | Full: %.1f %% | Volume: %.1f / %.0f L | Flow: %s\n",
                distanceCm, levelCm, percent, volumeL, capacityL, flowText);
}

// ---------------------------------------------------------
void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  char derived[240];
  if (sensorOk && cfg.calibrated) {
    snprintf(derived, sizeof derived,
      "\"level\":%.2f,\"percent\":%.1f,\"volume\":%.3f,\"capacity\":%.3f,\"flow\":%.3f",
      levelCm, percent, volumeL, capacityL, flowShown);
  } else {
    snprintf(derived, sizeof derived, "\"level\":null,\"percent\":null,\"volume\":null,\"capacity\":null,\"flow\":null");
  }
  char distance[24];
  if (sensorOk) snprintf(distance, sizeof distance, "%.2f", distanceCm);
  else snprintf(distance, sizeof distance, "null");
  char json[700];
  snprintf(json, sizeof json,
    "{\"build\":\"sta-only-v4\",\"ok\":%s,\"calibrated\":%s,\"distance\":%s,%s,"
    "\"flowReady\":%s,\"trend\":%d,\"height\":%.1f,\"emptyDistance\":%.2f,\"shape\":%u,\"d1\":%.1f,\"d2\":%.1f}",
    sensorOk ? "true" : "false", cfg.calibrated ? "true" : "false", distance, derived,
    flowReady ? "true" : "false", (int)(sensorOk && cfg.calibrated ? levelFilter.trend : 0),
    cfg.heightCm, cfg.emptyDistanceCm, (unsigned)cfg.shape, cfg.dim1Cm, cfg.dim2Cm);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void sendConfigError(const char* message) {
  char json[160];
  snprintf(json, sizeof json, "{\"ok\":false,\"error\":\"%s\"}", message);
  server.send(400, "application/json", json);
}

void handleConfig() {
  if (!server.hasArg("height") || !server.hasArg("shape") || !server.hasArg("d1") || !server.hasArg("emptyDistance")) {
    sendConfigError("Missing fields");
    return;
  }
  float emptyDistance = server.arg("emptyDistance").toFloat();
  float  h     = server.arg("height").toFloat();
  int    shape = server.arg("shape").toInt();
  float  d1    = server.arg("d1").toFloat();
  float  d2    = server.hasArg("d2") ? server.arg("d2").toFloat() : 0.0f;

  if (!(h >= MIN_TANK_CM && h <= MAX_TANK_CM)) { sendConfigError("Tank height must be 3 to 400 cm"); return; }
  if (shape != 0 && shape != 1)                { sendConfigError("Unknown tank shape"); return; }
  if (!(d1 >= 1.0f && d1 <= 2000.0f))          { sendConfigError("Diameter or length must be 1 to 2000 cm"); return; }
  if (shape == 1 && !(d2 >= 1.0f && d2 <= 2000.0f)) { sendConfigError("Width must be 1 to 2000 cm"); return; }

  if (!isfinite(emptyDistance) || emptyDistance < h || emptyDistance > MAX_TANK_CM) {
    sendConfigError("Sensor-to-bottom distance must be at least tank height and at most 400 cm"); return;
  }
  cfg.emptyDistanceCm = emptyDistance;
  cfg.calibrated = true;
  cfg.heightCm = h;
  cfg.shape    = (uint8_t)shape;
  cfg.dim1Cm   = d1;
  cfg.dim2Cm   = (shape == 1) ? d2 : 0.0f;
  saveConfig();
  resetMeasurement();

  Serial.printf("New tank settings saved: height %.1f cm, %s, dim1 %.1f cm, dim2 %.1f cm\n",
                cfg.heightCm, cfg.shape == 0 ? "cylinder" : "rectangle", cfg.dim1Cm, cfg.dim2Cm);
  server.send(200, "application/json", "{\"ok\":true}");
}

// ---------------------------------------------------------

// Wi-Fi diagnostics: event callbacks run on another task, hence atomic state.
std::atomic<int> lastStationDisconnectReason{0};
std::atomic<bool> stationAssociated{false};

const char* stationReasonText(int reason) {
  switch (reason) {
    case 0: return "No disconnect recorded";
    case 15: case 204: return "Security handshake timed out";
    case 200: return "Router signal lost";
    case 201: return "Network not found by ESP32";
    case 202: return "Authentication failed; verify password and router access policy";
    case 203: return "Router association failed";
    case 210: case 211: return "No access point with compatible security";
    case 212: return "Signal below connection threshold";
    default: return "See numeric ESP32 disconnect reason";
  }
}

void handleWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_CONNECTED) {
    stationAssociated.store(true);
    Serial.println("[STA] Associated with router; waiting for DHCP IP address.");
  } else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    lastStationDisconnectReason.store(0);
    Serial.println("[STA] IP address acquired. Connection successful.");
  } else if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    stationAssociated.store(false);
    const int reason = info.wifi_sta_disconnected.reason;
    lastStationDisconnectReason.store(reason);
    Serial.printf("[STA] Disconnected: reason=%d (%s)\n", reason, stationReasonText(reason));
  }
}

// Scan result for the target router, refreshed before each join attempt.
struct RouterScan {
  bool    found = false;
  uint8_t channel = 0;
  int8_t  rssi = 0;
  uint8_t authMode = 0;
  uint8_t visibleNetworks = 0;
};
RouterScan routerScan;
uint8_t joinAttempts = 0;

const char* authModeText(uint8_t mode) {
  switch (mode) {
    case WIFI_AUTH_OPEN:            return "Open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-Enterprise (not supported by this sketch)";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/WPA3";
    default:                        return "Unknown";
  }
}

// Finds the router's channel and security so the shared radio can be aligned.
void scanForRouter() {
  RouterScan result;
  const int found = WiFi.scanNetworks(false, true);
  if (found <= 0) {
    Serial.println("[STA] Scan found no networks. Check antenna and 2.4 GHz availability.");
    routerScan = result;
    return;
  }
  result.visibleNetworks = (uint8_t)min(found, 255);
  Serial.printf("[STA] Scan found %d networks:\n", found);
  for (int i = 0; i < found; i++) {
    const String ssid = WiFi.SSID(i);
    Serial.printf("   %-24s ch%-3d %4d dBm  %s\n", ssid.c_str(), WiFi.channel(i),
                  WiFi.RSSI(i), authModeText((uint8_t)WiFi.encryptionType(i)));
    if (ssid == WIFI_SSID && (!result.found || WiFi.RSSI(i) > result.rssi)) {
      result.found    = true;
      result.channel  = (uint8_t)WiFi.channel(i);
      result.rssi     = (int8_t)WiFi.RSSI(i);
      result.authMode = (uint8_t)WiFi.encryptionType(i);
    }
  }
  WiFi.scanDelete();
  if (result.found) {
    Serial.printf("[STA] '%s' is on channel %u, %d dBm, security %s\n",
                  WIFI_SSID, result.channel, result.rssi, authModeText(result.authMode));
  } else {
    Serial.printf("[STA] '%s' was not in the scan. The ESP32 only sees 2.4 GHz networks.\n", WIFI_SSID);
  }
  routerScan = result;
}

void joinRouter() {
  joinAttempts++;
  WiFi.disconnect(true, false);
  delay(100);
  if (routerScan.found) {
    // Pinning the channel skips the blind scan inside begin(), which makes the
    // join faster and more reliable on busy guest networks.
    WiFi.begin(WIFI_SSID, WIFI_PASS, routerScan.channel);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
  Serial.printf("[STA] Join attempt %u to '%s'%s\n", joinAttempts, WIFI_SSID,
                routerScan.found ? " on its scanned channel" : " (channel unknown)");
}

void handleWiFiStatus() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  const int reason = lastStationDisconnectReason.load();
  char response[900];
  snprintf(response, sizeof response,
    "{\"build\":\"sta-only-v4\",\"mode\":\"STA\",\"ssid\":\"%s\","
    "\"connected\":%s,\"stage\":\"%s\",\"statusCode\":%d,\"stationIP\":\"%s\","
    "\"stationMAC\":\"%s\",\"lastDisconnectReason\":%d,\"reason\":\"%s\","
    "\"joinAttempts\":%u,\"routerFound\":%s,\"routerChannel\":%u,\"routerRSSI\":%d,"
    "\"routerSecurity\":\"%s\",\"visibleNetworks\":%u}",
    WIFI_SSID,
    connected ? "true" : "false",
    connected ? "connected" : stationAssociated.load() ? "waiting_for_ip" : "connecting",
    (int)WiFi.status(), connected ? WiFi.localIP().toString().c_str() : "0.0.0.0",
    WiFi.macAddress().c_str(), reason, stationReasonText(reason),
    (unsigned)joinAttempts, routerScan.found ? "true" : "false", (unsigned)routerScan.channel,
    (int)routerScan.rssi, authModeText(routerScan.authMode), (unsigned)routerScan.visibleNetworks);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", response);
}

void startWiFi() {
  WiFi.persistent(false);
  WiFi.onEvent(handleWiFiEvent);
  WiFi.mode(WIFI_STA);                        // station only, no local hotspot
  WiFi.setHostname(HOST_NAME);
  WiFi.setSleep(false);                       // modem sleep drops weak guest links
  WiFi.setMinSecurity(WIFI_AUTH_OPEN);        // accept open and WPA-only guest networks
  WiFi.setAutoReconnect(true);

  scanForRouter();
  joinRouter();
  Serial.printf("Station mode only. The dashboard comes up once '%s' hands out an IP.\n", WIFI_SSID);
  if (MDNS.begin(HOST_NAME)) MDNS.addService("http", "tcp", 80);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== Overhead Water Tank Monitor ===");

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  for (uint8_t i = 0; i < SAMPLE_COUNT; i++) samples[i] = -1.0f;

  loadConfig();
  Serial.printf("Tank: height %.1f cm, %s, dim1 %.1f cm, dim2 %.1f cm\n",
                cfg.heightCm, cfg.shape == 0 ? "cylinder" : "rectangle", cfg.dim1Cm, cfg.dim2Cm);

  Serial.println("[BUILD] sta-only-v4 " __DATE__ " " __TIME__);
  startWiFi();

  server.on("/api/wifi", HTTP_GET, handleWiFiStatus);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_POST, handleConfig);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();
  Serial.println("Web server started.");
}

void loop() {
  server.handleClient();
  uint32_t now = millis();

  if (now - lastPing >= PING_INTERVAL_MS) {
    lastPing = now;
    samples[sampleIdx] = pingCm();
    sampleIdx = (sampleIdx + 1) % SAMPLE_COUNT;
  }

  if (now - lastMeasure >= MEASURE_INTERVAL_MS) {
    lastMeasure = now;
    updateMeasurement();
  }

  if (now - lastUpdate >= UPDATE_INTERVAL_MS) {
    lastUpdate = now;
    printSerial();
  }

  const bool stationConnected = WiFi.status() == WL_CONNECTED;
  if (stationConnected && !stationWasConnected) {
    Serial.printf("Connected to '%s'. Dashboard: http://%s or http://%s.local\n",
                  WIFI_SSID, WiFi.localIP().toString().c_str(), HOST_NAME);
  } else if (!stationConnected && stationWasConnected) {
    Serial.println("Router disconnected. The dashboard is offline until the link returns.");
  }
  stationWasConnected = stationConnected;

  // Keep retrying. Every third attempt rescans, in case the router changed
  // channel or only just came within range.
  if (!stationConnected && now - lastReconnect >= 20000) {
    lastReconnect = now;
    Serial.printf("[STA] Not connected (status=%d). Retrying.\n", (int)WiFi.status());
    if (joinAttempts % 3 == 0) scanForRouter();
    joinRouter();
  }
}
