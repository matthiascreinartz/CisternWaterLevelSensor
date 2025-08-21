/*
 * Projekt: CisternWaterLevelSensor - Wasserstandsmessung per Ultraschallsensor + ESP32
 * ---------------------------------------------------------------------------
 * Beschreibung:
 *   Misst den Wasserstand einer Zisterne per Ultraschallsensor und veröffentlicht
 *   die Daten regelmäßig über MQTT (JSON-Format). Zusätzlich läuft ein Webserver
 *   für OTA-Updates und Debugging. 
 *
 * Features:
 *   - WLAN-Anbindung (SSID & Passwort seriell konfigurierbar, im Flash gespeichert)
 *   - MQTT-Client mit optionalem Benutzername/Passwort (Last Will & Testament: "offline")
 *   - JSON-Log mit Zeitstempel, Prozent- und Absolutwert sowie Trend
 *   - OTA-Update über Webinterface
 *   - Täglicher Reboot als Failsafe
 *   - Error-Logging auf MQTT-Topic "wasserstand/lastError"
 *
 * Wichtige Topics:
 *   - wasserstand/prozent
 *   - wasserstand/absolut
 *   - wasserstand/log
 *   - wasserstand/status
 *   - wasserstand/lastError
 *
 * Autor: Matthias Reinartz
 * Git:   https://gitlab.com/matthiascreinartz/CisternWaterLevelSensor
 * ---------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <NewPing.h>
#include <time.h>
#include <ArduinoJson.h>
#include <ElegantOTA.h>
#include <Preferences.h>
#include <esp_system.h>

// ---------------------------------------------------------------------------
// Globale Variablen und Konstanten
// ---------------------------------------------------------------------------

// ---- WLAN ----
String wifiSsid;
String wifiPassword;

// ---- MQTT ----
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
String mqttHost;
String mqttUsername;
String mqttPassword;

// ---- Messparameter ----
constexpr float DIST_EMPTY_MM   = 1390.0;   // Sensorabstand bei leerer Zisterne
constexpr float DIST_FULL_MM    = 240.0;    // Sensorabstand bei voller Zisterne
constexpr float US_TO_MM_FACTOR = 0.16868;  // Faktor für Umrechnung Schallgeschwindigkeit

// ---- Intervalle und Schwellwerte ----
constexpr unsigned long MEASUREMENT_INTERVAL_MS = 60000;    // jede Minute messen
constexpr unsigned long SEND_INTERVAL_MS        = 3600000;  // spätestens alle 1h senden
constexpr unsigned long MIN_CHANGE_MM           = 5;        // mindestens 5mm Änderung für Senden

// ---- Trend-Erkennung ----
constexpr int HISTORY_SIZE = 60;  // letzte 60 Werte (ca. 1h)
float historyValues[HISTORY_SIZE];
unsigned long historyTimestamps[HISTORY_SIZE];
int historyIndex = 0;
int historyCount = 0;

// ---- OTA ----
WebServer otaServer(80);
bool otaInProgress = false;

// ---- Ultraschallsensor ----
#define TRIG_PIN 5
#define ECHO_PIN 18
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

// ---- Zeitserver ----
const char* NTP_SERVER          = "pool.ntp.org";
const long GMT_OFFSET_SEC       = 3600;
const int DAYLIGHT_OFFSET_SEC   = 3600;

// ---- Laufzeitvariablen ----
unsigned long lastMeasurementTime = 0;
unsigned long lastSendTime        = 0;
float lastSentDistanceMM          = 0;
String lastSentTrend              = "";
bool rebootDoneToday              = false;
unsigned long lastMQTTReconnect   = 0;

Preferences preferences;

// ---------------------------------------------------------------------------
// Hilfsfunktionen
// ---------------------------------------------------------------------------

/**
 * Liest eine Eingabezeile von der seriellen Schnittstelle ein.
 */
String readSerialLine() {
  String input;
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input.length() > 0) break;
      } else {
        input += c;
      }
    }
  }
  input.trim();
  return input;
}

/**
 * Fügt einen Wert in die Verlaufsliste für Trenderkennung ein.
 */
void addToHistory(float value) {
  historyValues[historyIndex] = value;
  historyTimestamps[historyIndex] = millis();
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

/**
 * Berechnet den Trend mittels linearer Regression (Steigung mm/min).
 */
float calculateTrendSlope() {
  if (historyCount < 5) return 0; // zu wenig Daten

  double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  for (int i = 0; i < historyCount; i++) {
    double x = (historyTimestamps[i] - historyTimestamps[0]) / 60000.0; // Minuten
    double y = historyValues[i];
    sumX  += x;
    sumY  += y;
    sumXY += x * y;
    sumXX += x * x;
  }

  double denom = historyCount * sumXX - sumX * sumX;
  if (denom == 0) return 0;

  return (historyCount * sumXY - sumX * sumY) / denom;
}

/**
 * Liefert Trend als String ("einlaufend", "abnehmend", "stabil").
 */
String getTrendLabel() {
  float slope = calculateTrendSlope();
  if (slope < -0.1) return "einlaufend";
  if (slope > 0.1)  return "abnehmend";
  return "stabil";
}

/**
 * Gibt aktuellen Zeitstempel zurück.
 */
String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "1970-01-01T00:00:00";
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buffer);
}

/**
 * Entfernt Zeitstempel aus einer Fehlermeldung.
 */
String stripTimestamp(const String &msg) {
  int pos = msg.indexOf(" -- ");
  return (pos >= 0) ? msg.substring(pos + 4) : msg;
}

/**
 * Stellt sicher, dass WLAN verbunden ist.
 */
void ensureWifiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifiSsid, wifiPassword);
    while (WiFi.status() != WL_CONNECTED) {
      delay(300);
      Serial.print(".");
    }
    Serial.printf("\nWLAN verbunden: %s\n", WiFi.localIP().toString().c_str());
  }
}

/**
 * Baut MQTT-Verbindung auf (synchron, blockierend).
 */
void connectMQTT() {
  mqttClient.setServer(mqttHost.c_str(), 1883);

  while (!mqttClient.connected()) {
    Serial.print("Verbinde MQTT...");
    bool connected = false;

    if (!mqttUsername.isEmpty() && !mqttPassword.isEmpty()) {
      connected = mqttClient.connect(
        "CisternSensor",
        mqttUsername.c_str(), mqttPassword.c_str(),
        "wasserstand/status", 0, true, "offline"
      );
    } else {
      connected = mqttClient.connect(
        "CisternSensor",
        "wasserstand/status", 0, true, "offline"
      );
    }

    if (connected) {
      Serial.println("✅ verbunden");
      mqttClient.publish("wasserstand/status", "online", true);
    } else {
      Serial.printf(" ❌ Fehler, rc=%d\n", mqttClient.state());
      delay(2000);
    }
  }
}

/**
 * Versucht MQTT-Verbindung aufrechtzuerhalten.
 */
void ensureMQTTConnected() {
  if (!mqttClient.connected() && millis() - lastMQTTReconnect > 10000) {
    lastMQTTReconnect = millis();
    Serial.println("🔌 Versuche MQTT-Reconnect...");
    connectMQTT();
  }
  mqttClient.loop();
}

/**
 * Veröffentlicht Nachricht über MQTT.
 */
void publishMQTT(const char* topic, const String& payload, bool retained = true) {
  if (mqttClient.connected()) {
    mqttClient.publish(topic, payload.c_str(), retained);
  }
}

/**
 * Fehler im Flash speichern und über MQTT publizieren.
 */
void logError(const String &err) {
  String payload = getTimestamp() + " -- " + err;
  preferences.begin("errorlog", false);
  preferences.putString("lastError", payload);
  preferences.end();
  publishMQTT("wasserstand/lastError", payload);
}

/**
 * Letzten gespeicherten Fehler lesen.
 */
String readLastError() {
  preferences.begin("errorlog", true);
  String err = preferences.getString("lastError", "Kein Fehler gespeichert");
  preferences.end();
  return err;
}

/**
 * Entscheidet, ob aktuelle Messung gesendet werden sollte.
 */
bool shouldSendMeasurement(float currentMM, const String& trend) {
  bool intervalExceeded = millis() - lastSendTime >= SEND_INTERVAL_MS;
  bool changeExceeded   = fabs(currentMM - lastSentDistanceMM) >= MIN_CHANGE_MM;
  bool trendChanged     = (trend != lastSentTrend);
  return intervalExceeded || changeExceeded || trendChanged;
}

/**
 * Führt eine Messung aus und sendet Ergebnisse bei Bedarf.
 */
void performMeasurement(bool forceSend = false) {
  float duration = sonar.ping_median(5);
  float distanceMM = duration * US_TO_MM_FACTOR;

  if (distanceMM <= 200 || distanceMM > 2000) {
    logError("Sensorfehler: " + String(distanceMM) + " mm");
    return;
  }

  float percentage = (DIST_EMPTY_MM - distanceMM) / (DIST_EMPTY_MM - DIST_FULL_MM) * 100.0;
  percentage = constrain(percentage, 0, 100);

  addToHistory(distanceMM);
  String trend = getTrendLabel();

  if (forceSend || shouldSendMeasurement(distanceMM, trend)) {
    lastSentDistanceMM = distanceMM;
    lastSendTime       = millis();
    lastSentTrend      = trend;

    publishMQTT("wasserstand/prozent", String((int)percentage));
    publishMQTT("wasserstand/absolut", String((int)round(distanceMM)));

    String json = "{\"timestamp\":\"" + getTimestamp() +
                  "\",\"prozent\":" + String(percentage, 1) +
                  ",\"absolut\":" + String((int)round(distanceMM)) + 
                  ",\"trend\":\"" + trend + "\"}";

    String state = "OFF";
    if (trend == "einlaufend") {
      state = "HEAT";
      publishMQTT("wasserstand/getTargetTemperature", "100");
    } else if (trend == "abnehmend") {
      state = "COOL";
      publishMQTT("wasserstand/getTargetTemperature", "0");
    }
    publishMQTT("wasserstand/log", json);
    publishMQTT("wasserstand/fuellt", (trend == "einlaufend") ? "true" : "false");
    publishMQTT("wasserstand/matchCoolingState", state);
  }
}

/**
 * Führt täglichen Reboot um 02:00 Uhr aus.
 */
void checkDailyReboot() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    if (timeinfo.tm_hour == 2 && timeinfo.tm_min == 0 && !rebootDoneToday) {
      logError("Geplanter täglicher Neustart");
      rebootDoneToday = true;
      ESP.restart();
    }
    if (timeinfo.tm_hour != 2) rebootDoneToday = false;
  }
}

/**
 * Reset-Grund als String zurückgeben.
 */
String getResetReasonString() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:    return "Power-On Reset";
    case ESP_RST_EXT:        return "Externer Reset";
    case ESP_RST_SW:         return "Software Reset";
    case ESP_RST_PANIC:      return "Guru Meditation";
    case ESP_RST_INT_WDT:    return "Interrupt Watchdog";
    case ESP_RST_TASK_WDT:   return "Task Watchdog";
    case ESP_RST_WDT:        return "Anderer Watchdog";
    case ESP_RST_DEEPSLEEP:  return "Wake from Deep-Sleep";
    case ESP_RST_BROWNOUT:   return "Brownout";
    case ESP_RST_SDIO:       return "SDIO Reset";
    default:                 return "Unbekannter Reset";
  }
}

// ---------------------------------------------------------------------------
// Setup & Loop
// ---------------------------------------------------------------------------

/**
 * Konfigurationsdaten seriell einlesen und im Flash speichern.
 */
void configSetup() {
  preferences.begin("config", false);

  Serial.println("⚙️  Neue Konfiguration eingeben:");

  Serial.print("WLAN SSID: ");
  preferences.putString("wifi_ssid", readSerialLine());

  Serial.print("WLAN Passwort: ");
  preferences.putString("wifi_pass", readSerialLine());

  Serial.print("MQTT Host/IP: ");
  preferences.putString("mqtt_host", readSerialLine());

  Serial.print("MQTT Benutzer (leer lassen wenn keiner): ");
  preferences.putString("mqtt_user", readSerialLine());

  Serial.print("MQTT Passwort (leer lassen wenn keiner): ");
  preferences.putString("mqtt_pass", readSerialLine());

  preferences.end();

  Serial.println("✅ Konfiguration gespeichert! Bitte Reset durchführen.");
  while (true) { delay(1000); }
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Konfiguration laden
  preferences.begin("config", false);
  wifiSsid      = preferences.getString("wifi_ssid", "");
  wifiPassword  = preferences.getString("wifi_pass", "");
  mqttHost      = preferences.getString("mqtt_host", "");
  mqttUsername  = preferences.getString("mqtt_user", "");
  mqttPassword  = preferences.getString("mqtt_pass", "");
  preferences.end();

  if (wifiSsid.isEmpty() || wifiPassword.isEmpty() || mqttHost.isEmpty()) {
    Serial.println("⚠️  Keine gültige Konfiguration gefunden.");
    configSetup();
  } else {
    Serial.println("✅ Konfiguration geladen.");
    Serial.printf("SSID: %s, MQTT Host: %s\n", wifiSsid.c_str(), mqttHost.c_str());
    Serial.println("👉 Tippe 'resetconfig' oder 'setconfig' um Einstellungen zu ändern.");
  }

  // WLAN & MQTT verbinden
  ensureWifiConnected();
  connectMQTT();

  // OTA-Setup
  ElegantOTA.onStart([]() {
    otaInProgress = true;
    logError("OTA Update gestartet – stoppe Messungen.");
  });
  ElegantOTA.onEnd([](bool success) {
    otaInProgress = false;
    logError(success ? "OTA erfolgreich – Neustart" : "OTA fehlgeschlagen");
    if (success) { delay(1000); ESP.restart(); }
  });
  ElegantOTA.begin(&otaServer);
  otaServer.on("/", []() { otaServer.sendHeader("Location", "/update"); otaServer.send(302, ""); });
  otaServer.begin();

  // NTP-Setup
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  // Letzten gespeicherten Fehler ggf. melden
  String resetMsg = "Reset-Grund: " + getResetReasonString();
  String lastErr  = readLastError();

  String lastErrCore  = stripTimestamp(lastErr);
  String resetMsgCore = resetMsg;

  if (lastErr != "Kein Fehler gespeichert" &&
      lastErrCore != resetMsgCore &&
      lastErrCore != "OTA erfolgreich – Neustart" &&
      lastErrCore != "OTA Update gestartet – stoppe Messungen." &&
      lastErrCore != "OTA fehlgeschlagen") {
    publishMQTT("wasserstand/lastError", lastErr);
  }

  preferences.begin("errorlog", false);
  preferences.remove("lastError");
  preferences.end();

  logError(resetMsg);

  // Initialmessung
  performMeasurement(true);
  lastMeasurementTime = millis();
}

void loop() {
  otaServer.handleClient();

  if (!otaInProgress) {
    ensureWifiConnected();
    ensureMQTTConnected();

    if (millis() - lastMeasurementTime >= MEASUREMENT_INTERVAL_MS) {
      lastMeasurementTime = millis();
      if (mqttClient.connected()) {
        performMeasurement();
      } else {
        logError("Messung übersprungen – MQTT offline");
      }
    }
  }
  checkDailyReboot();
}
