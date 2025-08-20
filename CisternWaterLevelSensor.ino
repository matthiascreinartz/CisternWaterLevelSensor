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
 *   - JSON-Log mit Zeitstempel, Prozent- und Absolutwert
 *   - OTA Update über Webinterface
 *   - Täglicher Reboot als Failsafe
 *   - Error-Logging auf MQTT-Topic "wasserstand/lastError"
 *
 * Wichtige Topics:
 *   - wasserstand/prozent   → Füllstand in %
 *   - wasserstand/log       → JSON mit timestamp, prozent, absolut, trend
 *   - wasserstand/status    → online/offline (Last Will)
 *   - wasserstand/lastError → letzte Fehlermeldung (inkl. Reset-Grund, OTA usw.)
 *
 * Developer Notes:
 *   - WiFi & MQTT Reconnect sind non-blocking (damit OTA/Webserver auch bei
 *     Verbindungsproblemen immer erreichbar bleiben).
 *   - Konfiguration (SSID, Passwort, MQTT Host/User/Pass) wird über Serial eingegeben
 *     und im Flash (Preferences) gespeichert.
 *   - Messungen laufen nur, wenn MQTT verbunden ist – andernfalls wird ein Error geloggt.
 *
 * Hardware:
 *   - ESP32 NodeMCU Development Board
 *   - JSN-SR04T-2.0 Wasserfester Ultraschallsensor
 *
 * Weitere Info:
 *   - Die gewonnen Daten verarbeite ich auf meinem RaspberryPi weiter:
 *     - Jede neue JSON Message wird in einer monatlich generierten CSV geloggt
 *     - Auf dem RasperryPi läuft Homebridge mit dem Plugin MQTTTHing. So habe ich dann die
 *       Daten in Apple HomeKit eingebunden und dort als Thermostat dargestellt.
 *
 * Autor: Matthiass Reinartz
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

// ---- WLAN ----
String wifiSsid;
String wifiPass;

// ---- MQTT ----
WiFiClient espClient;
PubSubClient mqttClient(espClient);
String mqttHost;
String mqttUser;
String mqttPass;

// ---- Messparameter ----
constexpr float DIST_LEER_MM = 1390.0;
constexpr float DIST_VOLL_MM = 240.0;
constexpr float ULTRASONIC_US_TO_MM = 0.16868; // Umrechnungsfaktor für Schallgeschwindigkeit bei 10°C Lufttemperatur

// ---- Flags -----
constexpr unsigned long MEASUREMENT_INTERVAL_MS = 60000;      // jede Minute prüfen
constexpr unsigned long SEND_INTERVAL_MS        = 3600000;    // spätestens alle 1h senden
constexpr unsigned long MIN_CHANGE_MM           = 5;          // mind. 5mm Unterschied senden

// ---- Trend-Erkennung ----
constexpr int HISTORY_SIZE = 60;  // letzte 60 Minuten speichern
float history[HISTORY_SIZE];
unsigned long historyTime[HISTORY_SIZE];  // Timestamps der Messungen (ms)
int historyIndex = 0;
int historyCount = 0;

// ---- OTA ----
WebServer server(80);

// ---- Ultraschall ----
#define TRIG_PIN 5
#define ECHO_PIN 18
NewPing sonar(TRIG_PIN, ECHO_PIN, 200);

// ---- Zeitserver ----
const char* NTP_SERVER = "pool.ntp.org";
const long GMT_OFFSET_SEC = 3600;
const int DAYLIGHT_OFFSET_SEC = 3600;

// ---- Variablen ----
unsigned long lastMeasurementTime = 0;
unsigned long lastSendTime        = 0;
float lastMeasuredMM              = 0;
String lastSentTrend              = "";
bool otaInProgress                = false;
bool rebootDoneToday              = false;
unsigned long lastMQTTAttempt     = 0;
Preferences prefs;

// ---------- Hilfsfunktionen ----------
String readSerialLine() {
  String input = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (input.length() > 0) break; // Eingabe abgeschlossen
      } else {
        input += c;
      }
    }
  }
  input.trim();
  return input;
}

void addToHistory(float value) {
  history[historyIndex] = value;
  historyTime[historyIndex] = millis();
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

float calcTrend() {
  // einfache lineare Regression: Steigung (mm/min)
  if (historyCount < 5) return 0;  // zu wenig Daten

  double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
  for (int i = 0; i < historyCount; i++) {
    double x = (historyTime[i] - historyTime[0]) / 60000.0;  // Minuten
    double y = history[i];
    sumX  += x;
    sumY  += y;
    sumXY += x * y;
    sumXX += x * x;
  }

  double denom = historyCount * sumXX - sumX * sumX;
  if (denom == 0) return 0;

  double slope = (historyCount * sumXY - sumX * sumY) / denom;
  return slope;  // mm pro Minute
}

String detectTrend() {
  float slope = calcTrend();

  if (slope < -0.1) return "einlaufend";   // Wasserstand steigt
  if (slope > 0.1)  return "abnehmend";    // Wasserstand sinkt
  return "stabil";                         // kaum Änderung
}

String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "1970-01-01T00:00:00";
  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  return String(buffer);
}

String stripTimestamp(const String &msg) {
  int pos = msg.indexOf(" -- ");
  if (pos >= 0) {
    return msg.substring(pos + 4); // alles nach " -- "
  }
  return msg;
}

void connectMQTT() {
  mqttClient.setServer(mqttHost.c_str(), 1883);

  while (!mqttClient.connected()) {
    Serial.print("Verbinde MQTT...");

    bool connected;

    if (mqttUser != "" && mqttPass != "") {
      connected = mqttClient.connect(
        "CisternSensor",                      // ClientID
        mqttUser.c_str(), mqttPass.c_str(),   // User/Pass
        "wasserstand/status",                 // LWT Topic
        0,                                    // QoS
        true,                                 // Retained
        "offline"                             // LWT Message
      );
    } else {
      connected = mqttClient.connect(
        "CisternSensor",
        "wasserstand/status",
        0,
        true,
        "offline"
      );
    }

    if (connected) {
      Serial.println("✅ verbunden");
      mqttClient.publish("wasserstand/status", "online", true); // retained = true
    } else {
      Serial.printf(" ❌ Fehler, rc=%d\n", mqttClient.state());
      delay(2000);
    }
  }
}

void ensureMQTT() {
   if (!mqttClient.connected()) {
    if (millis() - lastMQTTAttempt > 10000) {
      lastMQTTAttempt = millis();
      Serial.println("🔌 Versuche MQTT-Reconnect...");
      
      bool connected;
      if (mqttUser != "" && mqttPass != "") {
        connected = mqttClient.connect(
          "CisternSensor",
          mqttUser.c_str(), mqttPass.c_str(),
          "wasserstand/status", 0, true, "offline"
        );
      } else {
        connected = mqttClient.connect(
          "CisternSensor",
          "wasserstand/status", 0, true, "offline"
        );
      }

      if (connected) {
        Serial.println("✅ MQTT verbunden");
        mqttClient.publish("wasserstand/status", "online", true);
      } else {
        Serial.printf("❌ Fehler, rc=%d\n", mqttClient.state());
      }
    }
  }

  mqttClient.loop(); // trotzdem immer aufrufen!
}

void publishMQTT(const char* topic, const String& payload, bool retained = true) {
  if (mqttClient.connected()) {
    mqttClient.publish(topic, payload.c_str(), retained);
  }
}

void logError(const String &err) {
  String payload = getTimestamp() + " -- " + err;
  prefs.begin("errorlog", false);
  prefs.putString("lastError", payload);
  prefs.end();
  publishMQTT("wasserstand/lastError", payload);
}

String readLastError() {
  prefs.begin("errorlog", true);
  String e = prefs.getString("lastError", "Kein Fehler gespeichert");
  prefs.end();
  return e;
}

void ensureWifi() {
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(wifiSsid, wifiPass);
    while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.printf("\nWLAN verbunden: %s\n", WiFi.localIP().toString().c_str());
  }
}

bool shouldSendMeasurement(float currentMM, const String& trend) {
  bool timeExceeded = millis() - lastSendTime >= SEND_INTERVAL_MS;
  bool changeExceeded = fabs(currentMM - lastMeasuredMM) >= MIN_CHANGE_MM;
  bool trendChanged   = (trend != lastSentTrend);
  return timeExceeded || changeExceeded || trendChanged;
}

void performMeasurement(bool forceSend = false) {
  // Messung durchführen und Entfernung berechnen.
  // Es werden 5 Messungen durchgeführt, die beiden extreme verworfen und aus den drei verbleibenden der Durchschnitt geliefert.
  float duration = sonar.ping_median(5);
  float dist_mm  = duration * ULTRASONIC_US_TO_MM;

  // Plausibilitätsprüfung
  if (dist_mm <= 200 || dist_mm > 2000) {
    logError("Sensorfehler: " + String(dist_mm) + " mm");
    return;
  }

  float prozent = (DIST_LEER_MM - dist_mm) / (DIST_LEER_MM - DIST_VOLL_MM) * 100.0;
  prozent = constrain(prozent, 0, 100);

  addToHistory(dist_mm); // History für Trenderkennung befüllen

  String trend = detectTrend();

  if (forceSend || shouldSendMeasurement(dist_mm, trend)) {
    lastMeasuredMM = dist_mm;
    lastSendTime   = millis();
    lastSentTrend  = trend; 

    publishMQTT("wasserstand/prozent", String((int)prozent));
    publishMQTT("wasserstand/absolut", String((int)round(dist_mm)));

    String json = "{\"timestamp\":\"" + getTimestamp() +
                  "\",\"prozent\":" + String(prozent, 1) +
                  ",\"absolut\":" + String((int)round(dist_mm)) + 
                  ",\"trend\":\"" + trend + "\"}";

    String state = "OFF"; // Default Off
    if (trend == "einlaufend") {
      state = "HEAT"; // Heat
      publishMQTT("wasserstand/getTargetTemperature", "100");
    } else if (trend == "abnehmend") {
      state = "COOL"; // Cool
      publishMQTT("wasserstand/getTargetTemperature", "0");
    } else if (trend == "stabil") {
      state = "OFF"; // Off
    }
    publishMQTT("wasserstand/log", json);
    publishMQTT("wasserstand/fuellt", (trend == "einlaufend") ? "true" : "false"); // wenn trend einlaufend, dann true, für Batterie -ladend
    publishMQTT("wasserstand/matchCoolingState", state);
  }
}

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

// ---------- SETUP ----------
void configSetup() {
  prefs.begin("config", false);

  Serial.println("⚙️  Neue Konfiguration eingeben:");

  Serial.print("WLAN SSID: ");
  String ssid = readSerialLine();
  prefs.putString("wifi_ssid", ssid);

  Serial.print("WLAN Passwort: ");
  String pass = readSerialLine();
  prefs.putString("wifi_pass", pass);

  Serial.print("MQTT Host/IP: ");
  String mqttHost = readSerialLine();
  prefs.putString("mqtt_host", mqttHost);

  Serial.print("MQTT Benutzer (leer lassen wenn keiner): ");
  String mqttUser = readSerialLine();
  prefs.putString("mqtt_user", mqttUser);

  Serial.print("MQTT Passwort (leer lassen wenn keiner): ");
  String mqttPass = readSerialLine();
  prefs.putString("mqtt_pass", mqttPass);

  Serial.println("✅ Konfiguration gespeichert! Bitte Reset durchführen.");
  while (true) { delay(1000); } // stoppt, bis Reset gedrückt wird

  prefs.end();
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Zugangsdaten WLAN und MQTT aus dem NVS abfragen
  prefs.begin("config", false);

  wifiSsid = prefs.getString("wifi_ssid", "");
  wifiPass = prefs.getString("wifi_pass", "");
  mqttHost = prefs.getString("mqtt_host", "");
  mqttUser = prefs.getString("mqtt_user", "");
  mqttPass = prefs.getString("mqtt_pass", "");

  if (wifiSsid == "" || wifiPass == "" || mqttHost == "") {
    Serial.println("⚠️  Keine gültige Konfiguration gefunden.");
    configSetup();
  } else {
    Serial.println("✅ Konfiguration geladen.");
    Serial.printf("SSID: %s, MQTT Host: %s\n", wifiSsid.c_str(), mqttHost.c_str());
    Serial.println("👉 Tippe 'resetconfig' oder 'setconfig' um Einstellungen zu ändern.");
  }
  prefs.end();


  // WLAN verbinden (synchron)
  ensureWifi();

  // MQTT verbinden (synchron)
  connectMQTT();

  // OTA & Server
  ElegantOTA.onStart([]() {
    otaInProgress = true;
    logError("OTA Update gestartet – stoppe Messungen.");
  });
  ElegantOTA.onEnd([](bool success) {
    otaInProgress = false;
    String msg = success ? "OTA erfolgreich – Neustart" : "OTA fehlgeschlagen";
    logError(msg);
    if (success) { delay(1000); ESP.restart(); }
  });
  ElegantOTA.begin(&server);
  server.on("/", []() { server.sendHeader("Location", "/update"); server.send(302, ""); });
  server.begin();

  // NTP
  configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

  // ---- Letzten gespeicherten Fehler nur senden, wenn er NICHT gleich dem neuen Startgrund ist ----
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

  prefs.begin("errorlog", false);
  prefs.remove("lastError");
  prefs.end();

  // Jetzt neuen Reset-Grund loggen
  logError(resetMsg);

  // Initiale Messung
  performMeasurement(true);
  lastMeasurementTime = millis();
}

// ---------- LOOP ----------
void loop() {
  server.handleClient();

  if (!otaInProgress) {
    ensureWifi();
    ensureMQTT();
    mqttClient.loop();

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
