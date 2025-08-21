# CisternSensor – Wasserstandsmessung mit ESP32 und Ultraschallsensor

Misst den Wasserstand einer Zisterne per Ultraschallsensor und veröffentlicht die Daten über **MQTT**.  
Zusätzlich läuft ein Webserver für **OTA-Updates** und Debugging.

---

## ✨ Features
- WLAN-Anbindung (SSID & Passwort seriell konfigurierbar, im Flash gespeichert)
- MQTT-Client mit optionalem Benutzername/Passwort  
  → inkl. **Last Will & Testament** (`wasserstand/status = offline`)
- JSON-Log mit Zeitstempel, Prozent- und Absolutwert
- OTA Update über Webinterface
- Täglicher Reboot als Failsafe
- Error-Logging auf separatem Topic

---

## 📡 MQTT Topics
| Topic                  | Inhalt / Beispiel                             |
|-------------------------|-----------------------------------------------|
| `wasserstand/prozent`   | `74.7`                                        |
| `wasserstand/log`       | `{"timestamp":"2025-08-14T22:58:28","prozent":74.7,"absolut":531}` |
| `wasserstand/status`    | `online` / `offline` (Last Will)              |
| `wasserstand/lastError` | `2025-08-14T22:54:53 -- Reset-Grund: Software Reset` |

---

## 🛠️ Hardware
- ESP32 NodeMCU Development Board
- JSN-SR04T-2.0 Wasserfester Ultraschallsensor

---

## 🚀 Setup

1. **Firmware flashen**  
   - Sketch kompilieren & auf den ESP laden.

2. **WLAN & MQTT konfigurieren (über Serial)**  
   - Nach dem ersten Start fragt der ESP nach Zugangsdaten:  
     ```
     Enter WiFi SSID:
     Enter WiFi Password:
     Enter MQTT Host:
     (optional) MQTT Username:
     (optional) MQTT Password:
     ```
   - Daten werden im Flash gespeichert und beim Neustart automatisch verwendet.

3. **OTA Update**  
   - ESP im WLAN aufrufen: `http://<esp-ip>/update`  
   - Neue Firmware hochladen.

---

## ⚙️ Developer Notes
- **WiFi & MQTT Reconnect** sind non-blocking → OTA/Webserver bleiben auch bei Verbindungsproblemen erreichbar.
- Messungen werden **nur durchgeführt, wenn MQTT verbunden ist**.  
  Andernfalls wird ein Fehler ins Topic `wasserstand/lastError` geschrieben.
- Alle Log-Einträge enthalten einen **ISO8601-Zeitstempel**.

---

## 📖 License
MIT – feel free to use & adapt.  

---

## 👤 Autor
- Matthias Reinartz
- [GitLab Repo](https://gitlab.com/DEIN_REPO)
