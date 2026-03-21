// ============================================================================
//  AutoLee – wifi_ota.h
//  WiFi connection management, captive portal, ArduinoOTA
// ============================================================================
#pragma once

#include <ArduinoOTA.h>
// All globals and forward declarations are provided by AutoLee.ino
// (single translation unit — Arduino IDE model)

// ==========================================================================
//  WiFi CREDENTIALS
// ==========================================================================
void loadWiFiCredentials() {
  prefs.begin("autolee", true);
  wifiSSID = prefs.getString("ssid", "");
  wifiPass = prefs.getString("pass", "");
  prefs.end();
}

void saveWiFiCredentials(const String &ssid, const String &pass) {
  prefs.begin("autolee", false);
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.end();
  wifiSSID = ssid; wifiPass = pass;
}

void clearWiFiCredentials() {
  prefs.begin("autolee", false);
  prefs.remove("ssid"); prefs.remove("pass");
  prefs.end();
  wifiSSID = ""; wifiPass = "";
}

// ==========================================================================
//  NETWORK SCANNING
// ==========================================================================
static void scanNetworks() {
  scannedOptionsHTML = "<option value=''>-- Select WiFi --</option>";
  int n = WiFi.scanNetworks(false, true);
  if (n <= 0) {
    scannedOptionsHTML += "<option value=''>No networks found</option>";
    return;
  }
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    int rssi = WiFi.RSSI(i);
    String sec = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "OPEN" : "SEC";
    ssid.replace("\"", "&quot;"); ssid.replace("'", "&#39;");
    ssid.replace("<", "&lt;");    ssid.replace(">", "&gt;");
    scannedOptionsHTML += "<option value=\"" + ssid + "\">" + ssid +
      " (" + rssi + " dBm " + sec + ")</option>";
  }
  WiFi.scanDelete();
}

// ==========================================================================
//  WiFi CONNECTION
// ==========================================================================
static bool connectToWiFi(const char *ssid, const char *pass, uint32_t timeoutMs) {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(200);
  WiFi.begin(ssid, pass);
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    lv_timer_handler(); delay(10);
  }
  return (WiFi.status() == WL_CONNECTED);
}

void startWiFi() {
  loadWiFiCredentials();
  if (wifiSSID.length() > 0) {
    Serial.printf("WiFi: connecting to '%s'...\n", wifiSSID.c_str());
    if (connectToWiFi(wifiSSID.c_str(), wifiPass.c_str(), 10000)) {
      wifiConnected = true; wifiAPMode = false;
      Serial.printf("WiFi: connected! IP=%s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("WiFi: STA failed, starting captive portal");
    }
  }
  if (!wifiConnected) {
    // Start OPEN captive portal AP (no password — easier to connect)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEFAULT_AP_SSID);  // open AP, no password
    delay(300);
    wifiAPMode = true;
    captivePortalRunning = true;
    scanNetworks();

    // Start DNS server for captive portal redirect
    dnsServer.start(53, "*", WiFi.softAPIP());

    Serial.printf("WiFi AP: %s @ %s (captive portal)\n", DEFAULT_AP_SSID, WiFi.softAPIP().toString().c_str());
  }
  ui_update_wifi_label();
}

// ==========================================================================
//  ArduinoOTA (for PlatformIO/Arduino IDE OTA)
// ==========================================================================
void setupArduinoOTA() {
  ArduinoOTA.setHostname("autolee");
  ArduinoOTA.setPassword("autolee");
  ArduinoOTA.onStart([]() {
    if (runState == RUNNING) requestGracefulStop();
    Serial.println("OTA: start");
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    Serial.printf("OTA: %u%%", p / (t / 100));
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA err: %u", e); });
  ArduinoOTA.begin();
}
