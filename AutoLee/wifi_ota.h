// ============================================================================
//  AutoLee – wifi_ota.h
//  WiFi connection management, supervision, captive portal, ArduinoOTA
// ============================================================================
#pragma once

#include <ArduinoOTA.h>
// All globals and forward declarations are provided by AutoLee.ino
// (single translation unit — Arduino IDE model)

// ==========================================================================
//  WiFi CREDENTIALS
//  NVS access is slow; these are only ever called from loop() context
//  (the web handlers defer via webWifiSaveRequested / webWifiClearRequested).
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
    ssid.replace("&", "&amp;");  // must be escaped first
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
  WiFi.setAutoReconnect(true);
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
//  SERVICES (web server + OTA listener)
//  Registration happens once in setup(); only the listening sockets are
//  started and stopped here, so handlers are never registered twice.
// ==========================================================================
void wifiStartServices() {
  if (wifiServicesRunning) return;
  webServer.begin();
  ArduinoOTA.begin();
  wifiServicesRunning = true;
  Serial.println("Web server + OTA started on port 80");
}

void wifiStopServices() {
  if (!wifiServicesRunning) return;
  events.close();       // drop SSE clients before the socket goes away
  webServer.end();
  ArduinoOTA.end();
  wifiServicesRunning = false;
  Serial.println("Web server + OTA stopped");
}

static void wifiPowerOff() {
  if (captivePortalRunning) {
    dnsServer.stop();
    captivePortalRunning = false;
  }
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, false);   // drop the connection, keep credentials
  WiFi.mode(WIFI_OFF);
  wifiConnected = false;
  wifiAPMode    = false;
}

// Master WiFi switch. Always called from loop() context (the touch button and
// the web endpoint both defer via wifiEnableRequested).
//
// Enabling reboots on purpose. Bringing the radio and the listening sockets
// back up in place does NOT work: ESPAsyncWebServer's listener does not
// reliably re-bind once its socket has been closed and the interface taken
// down — WiFi reconnects and reports the correct IP, but nothing answers on
// port 80 until the next power cycle. A reboot takes ~2 s and is always
// clean. Disabling needs no reboot and stays immediate.
void setWifiEnabled(bool en) {
  if (en == wifiEnabled) return;
  wifiEnabled = en;

  if (en) {
    Serial.println("WiFi: enabling — rebooting to bring services up cleanly");
    if (lbl_wifi_status) {
      lv_label_set_text(lbl_wifi_status, "Enabling WiFi...\nrebooting");
      lv_refr_now(NULL);
    }
    saveSettings();            // persist the switch before we restart
    rebootRequestMs = millis();   // before the flag: loop() reads both
    rebootRequested = true;
    return;
  }

  markSettingsDirty();
  webLog("WiFi disabled");
  wifiStopServices();
  wifiPowerOff();
  ui_update_wifi_label();
}

// ==========================================================================
//  WiFi SUPERVISION — called from loop()
//  v1.10 latched wifiConnected at boot, so the screen and /api/state kept
//  advertising a stale SSID/IP forever after the AP dropped.
// ==========================================================================
static uint32_t wifiCheckMs = 0;
static uint32_t wifiRetryMs = 0;

void handleWiFi() {
  if (!wifiEnabled) return;           // radio is off
  if (wifiAPMode) return;             // captive portal: nothing to supervise
  if (wifiSSID.length() == 0) return; // never configured

  uint32_t now = millis();
  if ((now - wifiCheckMs) < WIFI_CHECK_MS) return;
  wifiCheckMs = now;

  bool up = (WiFi.status() == WL_CONNECTED);
  if (up != wifiConnected) {
    wifiConnected = up;
    if (up) {
      webLog("WiFi: reconnected, IP=%s", WiFi.localIP().toString().c_str());
      wifiRetryMs = now;
    } else {
      webLog("WiFi: connection lost");
    }
    ui_update_wifi_label();
  }

  // Nudge the supplicant if auto-reconnect hasn't recovered on its own.
  if (!up && (now - wifiRetryMs) > WIFI_RETRY_MS) {
    wifiRetryMs = now;
    webLog("WiFi: retrying '%s'", wifiSSID.c_str());
    WiFi.reconnect();
  }
}

// ==========================================================================
//  ArduinoOTA (for PlatformIO/Arduino IDE OTA)
// ==========================================================================
void setupArduinoOTA() {
  ArduinoOTA.setHostname("autolee");
  ArduinoOTA.setPassword("autolee");
  ArduinoOTA.onStart([]() {
    Serial.println("OTA: start");
    // onStart runs in loop() context, so we can actually wait for the ram to
    // park before the flash is erased instead of rebooting mid-stroke.
    batchActive = false;
    if (runState == RUNNING) requestGracefulStop();
    uint32_t t0 = millis();
    while (stepper && stepper->isRunning() && (millis() - t0) < 4000) {
      handleMotion();
      delay(1);
    }
    if (stepper && stepper->isRunning()) stepper->forceStop();
    runState = IDLE;
    if (settingsDirty) saveSettings();
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA: done"); });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    if (t > 0) Serial.printf("OTA: %u%%", (unsigned)((uint64_t)p * 100 / t));
  });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA err: %u", e); });
  // ArduinoOTA.begin() is deliberately NOT called here — the listener is
  // started and stopped by wifiStartServices()/wifiStopServices().
}
