// =============================================================================
// AutoFOV_V11_patched3_wifi.ino  —  WiFi / WebSocket server tab
//
// Arduino multi-file tab: compiled together with AutoFOV_V11_patched3.ino.
// All variables from the main file are accessible directly (same compilation
// unit — Arduino concatenates all .ino files before compiling).
//
// ── INTEGRATION ───────────────────────────────────────────────────────────────
//   Already wired up in patched3.ino:
//     setup() end   → wifiSetup();
//     loop() top    → wifiLoop();
//
// ── REQUIRED LIBRARIES (install via Arduino Library Manager) ─────────────────
//   • ESPAsyncWebServer  (by me-no-dev)
//   • AsyncTCP           (by me-no-dev)
//   • ArduinoJson        (by Benoit Blanchon, v6.x)
//   DNSServer, LittleFS, and WiFi are part of the ESP32 Arduino core.
//
// ── HTML FILE ────────────────────────────────────────────────────────────────
//   Create a  data/  folder alongside this .ino and place index.html in it.
//   Upload via the Arduino LittleFS filesystem uploader:
//   https://github.com/lorol/arduino-esp32fs-plugin
//
// ── CAPTIVE PORTAL ───────────────────────────────────────────────────────────
//   First boot (no saved credentials) → AP mode "AutoFOV-Setup".
//   Connect any device to that SSID, browser opens setup form automatically.
//   Enter SSID + password; optionally set a static IP (leave blank for DHCP).
//   After Save the ESP32 restarts, connects to your network, and shows its IP
//   on the Serial console + TFT WiFi indicator.
//   Hold BOOT (GPIO0) during power-on to force the portal at any time.
//   Tap the WiFi zone in the TFT header (x=93..151, y=0..42) to open the
//   WIFI_INFO screen, which includes a FORGET WiFi button for manual reset.
// =============================================================================

// patched3: these headers are also included in patched3.ino — kept here for
// readability and standalone-tab usability.  Header guards make the duplicate
// #include harmless.  Do NOT move the *primary* copy back to this tab: the
// Arduino preprocessor injects forward declarations for all .ino functions
// before the wifi tab's #includes are processed, which would break compilation
// of the AsyncWebSocket* / AsyncWebSocketClient* signatures below.
#include <WiFi.h>
#include <DNSServer.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// ─────────────────────────────────────────────────────────────────────────────
// CONFIG
// ─────────────────────────────────────────────────────────────────────────────
static const char*     AP_SSID             = "AutoFOV-Setup";
static const char*     AP_PASS             = "";          // open — no WPA2 for easy setup
static const IPAddress AP_IP               (192, 168, 4, 1);
static const IPAddress AP_GW               (192, 168, 4, 1);
static const IPAddress AP_MASK             (255, 255, 255, 0);
static const uint8_t   DNS_PORT            = 53;
static const uint32_t  CONNECT_TIMEOUT_MS  = 15000UL;    // 15 s to acquire IP
static const uint32_t  RECONNECT_INTERVAL_MS = 30000UL;  // retry every 30 s
static const uint32_t  FAST_TELEM_MS       = 33UL;       // ~30 Hz live sensor push — matches VL53L4CX timing budget 1:1
static const uint32_t  SLOW_TELEM_MS       = 5000UL;     // 5 s memory + BT push
static const int       CMD_QUEUE_DEPTH     = 16;

// ─────────────────────────────────────────────────────────────────────────────
// WIFI STATE
// ─────────────────────────────────────────────────────────────────────────────
enum WifiServerMode { WMODE_NONE, WMODE_PORTAL, WMODE_STA };
static WifiServerMode  wifiServerMode      = WMODE_NONE;
static bool            wifiConnected       = false;
static uint32_t        lastReconnectMs     = 0;
static uint32_t        lastFastTelemMs     = 0;
static uint32_t        lastSlowTelemMs     = 0;

// ─────────────────────────────────────────────────────────────────────────────
// SERVER OBJECTS
// ─────────────────────────────────────────────────────────────────────────────
static AsyncWebServer  httpServer(80);
static AsyncWebSocket  wsServer("/ws");
static DNSServer       dnsServer;

// Separate Preferences object for WiFi credentials — namespace "wifi".
// This is distinct from the main sketch's "calib" and "display" namespaces.
static Preferences     wifiPrefs;

// ─────────────────────────────────────────────────────────────────────────────
// COMMAND QUEUE
// WS onMessage callback fires on the lwIP/async task (NOT Core 1).
// We must not touch I²C, TFT, or FreeRTOS semaphores from there.
// Commands are pushed into this queue and drained in wifiLoop() on Core 1.
// ─────────────────────────────────────────────────────────────────────────────
struct WifiCmd {
    char key[32];
    char val[64];
};
static QueueHandle_t wifiCmdQueue = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// CAPTIVE PORTAL HTML  (served in AP mode, embedded in flash via PROGMEM)
// ─────────────────────────────────────────────────────────────────────────────
static const char PORTAL_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html><head><meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>AutoFOV WiFi Setup</title>
<style>
  *{box-sizing:border-box}
  body{background:#111;color:#eee;font-family:Arial,sans-serif;
       display:flex;flex-direction:column;align-items:center;padding:30px 20px;margin:0}
  h2{color:#ADFF2F;margin-bottom:6px;font-size:22px}
  p{color:#666;font-size:12px;margin:0 0 24px}
  label{display:block;margin-bottom:4px;font-size:13px;color:#888}
  input{display:block;width:100%;max-width:320px;padding:10px;margin-bottom:16px;
        background:#222;border:1px solid #444;color:#fff;border-radius:4px;font-size:15px}
  input:focus{outline:none;border-color:#008894}
  .sep{width:100%;max-width:320px;border:none;border-top:1px solid #2a2a2a;margin:8px 0 20px}
  .hint{font-size:11px;color:#555;margin:-12px 0 14px}
  button{background:#008894;color:#fff;border:none;padding:13px;font-size:15px;
         font-weight:bold;border-radius:4px;cursor:pointer;width:100%;max-width:320px}
  button:active{filter:brightness(1.3)}
</style></head>
<body>
<h2>AutoFOV WiFi Setup</h2>
<p>Connect to your home network</p>
<form method=POST action=/save>
  <label>Network SSID</label>
  <input type=text name=ssid placeholder="Your WiFi name" required autocomplete=off spellcheck=false>
  <label>Password</label>
  <input type=password name=pass placeholder="WiFi password" autocomplete=off>
  <hr class=sep>
  <label>Static IP <small style=color:#555>(optional — blank = DHCP)</small></label>
  <input type=text name=ip placeholder="192.168.1.200"
         pattern="^$|^(\d{1,3}\.){3}\d{1,3}$">
  <p class=hint>Leave blank to let your router assign an IP automatically.</p>
  <label>Gateway <small style=color:#555>(optional — auto-derived if blank)</small></label>
  <input type=text name=gw placeholder="192.168.1.1"
         pattern="^$|^(\d{1,3}\.){3}\d{1,3}$">
  <button type=submit>SAVE &amp; CONNECT</button>
</form>
</body></html>)html";

static const char PORTAL_SAVED_HTML[] PROGMEM = R"html(<!DOCTYPE html>
<html><head><meta charset=UTF-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>AutoFOV</title>
<style>body{background:#111;color:#eee;font-family:Arial,sans-serif;
            text-align:center;padding:40px 20px;margin:0}
h2{color:#ADFF2F}p{color:#888;font-size:13px;line-height:1.7}</style>
</head><body>
<h2>&#10003; Credentials saved!</h2>
<p>AutoFOV is restarting and connecting to your network.<br>
The assigned IP address will appear on the device screen and Serial console.<br>
<br>
Point your browser to that address to open the control panel.</p>
</body></html>)html";

// ─────────────────────────────────────────────────────────────────────────────
// FORWARD DECLARATIONS
// ─────────────────────────────────────────────────────────────────────────────
static void startPortalMode();
static void startStaMode(const String& ssid, const String& pass,
                          const String& staticIP, const String& gateway);
static void startFullServer();
static void handleWifiCommand(const char* key, const char* val);
static void buildFullStateJson(String& out, bool includeCalGraph = true);
static void buildFastTelemJson(String& out);
static void buildSlowTelemJson(String& out);
static void buildSettingsJson(String& out);

// ─────────────────────────────────────────────────────────────────────────────
// WEBSOCKET EVENT HANDLER  (runs on async task — enqueue only, no I²C/TFT)
// ─────────────────────────────────────────────────────────────────────────────
static void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {

    if (type == WS_EVT_CONNECT) {
        Serial.printf("[WS] client #%u connected from %s — sending full state\n",
                      client->id(), client->remoteIP().toString().c_str());
        Serial.flush();
        // Push the full device state to the newly connected client.
        // buildFullStateJson touches only atomics and const globals — safe here.
        String out;
        buildFullStateJson(out);
        client->text(out);
        Serial.printf("[WS] full-state push: %u bytes\n", out.length());
        Serial.flush();

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        // Accept only single-frame, complete text messages
        if (!info->final || info->index != 0 || info->len != len
            || info->opcode != WS_TEXT) {
            Serial.printf("[WS] data dropped (final=%d index=%u len=%u opcode=%d)\n",
                          info->final, (unsigned)info->index, (unsigned)info->len, info->opcode);
            Serial.flush();
            return;
        }

        char buf[len + 1];
        memcpy(buf, data, len);
        buf[len] = '\0';
        Serial.printf("[WS] data rx: %s\n", buf);
        Serial.flush();

        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, buf) != DeserializationError::Ok) return;

        for (JsonPair kv : doc.as<JsonObject>()) {
            WifiCmd cmd;
            strncpy(cmd.key, kv.key().c_str(), sizeof(cmd.key) - 1);
            cmd.key[sizeof(cmd.key) - 1] = '\0';

            // Serialise value to string for uniform dispatch
            String vStr;
            if      (kv.value().is<const char*>()) vStr = kv.value().as<const char*>();
            else if (kv.value().is<double>())       vStr = String(kv.value().as<double>(), 6);
            else if (kv.value().is<long>())         vStr = String(kv.value().as<long>());
            else if (kv.value().is<bool>())         vStr = kv.value().as<bool>() ? "1" : "0";
            else serializeJson(kv.value(), vStr);

            strncpy(cmd.val, vStr.c_str(), sizeof(cmd.val) - 1);
            cmd.val[sizeof(cmd.val) - 1] = '\0';
            xQueueSend(wifiCmdQueue, &cmd, (TickType_t)0);
        }

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] client #%u disconnected\n", client->id());
        Serial.flush();
        wsServer.cleanupClients();
    } else if (type == WS_EVT_ERROR) {
        Serial.printf("[WS] error on client #%u\n", client->id());
        Serial.flush();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiSetup()  —  call at the END of setup() in the main .ino
// ─────────────────────────────────────────────────────────────────────────────
void wifiSetup() {
    wifiCmdQueue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(WifiCmd));

    // Mount LittleFS — HTML served from /index.html
    if (!LittleFS.begin(true)) {
        Serial.println("[WiFi] LittleFS mount failed — /index.html will not be served");
    }

    // GPIO0 (BOOT button) held LOW at power-on → force captive portal
    pinMode(0, INPUT_PULLUP);
    delay(50);
    bool forcePortal = (digitalRead(0) == LOW);

    // Load saved credentials
    wifiPrefs.begin("wifi", true);   // read-only
    String ssid     = wifiPrefs.getString("ssid", "");
    String pass     = wifiPrefs.getString("pass", "");
    String staticIP = wifiPrefs.getString("ip",   "");
    String gateway  = wifiPrefs.getString("gw",   "");
    wifiPrefs.end();

    if (forcePortal || ssid.isEmpty()) {
        if (forcePortal) Serial.println("[WiFi] BOOT held — forcing captive portal");
        startPortalMode();
    } else {
        Serial.printf("[WiFi] Connecting to \"%s\"…\n", ssid.c_str());
        startStaMode(ssid, pass, staticIP, gateway);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiLoop()  —  call in loop() after the atomic-read block in the main .ino
// ─────────────────────────────────────────────────────────────────────────────
void wifiLoop() {

    // Captive portal: keep the DNS server ticking
    if (wifiServerMode == WMODE_PORTAL) {
        dnsServer.processNextRequest();

        // patched3 DEBUG: heartbeat every 5 s so we can tell whether the AP
        // is staying up + see when a client actually associates with us.
        // Delete once the AP-broadcast issue is resolved.
        static uint32_t lastApHb = 0;
        if (millis() - lastApHb > 5000) {
            lastApHb = millis();
            Serial.printf("[WiFi] AP heartbeat — clients=%d  IP=%s  ch=%d  RSSI?=N/A\n",
                          WiFi.softAPgetStationNum(),
                          WiFi.softAPIP().toString().c_str(),
                          WiFi.channel());
            Serial.flush();
        }
        return;   // no telemetry in portal mode
    }

    if (wifiServerMode != WMODE_STA) return;

    // ── Reconnect logic ──────────────────────────────────────────────────────
    if (!wifiConnected) {
        if (millis() - lastReconnectMs >= RECONNECT_INTERVAL_MS) {
            lastReconnectMs = millis();
            if (WiFi.status() == WL_CONNECTED) {
                wifiConnected = true;
                Serial.printf("[WiFi] Reconnected: %s\n",
                              WiFi.localIP().toString().c_str());
                if (currentMode == MAIN) drawWifiIndicator();   // patched3: update header
                String out; buildFullStateJson(out, false);    // reconnect: skip calGraphPoints
                wsServer.textAll(out);
            } else {
                Serial.println("[WiFi] Attempting reconnect…");
                WiFi.reconnect();
            }
        }
        return;
    }

    // Check for drop since last loop
    if (WiFi.status() != WL_CONNECTED) {
        wifiConnected = false;
        lastReconnectMs = millis();
        Serial.println("[WiFi] Connection lost — will retry");
        if (currentMode == MAIN) drawWifiIndicator();   // patched3: show disconnected bars
        return;
    }

    // ── Drain command queue ──────────────────────────────────────────────────
    WifiCmd cmd;
    while (xQueueReceive(wifiCmdQueue, &cmd, 0) == pdTRUE) {
        Serial.printf("[CMD] dispatch %s = %s\n", cmd.key, cmd.val);
        Serial.flush();
        handleWifiCommand(cmd.key, cmd.val);
    }

    // ── Telemetry push ───────────────────────────────────────────────────────
    if (wsServer.count() == 0) return;   // no connected clients — skip serialisation

    uint32_t now = millis();
    if (now - lastFastTelemMs >= FAST_TELEM_MS) {
        lastFastTelemMs = now;
        String out; buildFastTelemJson(out);
        wsServer.textAll(out);
    }
    if (now - lastSlowTelemMs >= SLOW_TELEM_MS) {
        lastSlowTelemMs = now;
        String out; buildSlowTelemJson(out);
        wsServer.textAll(out);
    }

    // ── Device-side state-change detection ──────────────────────────────────
    // Touch handlers on the device modify settings directly without notifying
    // wifi.  Snapshot the values that can change on-device; when any differs
    // from the last push, send a fresh full-state frame so the web UI tracks.
    static int      lastObj          = -1;
    static int      lastBrightness   = -1;
    static bool     lastLedEnabled   = false;
    static int      lastLedDuty      = -1;
    static bool     lastSensorSleep  = false;
    static bool     lastHighRefl     = false;
    static int      lastStepIndex    = -1;
    static int      lastImgs         = -1;
    static float    lastSecStep      = -1.0f;
    static int      lastTheme        = -1;
    static int      lastTint         = -1;
    static uint32_t lastDimMs        = 0xFFFFFFFFUL;
    static uint32_t lastSleepMs      = 0xFFFFFFFFUL;
    static bool     stateSnapInit    = false;

    if (currentobj      != lastObj          ||
        currentBrightness != lastBrightness ||
        ledEnabled      != lastLedEnabled   ||
        currentLedDuty  != lastLedDuty      ||
        sensorSleeping  != lastSensorSleep  ||
        highReflMode    != lastHighRefl     ||
        stackStepIndex  != lastStepIndex    ||
        stackTotalImgs  != lastImgs         ||
        fabsf(stackTimePerStep - lastSecStep) > 0.01f ||
        currentThemeIndex != lastTheme      ||
        themeIntensity  != lastTint         ||
        (uint32_t)dimTimeoutMs   != lastDimMs   ||
        (uint32_t)sleepTimeoutMs != lastSleepMs) {

        lastObj         = currentobj;
        lastBrightness  = currentBrightness;
        lastLedEnabled  = ledEnabled;
        lastLedDuty     = currentLedDuty;
        lastSensorSleep = sensorSleeping;
        lastHighRefl    = highReflMode;
        lastStepIndex   = stackStepIndex;
        lastImgs        = stackTotalImgs;
        lastSecStep     = stackTimePerStep;
        lastTheme       = currentThemeIndex;
        lastTint        = themeIntensity;
        lastDimMs       = (uint32_t)dimTimeoutMs;
        lastSleepMs     = (uint32_t)sleepTimeoutMs;

        // Skip the very first iteration — we'd be reacting to the snapshot
        // initialization itself, not a real user change.
        if (stateSnapInit) {
            String out; buildSettingsJson(out);
            wsServer.textAll(out);
        }
        stateSnapInit = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AP / CAPTIVE PORTAL MODE
// ─────────────────────────────────────────────────────────────────────────────
static void startPortalMode() {
    wifiServerMode = WMODE_PORTAL;

    // patched3 DEBUG: instrumented AP bring-up.  After the BLE-init-order fix
    // (wifiSetup now runs BEFORE NimBLEDevice::init), pure WIFI_AP should
    // succeed — WiFi grabs the controller first and registers with coex.

    Serial.printf("[WiFi] === startPortalMode entry  heap=%u ===\n", ESP.getFreeHeap());
    Serial.flush();

    // Reset any stale WiFi driver state from a previous boot/init attempt.
    WiFi.persistent(false);   // don't write to NVS — we manage our own creds
    WiFi.disconnect(true, true);
    delay(100);

    bool mOk = WiFi.mode(WIFI_AP);
    Serial.printf("[WiFi] mode(AP) = %d  heap=%u\n", mOk, ESP.getFreeHeap());
    Serial.flush();
    delay(100);

    bool cOk = WiFi.softAPConfig(AP_IP, AP_GW, AP_MASK);
    Serial.printf("[WiFi] softAPConfig = %d\n", cOk); Serial.flush();

    // softAP(ssid, pass, channel, hidden, max_connection)
    //   channel 1  — least-used in most environments
    //   hidden 0   — broadcast SSID
    //   max_conn 4 — default
    bool aOk = WiFi.softAP(AP_SSID, AP_PASS, /*channel*/ 1, /*hidden*/ 0, /*max_conn*/ 4);
    Serial.printf("[WiFi] softAP = %d\n", aOk); Serial.flush();

    // Bump TX power to maximum allowed (19.5 dBm).
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    delay(300);  // let the AP fully start before reporting

    Serial.printf("[WiFi] AP IP      = %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("[WiFi] AP MAC     = %s\n", WiFi.softAPmacAddress().c_str());
    Serial.printf("[WiFi] AP hostname= %s\n", WiFi.softAPgetHostname());
    Serial.printf("[WiFi] AP channel = %d\n", WiFi.channel());
    Serial.printf("[WiFi] AP clients = %d\n", WiFi.softAPgetStationNum());
    Serial.println("[WiFi] === startPortalMode AP up — should be visible NOW ===");
    Serial.flush();

    // Redirect ALL DNS queries to our IP so the OS captive-portal detector fires
    dnsServer.start(DNS_PORT, "*", AP_IP);

    // 302-redirect every unknown URL to http://192.168.4.1/ so the OS
    // captive-portal detector trips reliably:
    //   • iOS  hits /hotspot-detect.html, sees a non-"Success" 302 → portal pops
    //   • Android hits /generate_204, gets a 302 (not 204) → portal pops
    //   • Windows /connecttest.txt, same trip
    // Returning portal HTML directly on the probe URL was leaving some clients
    // in "captive portal detected — open browser manually" limbo.
    httpServer.onNotFound([](AsyncWebServerRequest* req) {
        AsyncWebServerResponse* resp = req->beginResponse(302, "text/plain", "");
        resp->addHeader("Location", "http://192.168.4.1/");
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", PORTAL_HTML);
    });

    // POST /save — persist credentials and restart
    httpServer.on("/save", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            String ssid = req->hasParam("ssid", true)
                          ? req->getParam("ssid", true)->value() : "";
            String pass = req->hasParam("pass", true)
                          ? req->getParam("pass", true)->value() : "";
            String ip   = req->hasParam("ip",   true)
                          ? req->getParam("ip",   true)->value() : "";
            String gw   = req->hasParam("gw",   true)
                          ? req->getParam("gw",   true)->value() : "";

            ssid.trim(); pass.trim(); ip.trim(); gw.trim();
            if (ssid.isEmpty()) { req->send(400, "text/plain", "SSID required"); return; }

            // Auto-derive gateway (x.x.x.1) when static IP is given but gateway omitted
            if (!ip.isEmpty() && gw.isEmpty()) {
                int dot = ip.lastIndexOf('.');
                if (dot != -1) gw = ip.substring(0, dot + 1) + "1";
            }

            wifiPrefs.begin("wifi", false);
            wifiPrefs.putString("ssid", ssid);
            wifiPrefs.putString("pass", pass);
            wifiPrefs.putString("ip",   ip);
            wifiPrefs.putString("gw",   gw);
            wifiPrefs.end();

            req->send_P(200, "text/html", PORTAL_SAVED_HTML);
            delay(1500);
            ESP.restart();
        }
    );

    httpServer.begin();
    Serial.printf("[WiFi] Portal AP: \"%s\"  IP: %s\n",
                  AP_SSID, AP_IP.toString().c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
// STA (station) MODE  —  non-blocking
//
// patched3: setup() previously blocked up to 15 s here, freezing touch.
// We now run WiFi.begin() + the connection wait inside a one-shot FreeRTOS
// task pinned to Core 0 (alongside AsyncTCP).  The task:
//   1. Configures static IP if provided
//   2. Calls WiFi.begin() and polls WL_CONNECTED with vTaskDelay (yields CPU)
//   3. Starts the HTTP/WS server when the wait completes (success OR timeout)
//   4. Self-deletes
//
// The task does NOT touch the TFT.  The header WiFi indicator updates via
// the loop()-side RSSI poll, which detects wifiConnected→true within 5 s.
// ─────────────────────────────────────────────────────────────────────────────
struct StaConnectArgs {
    String ssid;
    String pass;
    String staticIP;
    String gateway;
};

static void staConnectTask(void* arg) {
    StaConnectArgs* args = (StaConnectArgs*)arg;

    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);

    // Apply static IP before begin() if the user configured one
    if (!args->staticIP.isEmpty()) {
        IPAddress ip, gw, subnet(255, 255, 255, 0), dns(8, 8, 8, 8);
        if (ip.fromString(args->staticIP)) {
            if (!gw.fromString(args->gateway)) {
                int dot = args->staticIP.lastIndexOf('.');
                String gwStr = (dot != -1)
                    ? args->staticIP.substring(0, dot + 1) + "1"
                    : "192.168.1.1";
                gw.fromString(gwStr);
            }
            WiFi.config(ip, gw, subnet, dns);
            Serial.printf("[WiFi] Static IP: %s  GW: %s\n",
                          ip.toString().c_str(), gw.toString().c_str());
        } else {
            Serial.println("[WiFi] Static IP parse failed — falling back to DHCP");
        }
    }

    WiFi.begin(args->ssid.c_str(), args->pass.c_str());

    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < CONNECT_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("[WiFi] Connected!  IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), (int)WiFi.RSSI());
    } else {
        wifiConnected = false;
        lastReconnectMs = millis();
        Serial.printf("[WiFi] Connect failed (status %d) — will retry every %u s\n",
                      WiFi.status(), RECONNECT_INTERVAL_MS / 1000);
    }

    // startFullServer registers handlers + calls httpServer.begin().
    // Doesn't touch TFT — safe from Core 0.
    startFullServer();

    delete args;
    vTaskDelete(NULL);
}

static void startStaMode(const String& ssid, const String& pass,
                          const String& staticIP, const String& gateway) {
    wifiServerMode = WMODE_STA;

    StaConnectArgs* args = new StaConnectArgs();
    args->ssid     = ssid;
    args->pass     = pass;
    args->staticIP = staticIP;
    args->gateway  = gateway;

    // Pin to Core 0 alongside AsyncTCP/sensor-event tasks.  Priority 1 = low.
    // 4 KB stack is generous for WiFi.begin + AsyncWebServer init.
    xTaskCreatePinnedToCore(staConnectTask, "WiFiConnect",
                            4096, args, 1, NULL, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
// FULL HTTP + WEBSOCKET SERVER  (STA mode)
// ─────────────────────────────────────────────────────────────────────────────
static void startFullServer() {
    // ── CORS / preflight ────────────────────────────────────────────────────
    // Without these headers, opening the HTML directly from a local file://
    // URL (or any cross-origin host) is blocked by the browser's CORS policy.
    // DefaultHeaders adds the headers to every response served by httpServer.
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin",  "*");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers", "Content-Type");

    wsServer.onEvent(onWsEvent);
    httpServer.addHandler(&wsServer);

    // Serve HTML from LittleFS /index.html
    httpServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (LittleFS.exists("/index.html")) {
            req->send(LittleFS, "/index.html", "text/html");
        } else {
            req->send(200, "text/plain",
                "No HTML found. Upload data/index.html via the LittleFS uploader.");
        }
    });

    // GET /state — full JSON snapshot (used by HTML on initial WiFi connect)
    httpServer.on("/state", HTTP_GET, [](AsyncWebServerRequest* req) {
        String out; buildFullStateJson(out);
        req->send(200, "application/json", out);
    });

    // POST /cmd — fallback for commands when WebSocket is not yet open.
    //
    // patched3 (CRITICAL): AsyncWebServer dispatches ALL HTTP body callbacks
    // on the AsyncTCP/lwIP task, which is pinned to Core 0.  Calling
    // handleWifiCommand() directly from here would execute analogWrite,
    // NVS writes, I²C-mutex acquisition, and TFT redraws (via
    // finalizeCalibration) on Core 0 while the main UI runs on Core 1 —
    // an immediate mutex/hardware collision.
    // We enqueue commands and let wifiLoop() (running on Core 1) dispatch
    // them, mirroring the WebSocket handler's behaviour.
    httpServer.on("/cmd", HTTP_POST,
        [](AsyncWebServerRequest* req) { /* headers only — body handled below */ },
        nullptr,
        [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
            char buf[len + 1];
            memcpy(buf, data, len);
            buf[len] = '\0';
            StaticJsonDocument<256> doc;
            if (deserializeJson(doc, buf) == DeserializationError::Ok) {
                for (JsonPair kv : doc.as<JsonObject>()) {
                    WifiCmd cmd;
                    strncpy(cmd.key, kv.key().c_str(), sizeof(cmd.key) - 1);
                    cmd.key[sizeof(cmd.key) - 1] = '\0';

                    String vStr;
                    if      (kv.value().is<const char*>()) vStr = kv.value().as<const char*>();
                    else if (kv.value().is<double>())       vStr = String(kv.value().as<double>(), 6);
                    else if (kv.value().is<long>())         vStr = String(kv.value().as<long>());
                    else if (kv.value().is<bool>())         vStr = kv.value().as<bool>() ? "1" : "0";
                    else serializeJson(kv.value(), vStr);

                    strncpy(cmd.val, vStr.c_str(), sizeof(cmd.val) - 1);
                    cmd.val[sizeof(cmd.val) - 1] = '\0';
                    xQueueSend(wifiCmdQueue, &cmd, (TickType_t)0);
                }
            }
            req->send(200, "application/json", "{\"ok\":1}");
        }
    );

    // POST /forget-wifi — clear credentials and return to portal on next restart.
    // Safe to run on Core 0: only NVS write (Preferences is internally serialised)
    // and ESP.restart() (core-agnostic). The delay() blocks the async task but
    // we're about to reboot, so request servicing is irrelevant.
    httpServer.on("/forget-wifi", HTTP_POST, [](AsyncWebServerRequest* req) {
        wifiPrefs.begin("wifi", false);
        wifiPrefs.clear();
        wifiPrefs.end();
        req->send(200, "text/plain", "WiFi credentials cleared. Restarting into setup portal…");
        delay(500);
        ESP.restart();
    });

    // CORS preflight — browsers send OPTIONS before any cross-origin POST with
    // a JSON body.  Catch every unknown route and reply 204 with the headers
    // already attached by DefaultHeaders.  Real 404s still get a body.
    httpServer.onNotFound([](AsyncWebServerRequest* req) {
        if (req->method() == HTTP_OPTIONS) {
            req->send(204);
        } else {
            req->send(404, "text/plain", "Not found");
        }
    });

    httpServer.begin();
    Serial.println("[WiFi] HTTP server listening on port 80 (CORS: *, queued /cmd)");
}

// ─────────────────────────────────────────────────────────────────────────────
// COMMAND DISPATCHER
// Called from wifiLoop() on Core 1 — safe to call TFT, analogWrite, i2cMutex.
//
// Key reference (mirrors HTML sendCommand calls):
//   obj, stepIndex, imgs, brightness, ledEnabled, ledDuty,
//   sensorSleep, highRefl, dimMs, sleepMs, theme, tint,
//   calWidth, demarcDist, calPoints, calStart, calCapture, calUndoPoint,
//   resetAll, resetBonds, testAlert
// ─────────────────────────────────────────────────────────────────────────────
static void handleWifiCommand(const char* key, const char* val) {
    float fVal = atof(val);
    int   iVal = atoi(val);

    // ── Objective ────────────────────────────────────────────────────────────
    if (strcmp(key, "obj") == 0) {
        if (iVal >= 1 && iVal <= 3) currentobj = iVal;
        // patched3: redraw the objective-buttons strip on the device so the
        // highlight follows the HTML change immediately.  Cheap (sprite blit).
        if (currentMode == MAIN) updateObjectiveButtons();

    // ── Stack step index ─────────────────────────────────────────────────────
    } else if (strcmp(key, "stepIndex") == 0) {
        stackStepIndex = constrain(iVal, 0, STEP_TABLE_LEN - 1);
        stackStepSize  = STEP_TABLE[stackStepIndex];
        if (currentMode == STACK_CALC) refreshStackCalcValues(true);

    // ── Stack image count ────────────────────────────────────────────────────
    } else if (strcmp(key, "imgs") == 0) {
        stackTotalImgs = constrain(iVal, 2, 50000);
        if (currentMode == STACK_CALC) refreshStackCalcValues(true);

    // ── Time per step (seconds, 0.1-60) ─────────────────────────────────────
    } else if (strcmp(key, "secStep") == 0) {
        stackTimePerStep = constrain(fVal, 0.1f, 60.0f);
        settings.stackTimePerStep = stackTimePerStep;
        preferences.begin("calib", false);
        preferences.putBytes("settings", &settings, sizeof(CalibData));
        preferences.end();
        if (currentMode == STACK_TIME) refreshStackTimeValues(true);

    // ── Screen brightness (LITE_PIN, analogWrite 1-255) ──────────────────────
    } else if (strcmp(key, "brightness") == 0) {
        currentBrightness = constrain(iVal, 1, 255);
        analogWrite(LITE_PIN, currentBrightness);
        settings.brightness = currentBrightness;
        preferences.begin("calib", false);
        preferences.putBytes("settings", &settings, sizeof(CalibData));
        preferences.end();
        if (currentMode == BRIGHTNESS_SETTINGS) refreshBrightnessSettingsValues(true);

    // ── Trigger LED enabled flag ──────────────────────────────────────────────
    } else if (strcmp(key, "ledEnabled") == 0) {
        ledEnabled = (iVal != 0);
        settings.ledEnabled = ledEnabled ? 1 : 0;
        analogWrite(TRIGGER_LED_PIN, ledEnabled ? (255 - currentLedDuty) : 255);
        preferences.begin("calib", false);
        preferences.putBytes("settings", &settings, sizeof(CalibData));
        preferences.end();
        if (currentMode == BRIGHTNESS_SETTINGS) drawBrightnessSettingsUI();  // toggle text+color changes

    // ── Trigger LED duty (0-127: max safe duty matches on-device slider) ──────
    } else if (strcmp(key, "ledDuty") == 0) {
        currentLedDuty = constrain(iVal, 0, 127);
        settings.ledDuty = (uint8_t)currentLedDuty;
        analogWrite(TRIGGER_LED_PIN, ledEnabled ? (255 - currentLedDuty) : 255);
        preferences.begin("calib", false);
        preferences.putBytes("settings", &settings, sizeof(CalibData));
        preferences.end();
        if (currentMode == BRIGHTNESS_SETTINGS) refreshBrightnessSettingsValues(true);

    // ── Sensor sleep / wake ──────────────────────────────────────────────────
    } else if (strcmp(key, "sensorSleep") == 0) {
        bool wantSleep = (iVal != 0);
        if (wantSleep && !sensorSleeping) {
            sensorSleeping = true;
            sensorState.store(0, std::memory_order_release);
            sensorHealth.store(0xFF000000UL, std::memory_order_release);
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                sensor.VL53L4CX_StopMeasurement();
                xSemaphoreGive(i2cMutex);
            }
        } else if (!wantSleep && sensorSleeping) {
            sensorEmaReset.store(true, std::memory_order_release);
            sensorSleeping = false;
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                applyHighReflConfig();
                sensor.VL53L4CX_StartMeasurement();
                xSemaphoreGive(i2cMutex);
            }
        }
        if (currentMode == SENSOR_INFO) drawSensorInfoUI();

    // ── High-reflectivity mode ────────────────────────────────────────────────
    } else if (strcmp(key, "highRefl") == 0) {
        highReflMode = (iVal != 0);
        if (!sensorSleeping) {
            if (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200))) {
                applyHighReflConfig();
                xSemaphoreGive(i2cMutex);
            }
        }
        if (currentMode == SENSOR_INFO) drawSensorInfoUI();

    // ── Screen dim timeout ───────────────────────────────────────────────────
    } else if (strcmp(key, "dimMs") == 0) {
        dimTimeoutMs = constrain((unsigned long)fVal, DIM_MIN_MS, DIM_MAX_MS);
        displayPrefsDirty   = true;
        displayNeedsRedraw  = true;   // patched3: trigger deferred redraw for SCREEN_TIMEOUT
        lastTintDragMs = millis();

    // ── Auto-sleep timeout ────────────────────────────────────────────────────
    } else if (strcmp(key, "sleepMs") == 0) {
        sleepTimeoutMs = constrain((unsigned long)fVal, SLEEP_MIN_MS, SLEEP_MAX_MS);
        displayPrefsDirty   = true;
        displayNeedsRedraw  = true;   // patched3
        lastTintDragMs = millis();

    // ── Theme index ──────────────────────────────────────────────────────────
    // Guard: only force a repaint when the value actually changed.  A no-op
    // command (e.g. an echo of current state from the web) must NOT trip
    // displayNeedsRedraw — that would cause a full-screen flicker on every
    // unrelated device-side button press.
    } else if (strcmp(key, "theme") == 0) {
        if (iVal >= 0 && iVal < NUM_THEMES && iVal != currentThemeIndex) {
            currentThemeIndex = iVal;
            refreshCachedThemeBg();
            displayPrefsDirty   = true;
            displayNeedsRedraw  = true;   // patched3: trigger deferred TFT repaint
            lastTintDragMs      = millis();
        }

    // ── Tint / intensity ─────────────────────────────────────────────────────
    } else if (strcmp(key, "tint") == 0) {
        int newTint = constrain(iVal, 0, 100);
        if (newTint != themeIntensity) {
            themeIntensity      = newTint;
            refreshCachedThemeBg();
            displayPrefsDirty   = true;
            displayNeedsRedraw  = true;   // patched3: trigger deferred TFT repaint
            lastTintDragMs      = millis();
        }

    // ── Calibration: photo width ──────────────────────────────────────────────
    } else if (strcmp(key, "calWidth") == 0) {
        sensorWidthPixels = constrain(fVal, 100.0f, 30000.0f);
        settings.sensorWidth = sensorWidthPixels;
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);

    // ── Calibration: demarcation distance ────────────────────────────────────
    } else if (strcmp(key, "demarcDist") == 0) {
        demarcationDist = constrain(fVal, 0.01f, 5.0f);
        settings.demarcation = demarcationDist;
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);

    // ── Calibration: target point count ──────────────────────────────────────
    } else if (strcmp(key, "calPoints") == 0) {
        nPoints = constrain(iVal, 3, 20);
        if (currentMode == CAL_SETTINGS) refreshCalSettingsValues(true);

    // ── Calibration: start a fresh session (clear prior pointsCaptured) ──────
    //    HTML sends this on entering cal mode so that captures land in slot 0.
    //    Without it, leftover points from a previous physical-screen capture
    //    would cause new HTML captures to mix into the old data and
    //    finalizeCalibration() would fire early.
    } else if (strcmp(key, "calStart") == 0) {
        pointsCaptured = 0;

    // ── Calibration: capture a point ─────────────────────────────────────────
    //    iVal = pixel count measured in the photo at current distance
    //    Formula: fov_mm = (demarcationDist / pixels) * sensorWidthPixels
    } else if (strcmp(key, "calCapture") == 0) {
        if (iVal < 1) return;  // guard against zero/negative pixel counts
        uint32_t st   = sensorState.load(std::memory_order_acquire);
        bool     valid = (st >> 31) & 1;
        float    dist  = (float)(st & 0x7FFFFFFF);          // mm integer from EMA
        float    fov   = (demarcationDist / (float)iVal) * sensorWidthPixels;

        if (!valid || pointsCaptured >= 20) return;   // no valid range or buffer full

        int slot = pointsCaptured;
        distPoints[slot] = dist;
        fovPoints[slot]  = fov;
        pointsCaptured++;

        // Echo the result back — HTML review list needs the actual dist+fov values
        {
            StaticJsonDocument<128> ack;
            JsonObject r = ack.createNestedObject("calPointResult");
            r["idx"]  = slot;
            r["dist"] = roundf(dist * 10.0f) / 10.0f;
            r["fov"]  = roundf(fov * 10000.0f) / 10000.0f;
            String out; serializeJson(ack, out);
            wsServer.textAll(out);
        }

        // When all target points are captured, run the least-squares fit
        if (pointsCaptured >= nPoints) {
            finalizeCalibration();   // computes CTRLX/CTRLY, saves NVS, sets isCustomCalib
            // finalizeCalibration() calls drawSuccessScreen() — the physical display
            // will show the CAL_SUCCESS screen briefly (same as touch-triggered flow)
            String out; buildFullStateJson(out);
            wsServer.textAll(out);
        }

    // ── Calibration: undo last captured point ────────────────────────────────
    } else if (strcmp(key, "calUndoPoint") == 0) {
        if (pointsCaptured > 0) pointsCaptured--;

    // ── Reset to factory calibration ─────────────────────────────────────────
    //    NOTE: we manually restore defaults rather than calling finalizeCalibration()
    //    to avoid setting isCustom=1 and showing the TFT success screen.
    } else if (strcmp(key, "resetAll") == 0) {
        nPoints        = FACTORY_N;
        pointsCaptured = FACTORY_N;
        for (int i = 0; i < FACTORY_N; i++) {
            distPoints[i] = FACTORY_DIST[i];
            fovPoints[i]  = FACTORY_FOV[i];
        }
        CTRLX        = Config::DEFAULT_CTRL_X;
        CTRLY        = Config::DEFAULT_CTRL_Y;
        CALIB_ERROR  = Config::DEFAULT_CALIB_ERROR;
        CALIB_R2     = 0.994f;
        isCustomCalib = false;
        // Persist the reset
        settings.magic      = CALIB_MAGIC;
        settings.ctrlX      = CTRLX;
        settings.ctrlY      = CTRLY;
        settings.sensorWidth  = sensorWidthPixels;
        settings.demarcation  = demarcationDist;
        settings.calibError = CALIB_ERROR;
        settings.calibR2    = CALIB_R2;
        settings.isCustom   = 0;
        preferences.begin("calib", false);
        preferences.putBytes("settings", &settings, sizeof(CalibData));
        preferences.end();
        // Push updated calibration state to all HTML clients
        String out; buildFullStateJson(out);
        wsServer.textAll(out);

    // ── Reset Bluetooth bonds (restarts device) ───────────────────────────────
    // patched3: no-op if NimBLE was never initialized.
    } else if (strcmp(key, "resetBonds") == 0) {
        if (NimBLEDevice::getServer() != nullptr) {
            NimBLEDevice::deleteAllBonds();
            delay(200);
            ESP.restart();
        } else {
            Serial.println("[WiFi] resetBonds ignored — NimBLE not initialised");
        }

    // ── BLE test alert (sends HID F12 keypress) ───────────────────────────────
    // patched3: extra guard on keyboardInput (null until BLE is up).
    } else if (strcmp(key, "testAlert") == 0) {
        if (keyboardInput != nullptr && btConnHandle != 0xFFFF) {
            uint8_t press[8]   = {0x00, 0x00, HID_KEY_TRIGGER, 0, 0, 0, 0, 0};
            uint8_t release[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            keyboardInput->setValue(press, 8);
            keyboardInput->notify();
            delay(20);
            keyboardInput->setValue(release, 8);
            keyboardInput->notify();
        } else {
            Serial.println("[WiFi] testAlert ignored — BLE not ready");
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// JSON BUILDERS
// ─────────────────────────────────────────────────────────────────────────────

// Full state — sent on GET /state and when a new WebSocket client connects.
// Pass includeCalGraph=false on reconnect / periodic sends; true only on new-client
// connect, after calibration changes, and on explicit /state requests.
static void buildFullStateJson(String& out, bool includeCalGraph) {
    // Read all atomics in one burst
    uint32_t st  = sensorState.load(std::memory_order_acquire);
    uint32_t hst = sensorHealth.load(std::memory_order_acquire);
    uint32_t amb = sensorAmbient.load(std::memory_order_acquire);
    bool    valid   = (st >> 31) & 1;
    int     dist_mm = st & 0x7FFFFFFF;
    uint8_t status  = (hst >> 24) & 0xFF;
    float   signal  = (hst & 0xFFFFFF) / 1000.0f;
    float   ambient = (amb & 0xFFFFFF) / 1000.0f;

    // BT
    bool btConn = (NimBLEDevice::getServer() != nullptr &&
                   NimBLEDevice::getServer()->getConnectedCount() > 0);
    int8_t  btRSSI = 0;
    uint16_t btMTU = 0;
    if (btConn && btConnHandle != 0xFFFF) {
        ble_gap_conn_rssi(btConnHandle, &btRSSI);
        btMTU = NimBLEDevice::getServer()->getPeerMTU(btConnHandle);
    }

    // FOV
    float mult = (currentobj == 1) ? mul_5x : (currentobj == 2) ? mul_10x : mul_20x;
    float fov  = valid ? (mult * (dist_mm * CTRLX + CTRLY)) : 0.0f;

    DynamicJsonDocument doc(3072);

    // ── Live sensor ──────────────────────────────────────────────────────────
    doc["dist"]          = valid ? roundf(dist_mm * 10.0f) / 10.0f : 0;
    doc["fov"]           = roundf(fov * 1000.0f) / 1000.0f;
    doc["signal"]        = roundf(signal * 10.0f) / 10.0f;
    doc["sensorSignal"]  = doc["signal"];       // HTML uses both keys
    doc["sensorAmbient"] = roundf(ambient * 100.0f) / 100.0f;
    doc["sensorStatus"]  = status;
    doc["trigger"]       = shutterActive ? 1 : 0;   // patched3

    // ── Settings ─────────────────────────────────────────────────────────────
    doc["obj"]           = currentobj;
    doc["stepIndex"]     = stackStepIndex;
    doc["imgs"]          = stackTotalImgs;
    doc["brightness"]    = currentBrightness;
    doc["ledEnabled"]    = ledEnabled ? 1 : 0;
    doc["ledDuty"]       = currentLedDuty;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode ? 1 : 0;
    doc["dimMs"]         = (uint32_t)dimTimeoutMs;
    doc["sleepMs"]       = (uint32_t)sleepTimeoutMs;
    doc["theme"]         = currentThemeIndex;
    doc["tint"]          = themeIntensity;

    // ── Bluetooth ────────────────────────────────────────────────────────────
    // patched3: guard against uninitialized NimBLE — getAddress() and
    // getNumBonds() can hang/crash if NimBLEDevice::init() was never called
    // (currently disabled while we work out heap budget).
    bool bleReady = (NimBLEDevice::getServer() != nullptr);
    doc["btConnected"]   = btConn ? 1 : 0;
    doc["btMac"]         = bleReady ? NimBLEDevice::getAddress().toString().c_str()
                                    : "00:00:00:00:00:00";
    doc["btBonds"]       = bleReady ? NimBLEDevice::getNumBonds() : 0;
    doc["btDevices"]     = btConn ? 1 : 0;
    doc["btRSSI"]        = btConn ? (int)btRSSI : 0;
    doc["btMTU"]         = btMTU;

    // ── Calibration fit ───────────────────────────────────────────────────────
    doc["isCustomCalib"] = isCustomCalib ? 1 : 0;
    doc["fovSlope"]      = roundf(CTRLX * 1e6f) / 1e6f;   // 6 dp — slope is ~0.000xxx
    doc["fovIntercept"]  = roundf(CTRLY * 1000.0f) / 1000.0f;
    doc["calibError"]    = roundf(CALIB_ERROR * 1000.0f) / 1000.0f;
    doc["calibR2"]       = roundf(CALIB_R2 * 1000.0f) / 1000.0f;
    doc["calWidth"]      = (int)sensorWidthPixels;
    doc["demarcDist"]    = roundf(demarcationDist * 100.0f) / 100.0f;
    doc["calPoints"]     = nPoints;

    // Cal graph scatter data — only sent when calibration changes or on new connect.
    if (includeCalGraph) {
        int n = constrain(pointsCaptured, 0, 20);
        JsonArray pts = doc.createNestedArray("calGraphPoints");
        for (int i = 0; i < n; i++) {
            JsonObject p = pts.createNestedObject();
            p["dist"] = roundf(distPoints[i] * 10.0f) / 10.0f;
            p["fov"]  = roundf(fovPoints[i] * 1000.0f) / 1000.0f;
        }
    }

    // ── Memory ───────────────────────────────────────────────────────────────
    doc["freeHeap"]   = ESP.getFreeHeap()    / 1024;
    doc["totalHeap"]  = ESP.getHeapSize()    / 1024;
    doc["lowestFree"] = ESP.getMinFreeHeap() / 1024;
    doc["spriteKB"]   = 92;   // 240×48+240×45+80×40+44×12+120×40+240×67 = ~91.7 KB (PSRAM)
    doc["psramFree"]  = ESP.getFreePsram()   / 1024;
    doc["psramTotal"] = ESP.getPsramSize()   / 1024;
    doc["cpu"]        = getCpuFrequencyMhz();
    doc["sdk"]        = ESP.getSdkVersion();

    // ── About / build ─────────────────────────────────────────────────────────
    doc["buildDate"]   = __DATE__;
    doc["buildTime"]   = __TIME__;
    doc["sketchKB"]    = ESP.getSketchSize()      / 1024;
    doc["freeFlashKB"] = ESP.getFreeSketchSpace() / 1024;
    doc["chipInfo"]    = ESP.getChipModel();
    // sourceLines is managed by the HTML default (3866) — not sent here to save space

    // ── WiFi ─────────────────────────────────────────────────────────────────
    doc["wifiSSID"]   = WiFi.SSID();
    doc["wifiRSSI"]   = (int)WiFi.RSSI();

    serializeJson(doc, out);
}

// Fast telemetry — 10 Hz live sensor values + WiFi/BT signal levels
static void buildFastTelemJson(String& out) {
    uint32_t st  = sensorState.load(std::memory_order_acquire);
    uint32_t hst = sensorHealth.load(std::memory_order_acquire);
    uint32_t amb = sensorAmbient.load(std::memory_order_acquire);
    bool    valid   = (st >> 31) & 1;
    int     dist_mm = st & 0x7FFFFFFF;                              // raw EMA — matches TFT DISTANCE display
    float   avgDist = sensorAvgDist.load(std::memory_order_acquire) / 10.0f; // 5-sample avg — matches TFT AVG FOV
    uint8_t status  = (hst >> 24) & 0xFF;
    float   signal  = (hst & 0xFFFFFF) / 1000.0f;
    float   ambient = (amb & 0xFFFFFF) / 1000.0f;

    float mult = (currentobj == 1) ? mul_5x : (currentobj == 2) ? mul_10x : mul_20x;
    float fov  = valid ? (mult * (avgDist * CTRLX + CTRLY)) : 0.0f;

    int8_t btRSSI = 0;
    bool btConn = (NimBLEDevice::getServer() != nullptr &&
                   NimBLEDevice::getServer()->getConnectedCount() > 0);
    if (btConn && btConnHandle != 0xFFFF)
        ble_gap_conn_rssi(btConnHandle, &btRSSI);

    // Short keys save ~30 bytes/frame at 30 Hz.  HTML applyFullState() unpacks
    // them back to long names at the top of the function.
    StaticJsonDocument<256> doc;
    doc["d"]  = valid ? roundf(dist_mm * 10.0f) / 10.0f : 0;   // dist
    doc["f"]  = roundf(fov * 1000.0f) / 1000.0f;                // fov
    doc["e"]  = (int)sensorErrInt.load(std::memory_order_acquire); // fovErr (2σ × 100, centimm)
    doc["sg"] = roundf(signal * 10.0f) / 10.0f;                 // signal
    doc["a"]  = roundf(ambient * 100.0f) / 100.0f;              // sensorAmbient
    doc["ss"] = status;                                          // sensorStatus
    doc["t"]  = shutterActive ? 1 : 0;                          // trigger
    doc["wr"] = (int)WiFi.RSSI();                                // wifiRSSI
    if (btConn) doc["br"] = (int)btRSSI;                        // btRSSI

    serializeJson(doc, out);
}

// Settings-only push — used by the state-change detector in wifiLoop().
// Compact (≤ 400 bytes) so wsServer.textAll() doesn't stall Core 1 the way a
// full-state frame (2 KB, DynamicJsonDocument 3072) does.
static void buildSettingsJson(String& out) {
    StaticJsonDocument<512> doc;
    doc["obj"]           = currentobj;
    doc["stepIndex"]     = stackStepIndex;
    doc["imgs"]          = stackTotalImgs;
    doc["secStep"]       = stackTimePerStep;
    doc["brightness"]    = currentBrightness;
    doc["ledEnabled"]    = ledEnabled ? 1 : 0;
    doc["ledDuty"]       = currentLedDuty;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode ? 1 : 0;
    doc["dimMs"]         = (uint32_t)dimTimeoutMs;
    doc["sleepMs"]       = (uint32_t)sleepTimeoutMs;
    doc["theme"]         = currentThemeIndex;
    doc["tint"]          = themeIntensity;
    serializeJson(doc, out);
}

// Slow telemetry — 5 s memory stats + BT connectivity changes
static void buildSlowTelemJson(String& out) {
    bool btConn = (NimBLEDevice::getServer() != nullptr &&
                   NimBLEDevice::getServer()->getConnectedCount() > 0);
    uint16_t btMTU = 0;
    if (btConn && btConnHandle != 0xFFFF)
        btMTU = NimBLEDevice::getServer()->getPeerMTU(btConnHandle);

    StaticJsonDocument<320> doc;
    doc["freeHeap"]      = ESP.getFreeHeap()    / 1024;
    doc["lowestFree"]    = ESP.getMinFreeHeap() / 1024;
    doc["psramFree"]     = ESP.getFreePsram()   / 1024;
    doc["psramTotal"]    = ESP.getPsramSize()   / 1024;
    doc["btConnected"]   = btConn ? 1 : 0;
    doc["btDevices"]     = btConn ? 1 : 0;
    doc["btMTU"]         = btMTU;
    doc["wifiSSID"]      = WiFi.SSID();
    doc["obj"]           = currentobj;
    doc["sensorSleeping"]= sensorSleeping ? 1 : 0;
    doc["highReflMode"]  = highReflMode   ? 1 : 0;

    serializeJson(doc, out);
}

// ─────────────────────────────────────────────────────────────────────────────
// ACCESSORS  —  read-only view of wifi state for drawWifiIndicator() in the
//   main tab.  Because the main tab is concatenated BEFORE this tab, its code
//   sees these functions only via the forward declarations it contains.
//   (Arduino builds all .ino files into one TU in alphabetical order, with the
//   sketch-name file first.)
// ─────────────────────────────────────────────────────────────────────────────
bool   wifiIsConnected() { return wifiConnected; }
bool   wifiIsPortal()    { return wifiServerMode == WMODE_PORTAL; }
int    wifiGetRSSI()     { return wifiConnected ? (int)WiFi.RSSI() : 0; }
String wifiGetIP()       { if (wifiServerMode == WMODE_PORTAL) return AP_IP.toString();
                           return wifiConnected ? WiFi.localIP().toString() : ""; }
String wifiGetSSID()     { if (wifiServerMode == WMODE_PORTAL) return String(AP_SSID);
                           return wifiConnected ? WiFi.SSID() : ""; }

// Called from main.ino when a stack sequence finishes (the same moment the
// BLE HID key would have fired).  Pushes a one-shot WS event so the web UI
// can fire a browser Notification — BLE-free fallback for the stacker prompt.
void wifiNotifyStackComplete() {
    if (wsServer.count() == 0) return;
    wsServer.textAll("{\"stackDone\":1}");
}

// ─────────────────────────────────────────────────────────────────────────────
// wifiForgetAndRestart()  —  called from handleWifiInfoTouch() in the main tab.
//   Clears saved credentials and reboots into captive-portal mode.
// ─────────────────────────────────────────────────────────────────────────────
void wifiForgetAndRestart() {
    wifiPrefs.begin("wifi", false);
    wifiPrefs.clear();
    wifiPrefs.end();
    Serial.println("[WiFi] Credentials cleared — restarting into setup portal");
    delay(300);
    ESP.restart();
}
