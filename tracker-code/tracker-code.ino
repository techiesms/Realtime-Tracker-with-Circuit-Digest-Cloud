/*
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║         GPS TRACKER  —  XIAO ESP32-C3 + GP02 GPS + SIM800L GSM           ║
 * ║              GSM/GPRS upload to CircuitDigest Geolinker cloud            ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * ── HARDWARE ─────────────────────────────────────────────────────────────────
 *   XIAO ESP32-C3
 *   GP02 GPS    → TX→GPIO6(D4),  RX→GPIO7(D5)
 *   SIM800L     → RX→GPIO20,     TX→GPIO21
 *   SOS button  → D2  (INPUT_PULLUP)
 *   CFG button  → D1  (INPUT_PULLUP)
 *   Mic → SIM800L MIC+/MIC-  |  Speaker → SIM800L SPK+/SPK-
 *
 * ── PROGRAM FLOW (OVERVIEW) ──────────────────────────────────────────────────
 *
 *  [BOOT]
 *   Power on
 *   └─ Init SPIFFS, pins, GPS & GSM serial
 *   └─ Load config.json  (API key, Device ID, APN, SOS numbers, upload interval)
 *        │
 *        ├─ Config missing / D1 held at boot?
 *        │   └─ Open WiFi portal  (SSID: GPS_Tracker_Setup / gps123456)
 *        │       └─ User fills: API key · APN · SOS numbers · interval
 *        │           └─ Save to SPIFFS → hard-reset SIM800L (AT+CFUN=1,1)
 *        │
 *        └─ Config complete → WiFi OFF
 *
 *   └─ initializeGSM()   — AT handshake, SMS text mode, caller-ID, audio
 *   └─ initializeGPRS()  — wait for network, set APN, open bearer, get IP
 *   └─ sendStartupSMS()  — one-time "device online" SMS to all SOS numbers
 *
 *  [MAIN LOOP]  (repeats every 100 ms)
 *   ├─ readGPS()          — parse NMEA sentences, update fix & coordinates
 *   ├─ processSMS()       — read SIM800L serial for incoming SMS / calls
 *   ├─ handleSOSButton()  — D2 held 5 s → triggerSOS()
 *   ├─ handleConfigButton()— D1 held 3 s → reopen config portal
 *   └─ Upload timer       — every N s: if GPS fixed & GPRS up → POST to cloud
 *
 *  [EVENTS]
 *   SOS triggered
 *   └─ SMS with Google Maps link → all 3 SOS numbers
 *   └─ Voice call → primary number only
 *
 *   Incoming SMS  "SEND LOCATION"  (from a saved SOS number)
 *   └─ normalizePhone() — last-10-digit match (handles +91 / 0 prefix etc.)
 *   └─ Reply: Maps URL + speed + satellite count + timestamp
 *
 *   Incoming call
 *   └─ Auto-answer (ALL calls, or SOS numbers only — toggle with 'call mode')
 *   └─ Auto-hangup after 120 s
 *
 *   Geolinker upload
 *   └─ AT+HTTPINIT → POST JSON {device_id, lat, long, timestamp}
 *   └─ Authorization header carries API key from config
 *   └─ HTTP 200 = success
 *
 *  [STORAGE  —  SPIFFS flash, survives power cycles]
 *   config.json    → API key, APN, SOS numbers x3, upload interval
 *   sms_sent.flag  → prevents duplicate startup SMS after every reboot
 *
 *  [SERIAL COMMANDS]  (115200 baud)
 *   status · send now · reset gprs · test sms · signal · network
 *   hangup · call mode · vol up/down · mute/unmute · help
 * ─────────────────────────────────────────────────────────────────────────────
 */
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <SPIFFS.h>

// ── File paths ────────────────────────────────────────────────────────────────
#define CONFIG_FILE "/config.json"
#define SMS_FLAG_FILE "/sms_sent.flag"

// ── Geolinker ─────────────────────────────────────────────────────────────────
const char* geolinkerHost = "www.circuitdigest.cloud";
const char* geolinkerPath = "/api/v1/geolinker";

// ── Runtime config (loaded from SPIFFS — NO hardcoded defaults) ───────────────
String apiKey = "";  // must be entered via portal
String apnStr = "";  // must be entered via portal
String gprsUser = "";
String gprsPass = "";
String deviceID = "";

// ── SOS numbers ───────────────────────────────────────────────────────────────
String sosNumbers[3];
const String EMERGENCY_MESSAGE = "SOS EMERGENCY! Location: ";

// ── GPS ───────────────────────────────────────────────────────────────────────
TinyGPSPlus gps;
HardwareSerial GPSSerial(1);
#define GPS_RX_PIN 6
#define GPS_TX_PIN 7

// ── GSM ───────────────────────────────────────────────────────────────────────
HardwareSerial GSMSerial(0);
#define GSM_RX_PIN 20
#define GSM_TX_PIN 21

#define SIM800_RST_PIN 10

// ── Buttons ───────────────────────────────────────────────────────────────────
#define SOS_BUTTON_PIN D2
#define CONFIG_BUTTON_PIN D1
const unsigned long SOS_PRESS_DURATION = 5000;
const unsigned long CONFIG_PRESS_DURATION = 3000;

unsigned long sosButtonPressStart = 0;
bool sosButtonPressed = false;
unsigned long configButtonPressStart = 0;
bool configButtonPressed = false;
bool configModeTriggered = false;

// ── LEDs ──────────────────────────────────────────────────────────────────────
const int gpsStatusLED = 3;
const int sosStatusLED = 0;
const int configLED = 1;

// ── Web server ────────────────────────────────────────────────────────────────
WebServer server(80);

// ── GPS data struct ───────────────────────────────────────────────────────────
struct GPSData {
  double latitude;
  double longitude;
  String timestamp;
  double altitude;
  double speed;
  int satellites;
};

// ── State ─────────────────────────────────────────────────────────────────────
bool gpsFixed = false;
bool gsmReady = false;
bool gprsConnected = false;
bool callInProgress = false;
GPSData latestGPSData;

unsigned long lastGPSPrint = 0;
unsigned long lastGSMCheck = 0;
unsigned long lastGeolinkerUpload = 0;

const unsigned long gpsPrintInterval = 5000;
const unsigned long gsmCheckInterval = 30000;

unsigned long geolinkerUploadInterval = 60000;

// ── Auto-answer ───────────────────────────────────────────────────────────────
bool autoAnswerAll = true;
const unsigned long MAX_CALL_DURATION = 120;
const int MIC_GAIN = 5;
String incomingCallerID = "";
unsigned long callAnsweredAt = 0;

// ─────────────────────────────────────────────────────────────────────────────
// HTML — Config Portal
// ─────────────────────────────────────────────────────────────────────────────
const char HTML_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>GPS Tracker Setup</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:Arial,sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);
     min-height:100vh;display:flex;align-items:center;justify-content:center;padding:20px}
.card{background:#fff;padding:36px 30px;border-radius:16px;
      box-shadow:0 12px 40px rgba(0,0,0,.25);max-width:460px;width:100%}
h1{color:#333;text-align:center;margin-bottom:4px;font-size:22px}
.sub{text-align:center;color:#888;margin-bottom:22px;font-size:13px}
.section{background:#f4f7ff;border-left:4px solid #667eea;padding:11px 14px;
         border-radius:6px;margin-bottom:18px}
.section h3{color:#445;font-size:14px;margin-bottom:3px}
.section p{color:#667;font-size:12px;line-height:1.5}
.section p b{color:#333}
label{display:block;color:#444;font-weight:600;font-size:14px;margin-bottom:5px;margin-top:14px}
.note{font-size:11px;color:#e67e22;font-weight:600;margin-bottom:6px;margin-top:-2px}
input[type=tel],input[type=text],input[type=password],select{
  width:100%;padding:11px 12px;border:2px solid #ddd;border-radius:8px;
  font-size:15px;background:#fff;margin-bottom:2px}
input:focus,select:focus{outline:none;border-color:#667eea}
.row{display:flex;gap:8px;align-items:center}
.row input{flex:1;margin-bottom:0}
.show-btn{padding:11px 13px;background:#667eea;color:#fff;border:none;
          border-radius:8px;cursor:pointer;font-size:12px;white-space:nowrap;flex-shrink:0}
.custom-apn{display:none;margin-top:8px}
hr{border:none;border-top:2px dashed #e5e5e5;margin:20px 0}
.req{color:#c0392b;margin-left:2px}
button[type=submit]{width:100%;background:linear-gradient(135deg,#667eea,#764ba2);
  color:#fff;padding:14px;border:none;border-radius:8px;font-size:17px;
  font-weight:700;cursor:pointer;margin-top:14px;letter-spacing:.3px}
button[type=submit]:hover{opacity:.93}
</style></head>
<body>
<div class="card">
  <h1>&#128225; GPS Tracker</h1>
  <p class="sub">Device Configuration Portal</p>

  <form action="/save" method="POST">

    <!-- ── API KEY ── -->
    <div class="section">
      <h3>&#128273; CircuitDigest API Key <span style="color:#c0392b">*</span></h3>
      <p>Get your key from <b>circuitdigest.cloud</b> dashboard.<br>
         This is required — the device will not start GSM without it.</p>
    </div>
    <label>API Key <span class="req">*</span></label>
    <div class="row">
      <input type="password" id="apikey" name="apikey"
             placeholder="cd_dev_XXXXXX_XXXXXXXX" maxlength="80">
      <button type="button" class="show-btn"
        onclick="var f=document.getElementById('apikey');
                 f.type=f.type==='password'?'text':'password';
                 this.textContent=f.type==='password'?'Show':'Hide'">Show</button>
    </div>
    <hr>

<div class="section">
  <h3>Device ID <span style="color:#c0392b">*</span></h3>
  <p>
    Enter the <b>same Device ID</b> you created on the CircuitDigest portal.<br>
    Must match exactly, otherwise data will not appear.
  </p>
</div>

<label>Device ID <span class="req">*</span></label>
<input type="text" name="deviceid" placeholder="e.g. XIAO_C3_GPS_01" maxlength="40">

    <hr>

    <!-- ── APN ── -->
    <div class="section">
      <h3>&#128225; Mobile Data APN</h3>
      <p><b>Vi (Vodafone-Idea):</b> www or portalnmms<br>
         <b>Airtel:</b> airtelgprs.com<br>
         <b>BSNL:</b> bsnlnet<br>
    </div>
    <label>Select APN <span class="req">*</span></label>
    <select id="apnSel" name="apn" onchange="
      document.getElementById('customApn').style.display=
        this.value==='custom'?'block':'none'">
      <option value="">-- Select your operator --</option>
      <option value="www">Vi / Vodafone-Idea — www</option>
      <option value="airtelgprs.com">Airtel — airtelgprs.com</option>
      <option value="bsnlnet">BSNL — bsnlnet</option>
      <option value="custom">Custom (enter manually)</option>
    </select>
    <div class="custom-apn" id="customApn">
      <label style="margin-top:8px">Custom APN value</label>
      <input type="text" name="apn_custom" placeholder="e.g. internet">
    </div>

    <hr>
<div class="section">
  <h3>Data Upload Interval</h3>
</div>
<label>Select Interval</label>
<select name="interval">
  <option value="15000">15 seconds</option>
  <option value="30000">30 seconds</option>
  <option value="60000">1 minute</option>
  <option value="300000">5 minutes</option>
  <option value="600000">10 minutes</option>
</select>

    <hr>

    <!-- ── SOS NUMBERS ── -->
    <div class="section">
      <h3>&#128222; Emergency SOS Numbers</h3>
      <p>Enter up to 3 numbers in international format: <b>+919876543210</b></p>
    </div>

    <label>Primary SOS Number <span class="req">*</span></label>
    <p class="note">&#9888; This number will also receive the emergency voice call</p>
    <input type="tel" name="sos1" placeholder="+919876543210" pattern="[0-9+]{10,15}">

    <label>Secondary SOS Number</label>
    <input type="tel" name="sos2" placeholder="+919876543211" pattern="[0-9+]{10,15}">

    <label>Tertiary SOS Number</label>
    <input type="tel" name="sos3" placeholder="+919876543212" pattern="[0-9+]{10,15}">

    <button type="submit">&#10003; Save &amp; Start Device</button>
  </form>
</div>
</body></html>
)rawliteral";

// Success page — countdown stops at 0, never goes negative
const char SUCCESS_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Saved!</title>
<style>
body{font-family:Arial,sans-serif;background:linear-gradient(135deg,#667eea,#764ba2);
     min-height:100vh;display:flex;align-items:center;justify-content:center}
.card{background:#fff;padding:44px 36px;border-radius:16px;text-align:center;max-width:380px;width:90%}
.check{font-size:72px;color:#27ae60;line-height:1}
h1{color:#333;margin:16px 0 8px;font-size:22px}
p{color:#666;font-size:14px;margin-bottom:6px}
.cd{font-size:52px;color:#667eea;font-weight:700;margin:14px 0}
.info{font-size:13px;color:#888;margin-top:8px}
</style></head><body>
<div class="card">
  <div class="check">&#10003;</div>
  <h1>Configuration Saved!</h1>
  <p>GSM &amp; GPRS are starting up.<br>You can close this page.</p>
  <div class="cd" id="c">5</div>
  <p class="info">Portal closing in <span id="s">5</span> seconds</p>
</div>
<script>
  // Countdown stops at 0 — never goes negative
  var n = 5;
  var iv = setInterval(function(){
    if(n > 0) n--;
    document.getElementById('c').textContent = n;
    document.getElementById('s').textContent = n;
    if(n === 0) clearInterval(iv);
  }, 1000);
</script>
</body></html>
)rawliteral";

// Error page — shown when required fields are missing
const char ERROR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Missing Fields</title>
<style>
body{font-family:Arial,sans-serif;background:linear-gradient(135deg,#e74c3c,#c0392b);
     min-height:100vh;display:flex;align-items:center;justify-content:center}
.card{background:#fff;padding:36px;border-radius:16px;text-align:center;max-width:380px;width:90%}
.icon{font-size:60px;color:#e74c3c}
h2{color:#333;margin:12px 0}p{color:#666;margin-bottom:18px}
a{display:inline-block;background:#667eea;color:#fff;padding:12px 28px;
  border-radius:8px;text-decoration:none;font-weight:600}
</style></head><body>
<div class="card">
  <div class="icon">&#9888;</div>
  <h2>Required Fields Missing</h2>
  <p id="msg">Please fill in all required fields.</p>
  <a href="/">&#8592; Go Back</a>
</div>
</body></html>
)rawliteral";

// ─────────────────────────────────────────────────────────────────────────────
// SPIFFS — unified config
// ─────────────────────────────────────────────────────────────────────────────
void saveConfig(const String& n1, const String& n2, const String& n3,
                const String& key, const String& apn,
                int interval, const String& devID) {
  File file = SPIFFS.open(CONFIG_FILE, "w");
  if (!file) {
    Serial.println("✗ Cannot open config for write");
    return;
  }
  StaticJsonDocument<512> doc;
  doc["sos1"] = n1;
  doc["sos2"] = n2;
  doc["sos3"] = n3;
  doc["apikey"] = key;
  doc["apn"] = apn;
  doc["interval"] = interval;
  doc["deviceid"] = devID;

  serializeJson(doc, file);
  file.close();
  Serial.println("✓ Config saved");
}

/**
 * Load config from SPIFFS.
 * Always populates sosNumbers[], apiKey, apnStr from file.
 * Returns TRUE only when BOTH apiKey and apnStr are non-empty
 * (i.e. device is fully configured and can start GSM).
 */
bool loadConfig() {
  if (!SPIFFS.exists(CONFIG_FILE)) {
    Serial.println("No config file found");
    return false;
  }
  File file = SPIFFS.open(CONFIG_FILE, "r");
  if (!file) return false;

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, file)) {
    file.close();
    return false;
  }

  sosNumbers[0] = doc["sos1"] | "";
  sosNumbers[1] = doc["sos2"] | "";
  sosNumbers[2] = doc["sos3"] | "";
  apiKey = doc["apikey"] | "";
  apnStr = doc["apn"] | "";
  geolinkerUploadInterval = doc["interval"] | 60000;
  deviceID = doc["deviceid"] | "";


  file.close();
  Serial.println("✓ Config loaded");

  if (deviceID.length() == 0) {
    Serial.println("✗ Device ID missing — portal required");
    return false;
  }

  if (apiKey.length() == 0) {
    Serial.println("✗ API key missing — portal required");
    return false;
  }
  if (apnStr.length() == 0) {
    Serial.println("✗ APN missing — portal required");
    return false;
  }
  Serial.println("✓ API key: " + apiKey);
  Serial.println("✓ APN:     " + apnStr);
  return true;
}

bool isStartupSMSSent() {
  return SPIFFS.exists(SMS_FLAG_FILE);
}

void markStartupSMSSent() {
  File f = SPIFFS.open(SMS_FLAG_FILE, "w");
  if (f) {
    f.print("sent");
    f.close();
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Web server handlers
// ─────────────────────────────────────────────────────────────────────────────
void handleRoot() {
  server.send_P(200, "text/html", HTML_PAGE);
}

void handleSave() {
  if (server.method() != HTTP_POST) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  String s1 = server.arg("sos1");
  s1.trim();
  String s2 = server.arg("sos2");
  s2.trim();
  String s3 = server.arg("sos3");
  s3.trim();
  String ak = server.arg("apikey");
  ak.trim();
  String apn = server.arg("apn");
  apn.trim();
  String apnC = server.arg("apn_custom");
  apnC.trim();
  String devID = server.arg("deviceid");
  devID.trim();

  // If "custom" was selected, use the manually typed value
  if (apn == "custom") apn = apnC;

  int interval = server.arg("interval").toInt();
  if (interval <= 0) interval = 60000;


  // ── Validation ────────────────────────────────────────────────────────────
  if (ak.length() == 0) {
    // Return error page — API key is mandatory
    String err = String(ERROR_PAGE);
    err.replace("Please fill in all required fields.",
                "The <b>API Key</b> is required. Please go back and enter it.");
    server.send(400, "text/html", err);
    return;
  }
  if (apn.length() == 0) {
    String err = String(ERROR_PAGE);
    err.replace("Please fill in all required fields.",
                "Please select or enter your <b>APN</b> and try again.");
    server.send(400, "text/html", err);
    return;
  }
  if (s1.length() < 10) {
    String err = String(ERROR_PAGE);
    err.replace("Please fill in all required fields.",
                "A valid <b>Primary SOS number</b> is required (min 10 digits).");
    server.send(400, "text/html", err);
    return;
  }
  if (devID.length() == 0) {
    String err = String(ERROR_PAGE);
    err.replace("Please fill in all required fields.",
                "Device ID is required. It must match CircuitDigest portal.");
    server.send(400, "text/html", err);
    return;
  }

  // ── All good — save and start ─────────────────────────────────────────────
  apiKey = ak;
  apnStr = apn;
  sosNumbers[0] = s1;
  sosNumbers[1] = s2;
  sosNumbers[2] = s3;
  deviceID = devID;
  saveConfig(s1, s2, s3, ak, apn, interval, devID);
  geolinkerUploadInterval = interval;


  Serial.println("✓ Portal saved — API key, APN, SOS numbers");

  // Send success page first so browser gets it before we block on GSM init
  server.send_P(200, "text/html", SUCCESS_PAGE);

  // Give the browser 5 seconds to render the page, then shut down portal
  delay(5000);
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  digitalWrite(configLED, LOW);
  Serial.println("WiFi OFF — starting GSM/GPRS...");

  // GSM + GPRS init happens AFTER portal closes with valid credentials
  resetSIM800_AT();
  initializeGSM();
  sendStartupSMS();
}

// ─────────────────────────────────────────────────────────────────────────────
// Config portal
// ─────────────────────────────────────────────────────────────────────────────
void startConfigPortal() {
  Serial.println("\n=== Config Portal ===");
  for (int i = 0; i < 6; i++) {
    digitalWrite(configLED, HIGH);
    delay(100);
    digitalWrite(configLED, LOW);
    delay(100);
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP("GPS_Tracker_Setup", "gps123456");
  IPAddress IP = WiFi.softAPIP();
  Serial.println("SSID: GPS_Tracker_Setup  Pass: gps123456");
  Serial.println("URL : http://" + IP.toString());
  Serial.println("Configure: API Key + APN + SOS numbers");
  digitalWrite(configLED, HIGH);
  server.on("/", handleRoot);
  server.on("/save", handleSave);
  server.begin();
}

// ─────────────────────────────────────────────────────────────────────────────
// setup / loop
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== GPS Tracker ===");

  // 1. Start SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("✗ SPIFFS failed to mount");
    return;
  }

  // 2. CLEAR THE FLAG immediately on every boot
  // This ensures sendStartupSMS() always runs later in setup
  if (SPIFFS.exists(SMS_FLAG_FILE)) {
    SPIFFS.remove(SMS_FLAG_FILE);
    Serial.println("✓ Resetting startup SMS flag for this session");
  }

  pinMode(gpsStatusLED, OUTPUT);
  pinMode(sosStatusLED, OUTPUT);
  pinMode(configLED, OUTPUT);
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);

  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GSMSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  Serial.println("GPS & GSM serial initialized");

  bool configOK = loadConfig();

  if (digitalRead(CONFIG_BUTTON_PIN) == LOW || !configOK) {
    if (digitalRead(CONFIG_BUTTON_PIN) == LOW) delay(2000);
    startConfigPortal();
    while (WiFi.getMode() == WIFI_AP) {
      server.handleClient();
      delay(10);
    }
  } else {
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi OFF — full config found");
    initializeGSM();

    // This will now execute every time because we deleted the file above
    sendStartupSMS();
  }

  Serial.println("\n=== Ready ===");
  Serial.println("CONFIG: Hold D1 3s | SOS: Hold D2 5s | Serial: 'help'");
}

void loop() {
  if (WiFi.getMode() == WIFI_AP) server.handleClient();

  readGPS();
  handleSerialCommands();
  handleSOSButton();
  handleConfigButton();

  if (configModeTriggered) {
    SPIFFS.remove(SMS_FLAG_FILE);
    configModeTriggered = false;
    startConfigPortal();
  }

  processSMS();

  if (millis() - lastGSMCheck > gsmCheckInterval) {
    checkGSMStatus();
    lastGSMCheck = millis();
  }

  if (gpsFixed && gprsConnected && (millis() - lastGeolinkerUpload > geolinkerUploadInterval)) {
    sendToGeolinkerViaGPRS();
    lastGeolinkerUpload = millis();
  }

  if (millis() - lastGPSPrint > gpsPrintInterval) {
    printGPSStatus();
    lastGPSPrint = millis();
  }

  delay(100);
}

// ─────────────────────────────────────────────────────────────────────────────
// Button handlers
// ─────────────────────────────────────────────────────────────────────────────
void handleSOSButton() {
  bool pressed = (digitalRead(SOS_BUTTON_PIN) == LOW);
  static bool lastState = HIGH;
  static unsigned long lastDebounce = 0;
  if (pressed != lastState) {
    lastDebounce = millis();
    lastState = pressed;
  }
  if ((millis() - lastDebounce) > 50) {
    if (pressed && !sosButtonPressed) {
      sosButtonPressed = true;
      sosButtonPressStart = millis();
      digitalWrite(sosStatusLED, HIGH);
    } else if (!pressed && sosButtonPressed) {
      sosButtonPressed = false;
      digitalWrite(sosStatusLED, LOW);
      if ((millis() - sosButtonPressStart) >= SOS_PRESS_DURATION) triggerSOS();
    }
  }
}

void handleConfigButton() {
  bool pressed = (digitalRead(CONFIG_BUTTON_PIN) == LOW);
  static bool lastState = HIGH;
  static unsigned long lastDebounce = 0;
  if (pressed != lastState) {
    lastDebounce = millis();
    lastState = pressed;
  }
  if ((millis() - lastDebounce) > 50) {
    if (pressed && !configButtonPressed) {
      configButtonPressed = true;
      configButtonPressStart = millis();
      digitalWrite(configLED, HIGH);
    } else if (!pressed && configButtonPressed) {
      configButtonPressed = false;
      digitalWrite(configLED, LOW);
      if ((millis() - configButtonPressStart) >= CONFIG_PRESS_DURATION)
        configModeTriggered = true;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// SOS
// ─────────────────────────────────────────────────────────────────────────────
void triggerSOS() {
  Serial.println("\n=== SOS EMERGENCY ===");
  digitalWrite(sosStatusLED, HIGH);

  String message;
  if (gpsFixed) {
    String url = "https://maps.google.com/?q=" + String(latestGPSData.latitude, 6) + "," + String(latestGPSData.longitude, 6);
    message = EMERGENCY_MESSAGE + url;
  } else {
    message = "SOS EMERGENCY! GPS unavailable!";
  }

  for (int i = 0; i < 3; i++) {
    if (sosNumbers[i].length() >= 10) {
      sendSMS(sosNumbers[i], message);
      delay(3000);
    }
  }
  if (sosNumbers[0].length() >= 10) makeEmergencyCall(sosNumbers[0]);

  delay(10000);
  digitalWrite(sosStatusLED, LOW);
}

// ─────────────────────────────────────────────────────────────────────────────
// GSM helpers  ← UNCHANGED FROM ORIGINAL
// ─────────────────────────────────────────────────────────────────────────────
void clearGSMBuffer() {
  delay(100);
  while (GSMSerial.available()) GSMSerial.read();
}

bool waitForGSMResponse(String expected, unsigned long timeout) {
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    if (GSMSerial.available()) {
      char c = GSMSerial.read();
      response += c;
      if (response.indexOf(expected) != -1) return true;
    }
    delay(10);
  }
  Serial.println("Timeout waiting for: " + expected);
  Serial.println("Got: " + response);
  return false;
}

void checkGSMStatus() {
  GSMSerial.println("AT+CREG?");
  delay(1000);
  String response = "";
  unsigned long t = millis() + 3000;
  while (millis() < t) {
    if (GSMSerial.available()) response += (char)GSMSerial.read();
  }
  gsmReady = (response.indexOf("+CREG: 0,1") != -1 || response.indexOf("+CREG: 0,5") != -1);
  Serial.println(gsmReady ? "✓ GSM registered on network"
                          : "✗ GSM not registered - check SIM and signal");
}

// ─────────────────────────────────────────────────────────────────────────────
// GSM init  ← UNCHANGED FROM ORIGINAL
// ─────────────────────────────────────────────────────────────────────────────
void initializeGSM() {
  Serial.println("Init GSM...");
  while (GSMSerial.available()) GSMSerial.read();

  GSMSerial.println("AT");
  delay(1000);
  if (!waitForGSMResponse("OK", 5000)) {
    Serial.println("GSM no response - check connections and power");
    return;
  }
  Serial.println("✓ GSM responding");

  GSMSerial.println("AT+CFUN=1");
  delay(2000);
  GSMSerial.println("AT+CMGF=1");
  delay(1000);  // SMS text mode
  GSMSerial.println("AT+CNMI=1,2,0,0,0");
  delay(1000);  // SMS to serial
  GSMSerial.println("AT+CLIP=1");
  delay(1000);  // Caller ID

  // Audio config for call answering
  GSMSerial.println("AT+CECM=1");
  delay(500);
  GSMSerial.println("AT+CMIC=0," + String(MIC_GAIN));
  delay(500);

  initializeGPRS();
  checkGSMStatus();

  Serial.println("✓ GSM ready");
  Serial.println("  Auto-answer: " + String(autoAnswerAll ? "ALL calls" : "SOS numbers only"));
}

// ─────────────────────────────────────────────────────────────────────────────
// GPRS init  ← UNCHANGED FROM ORIGINAL (now uses runtime apnStr variable)
// ─────────────────────────────────────────────────────────────────────────────
// GPRS init - FIXED for Registration & Reset
// ─────────────────────────────────────────────────────────────────────────────
void initializeGPRS() {
  Serial.println("--- Initializing GPRS Bearer ---");

  // Wait for registration before trying GPRS
  unsigned long regTimeout = millis() + 15000;
  bool registered = false;
  while (millis() < regTimeout) {
    checkGSMStatus();
    if (gsmReady) {
      registered = true;
      break;
    }
    delay(2000);
    Serial.println("Waiting for network registration...");
  }

  if (!registered) {
    Serial.println("✗ GPRS aborted: Module not registered on network.");
    return;
  }

  Serial.println("   APN: " + apnStr);

  // FORCE RESET: Terminate any existing HTTP or Bearer sessions
  GSMSerial.println("AT+HTTPTERM");
  delay(500);
  GSMSerial.println("AT+SAPBR=0,1");
  delay(1000);  // Close bearer if open

  clearGSMBuffer();

  // 1. Set Connection Type to GPRS
  GSMSerial.println("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  if (!waitForGSMResponse("OK", 5000)) {
    Serial.println("✗ Failed to set CONTYPE (Module might be busy)");
    gprsConnected = false;
    return;
  }

  // 2. Set APN
  String apnCmd = "AT+SAPBR=3,1,\"APN\",\"" + apnStr + "\"";
  GSMSerial.println(apnCmd);
  if (!waitForGSMResponse("OK", 5000)) {
    Serial.println("✗ Failed to set APN");
    gprsConnected = false;
    return;
  }

  // 3. Open the Bearer
  Serial.println("Opening Bearer...");
  GSMSerial.println("AT+SAPBR=1,1");
  if (!waitForGSMResponse("OK", 30000)) {
    Serial.println("✗ Bearer Open Failed (Check Signal/Data Balance)");
    gprsConnected = false;
    return;
  }

  // 4. Verify IP Address
  GSMSerial.println("AT+SAPBR=2,1");
  delay(1000);
  String ipRes = "";
  while (GSMSerial.available()) ipRes += (char)GSMSerial.read();

  if (ipRes.indexOf(".") != -1 && ipRes.indexOf("0.0.0.0") == -1) {
    gprsConnected = true;
    Serial.println("✓ GPRS Bearer Active!");
  } else {
    gprsConnected = false;
    Serial.println("✗ Invalid IP received");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// SMS / Call  ← UNCHANGED FROM ORIGINAL
// ─────────────────────────────────────────────────────────────────────────────
void sendSMS(String phone, String message) {
  if (!gsmReady) {
    Serial.println("Cannot send SMS - GSM not ready");
    return;
  }
  Serial.println("SMS → " + phone);
  GSMSerial.println("AT+CMGS=\"" + phone + "\"");
  delay(1000);
  GSMSerial.print(message);
  delay(500);
  GSMSerial.write(26);  // Ctrl+Z
  delay(5000);
  Serial.println("✓ SMS sent");
}

void makeEmergencyCall(String phone) {
  if (!gsmReady) {
    Serial.println("Cannot call - GSM not ready");
    return;
  }
  Serial.println("Calling: " + phone);
  GSMSerial.println("ATD" + phone + ";");
  delay(30000);
  GSMSerial.println("ATH");
  Serial.println("✓ Call ended");
}

void sendStartupSMS() {
  if (!gsmReady || isStartupSMSSent()) return;
  Serial.println("Sending startup SMS...");
  for (int i = 0; i < 3; i++) {
    if (sosNumbers[i].length() >= 10) {
      sendSMS(sosNumbers[i], "GPS Tracker online. Send 'SEND LOCATION' for position.");
      delay(3000);
    }
  }
  markStartupSMSSent();
}

// ─────────────────────────────────────────────────────────────────────────────
// Incoming call / SMS processing  ← UNCHANGED FROM ORIGINAL
// ─────────────────────────────────────────────────────────────────────────────
void processSMS() {
  static String currentLine = "";
  static String smsFull = "";

  while (GSMSerial.available()) {
    char c = GSMSerial.read();
    if (c == '\n') {
      currentLine.trim();

      if (currentLine.indexOf("RING") != -1 && !callInProgress) {
        Serial.println("Incoming call...");
        incomingCallerID = "Unknown";
      } else if (currentLine.startsWith("+CLIP:")) {
        int q1 = currentLine.indexOf('"');
        int q2 = currentLine.indexOf('"', q1 + 1);
        if (q1 != -1 && q2 != -1) incomingCallerID = currentLine.substring(q1 + 1, q2);
        Serial.println("Caller: " + incomingCallerID);
        handleIncomingCall(incomingCallerID);
      } else if (currentLine.indexOf("NO CARRIER") != -1 || currentLine.indexOf("BUSY") != -1) {
        if (callInProgress) {
          callInProgress = false;
          callAnsweredAt = 0;
          incomingCallerID = "";
          digitalWrite(sosStatusLED, LOW);
          Serial.println("Call ended by remote");
        }
      } else if (currentLine.startsWith("+CMT:")) {
        // +CMT: header line arrived — save it and wait for the body on next line
        smsFull = currentLine + "\n";
        Serial.println("SMS header captured: " + currentLine);
      } else if (smsFull.length() > 0) {
        // This is the SMS body line that follows the +CMT: header
        // Skip blank lines (pure \r artifacts) and keep waiting
        if (currentLine.length() < 2) {
          // blank — do nothing, keep smsFull so next non-blank line is the body
        } else {
          smsFull += currentLine + "\n";
          Serial.println("SMS body captured: " + currentLine);
          processIncomingSMS(smsFull);
          smsFull = "";
        }
      }

      currentLine = "";
    } else {
      currentLine += c;
    }
  }

  // Auto-hangup
  if (callInProgress && MAX_CALL_DURATION > 0 && (millis() - callAnsweredAt) >= (MAX_CALL_DURATION * 1000UL)) {
    Serial.println("Max duration – hanging up");
    hangupCall();
  }
}

void handleIncomingCall(String caller) {
  if (callInProgress) return;
  bool answer = false;
  if (autoAnswerAll) {
    answer = true;
    Serial.println("Auto-answer: ALL");
  } else {
    for (int i = 0; i < 3; i++) {
      if (sosNumbers[i].length() >= 10 && caller.endsWith(sosNumbers[i].substring(sosNumbers[i].length() - 10))) {
        answer = true;
        Serial.println("Caller matches SOS #" + String(i + 1));
        break;
      }
    }
    if (!answer) Serial.println("Caller not in SOS list – ignoring");
  }
  if (answer) answerCall();
}

void answerCall() {
  Serial.println("Answering...");
  GSMSerial.println("ATA");
  delay(1000);
  callInProgress = true;
  callAnsweredAt = millis();
  digitalWrite(sosStatusLED, HIGH);
  GSMSerial.println("AT+CHFA=0");
  delay(300);
  GSMSerial.println("AT+CMIC=0," + String(MIC_GAIN));
  delay(300);
  GSMSerial.println("AT+CLVL=70");
  delay(300);
  GSMSerial.println("AT+CSIDET=1");
  delay(300);
  Serial.println("🎙 Audio active | caller: " + incomingCallerID + " | auto-end in " + String(MAX_CALL_DURATION) + "s");
}

void hangupCall() {
  GSMSerial.println("ATH");
  delay(500);
  callInProgress = false;
  callAnsweredAt = 0;
  incomingCallerID = "";
  digitalWrite(sosStatusLED, LOW);
  Serial.println("Hung up");
}

// ─────────────────────────────────────────────────────────────────────────────
// Phone number normaliser
// Strips non-digit chars and returns the last 10 digits.
// This makes +919876543210, 919876543210, 09876543210 all match 9876543210.
// ─────────────────────────────────────────────────────────────────────────────
String normalizePhone(String num) {
  String digits = "";
  for (int i = 0; i < (int)num.length(); i++) {
    if (isdigit(num[i])) digits += num[i];
  }
  if (digits.length() > 10) digits = digits.substring(digits.length() - 10);
  return digits;
}

bool phonesMatch(String a, String b) {
  return normalizePhone(a) == normalizePhone(b);
}

void processIncomingSMS(String full) {
  int end1 = full.indexOf('\n');
  if (end1 == -1) return;
  String header = full.substring(0, end1);
  String msg = full.substring(end1 + 1);
  msg.trim();
  msg.toUpperCase();

  int q1 = header.indexOf('"'), q2 = header.indexOf('"', q1 + 1);
  if (q1 == -1 || q2 == -1) return;
  String from = header.substring(q1 + 1, q2);

  Serial.println("SMS from: " + from + " | msg: " + msg);

  if (msg.indexOf("SEND LOCATION") != -1) {

    // Check if sender is one of the saved SOS numbers (format-agnostic)
    bool ok = false;
    for (int i = 0; i < 3; i++) {
      if (sosNumbers[i].length() >= 10 && phonesMatch(from, sosNumbers[i])) {
        ok = true;
        Serial.println("✓ Sender matched SOS #" + String(i + 1));
        break;
      }
    }

    if (!ok) {
      // Sender not in list — log and ignore (no reply to unknown numbers)
      Serial.println("✗ Sender not in SOS list — ignoring");
      return;
    }

    if (gpsFixed) {
      String url = "https://maps.google.com/?q=" + String(latestGPSData.latitude, 6) + "," + String(latestGPSData.longitude, 6);
      Serial.println("Replying with location to: " + from);
      sendSMS(from, "Location: " + url + " | Speed: " + String(latestGPSData.speed, 1) + "km/h" + " | Sats: " + String(latestGPSData.satellites) + " | Time: " + latestGPSData.timestamp);
    } else {
      Serial.println("GPS not fixed — sending no-fix reply to: " + from);
      sendSMS(from, "GPS Tracker: No GPS fix yet. " + String(latestGPSData.satellites) + " satellites visible. Try again shortly.");
    }
  } else {
    // Unknown command — send a helpful reply so user knows the device is alive
    Serial.println("Unknown SMS command from SOS number — sending help reply");
    bool knownSender = false;
    for (int i = 0; i < 3; i++) {
      if (sosNumbers[i].length() >= 10 && phonesMatch(from, sosNumbers[i])) {
        knownSender = true;
        break;
      }
    }
    if (knownSender) {
      sendSMS(from, "GPS Tracker ready. Commands: SEND LOCATION");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// GPS  ← UNCHANGED FROM ORIGINAL
// ─────────────────────────────────────────────────────────────────────────────
void readGPS() {
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      if (gps.location.isUpdated()) {
        latestGPSData.latitude = gps.location.lat();
        latestGPSData.longitude = gps.location.lng();
        latestGPSData.satellites = gps.satellites.value();
        latestGPSData.altitude = gps.altitude.meters();
        latestGPSData.speed = gps.speed.kmph();

        if (gps.date.isValid() && gps.time.isValid()) {
          int hour = gps.time.hour() + 5;
          int minute = gps.time.minute() + 30;
          if (minute >= 60) {
            minute -= 60;
            hour++;
          }
          if (hour >= 24) { hour -= 24; }
          char buf[20];
          snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                   gps.date.year(), gps.date.month(), gps.date.day(),
                   hour, minute, gps.time.second());
          latestGPSData.timestamp = String(buf);
        } else {
          latestGPSData.timestamp = "2025-02-10 12:00:00";
        }

        bool was = gpsFixed;
        gpsFixed = gps.location.isValid() && (latestGPSData.satellites >= 4) && (abs(latestGPSData.latitude) > 0.001) && (abs(latestGPSData.longitude) > 0.001);

        if (gpsFixed != was) {
          digitalWrite(gpsStatusLED, gpsFixed ? HIGH : LOW);
          if (gpsFixed) {
            Serial.println("✓ GPS FIX");
            Serial.println("  Lat: " + String(latestGPSData.latitude, 6));
            Serial.println("  Lng: " + String(latestGPSData.longitude, 6));
          }
        }
      }
    }
  }
}

void printGPSStatus() {
  Serial.print("GPS: ");
  Serial.print(gpsFixed ? "FIXED" : "NO FIX");
  if (gpsFixed) {
    Serial.print(" (");
    Serial.print(latestGPSData.satellites);
    Serial.print(" sats)");
  }
  Serial.print(" | GSM: ");
  Serial.print(gsmReady ? "Ready" : "Not Ready");
  Serial.print(" | GPRS: ");
  Serial.println(gprsConnected ? "Connected" : "Disconnected");
}


// ─────────────────────────────────────────────────────────────────────────────
// Geolinker Upload - FIXED to use Portal API Key
// ─────────────────────────────────────────────────────────────────────────────
void sendToGeolinkerViaGPRS() {
  if (!gpsFixed || !gprsConnected) return;

  Serial.println("\n=== Geolinker Upload ===");

  GSMSerial.println("AT+HTTPTERM");
  delay(300);
  GSMSerial.println("AT+HTTPINIT");
  if (!waitForGSMResponse("OK", 2000)) return;

  GSMSerial.println("AT+HTTPPARA=\"CID\",1");
  delay(100);

  // Note: Geolinker Host is currently set to www.circuitdigest.cloud
  String url = "http://" + String(geolinkerHost) + String(geolinkerPath);
  GSMSerial.println("AT+HTTPPARA=\"URL\",\"" + url + "\"");
  delay(100);

  GSMSerial.println("AT+HTTPPARA=\"CONTENT\",\"application/json\"");
  delay(100);

  // IMPORTANT: Using the apiKey variable from the portal
  GSMSerial.println("AT+HTTPPARA=\"USERDATA\",\"Authorization: " + apiKey + "\"");
  delay(100);

  // Payload
  String payload = "{\"device_id\":\"" + String(deviceID) + "\",\"timestamp\":[\"" + latestGPSData.timestamp + "\"],\"lat\":[" + String(latestGPSData.latitude, 6) + "],\"long\":[" + String(latestGPSData.longitude, 6) + "]}";

  GSMSerial.println("AT+HTTPDATA=" + String(payload.length()) + ",10000");
  if (waitForGSMResponse("DOWNLOAD", 5000)) {
    GSMSerial.print(payload);
    waitForGSMResponse("OK", 5000);

    Serial.println("Executing POST...");
    GSMSerial.println("AT+HTTPACTION=1");

    // Wait for response
    unsigned long timeout = millis() + 30000;
    while (millis() < timeout) {
      if (GSMSerial.available()) {
        String line = GSMSerial.readStringUntil('\n');
        if (line.indexOf("+HTTPACTION:") != -1) {
          Serial.println("Result: " + line);
          if (line.indexOf(",200,") != -1) Serial.println("✓✓✓ DATA UPLOAD SUCCESS!");
          break;
        }
      }
    }
  }
  GSMSerial.println("AT+HTTPTERM");
}

void resetSIM800_AT() {
  Serial.println("Resetting SIM800L via AT...");

  GSMSerial.println("AT+CFUN=1,1");  // full reset
  delay(8000);                       // IMPORTANT: wait for reboot

  while (GSMSerial.available()) GSMSerial.read();

  Serial.println("✓ SIM800L rebooted via AT");
}

// ─────────────────────────────────────────────────────────────────────────────
// Serial commands
// ─────────────────────────────────────────────────────────────────────────────
void handleSerialCommands() {
  if (!Serial.available()) return;
  String cmd = Serial.readString();
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "status") {
    Serial.println("\n=== Status ===");
    Serial.println("GPS:     " + String(gpsFixed ? "Fixed" : "No Fix"));
    Serial.println("GSM:     " + String(gsmReady ? "Ready" : "Not Ready"));
    Serial.println("GPRS:    " + String(gprsConnected ? "Connected" : "Disconnected"));
    Serial.println("APN:     " + apnStr);
    Serial.println("API Key: " + apiKey);
    Serial.println("Call:    " + String(callInProgress ? "In Progress (" + incomingCallerID + ")" : "Idle"));
    Serial.println("Auto-answer: " + String(autoAnswerAll ? "ALL" : "SOS only"));
    if (gpsFixed) {
      Serial.println("Lat:  " + String(latestGPSData.latitude, 6));
      Serial.println("Lng:  " + String(latestGPSData.longitude, 6));
      Serial.println("Sats: " + String(latestGPSData.satellites));
      Serial.println("Time: " + latestGPSData.timestamp);
    }
    Serial.println("SOS numbers:");
    for (int i = 0; i < 3; i++) Serial.println("  " + String(i + 1) + ": " + sosNumbers[i]);
  } else if (cmd == "send now") {
    sendToGeolinkerViaGPRS();
  } else if (cmd == "reset gprs") {
    Serial.println("Resetting GPRS...");
    initializeGPRS();
  } else if (cmd == "test sms") {
    if (sosNumbers[0].length() >= 10) sendSMS(sosNumbers[0], "Test SMS from GPS Tracker");
    else Serial.println("No SOS number configured");
  } else if (cmd == "test gsm") {
    GSMSerial.println("AT");
    delay(1000);
    while (GSMSerial.available()) Serial.write(GSMSerial.read());
  } else if (cmd == "signal") {
    GSMSerial.println("AT+CSQ");
    delay(1000);
    while (GSMSerial.available()) Serial.write(GSMSerial.read());
    Serial.println("\n0-9=marginal 10-14=ok 15-19=good 20-30=excellent 99=no signal");
  } else if (cmd == "network") {
    GSMSerial.println("AT+CREG?");
    delay(1000);
    while (GSMSerial.available()) Serial.write(GSMSerial.read());
    Serial.println("\n+CREG: 0,1 or 0,5 = registered");
  } else if (cmd == "hangup") {
    if (callInProgress) hangupCall();
    else Serial.println("No active call");
  } else if (cmd == "call mode") {
    autoAnswerAll = !autoAnswerAll;
    Serial.println("Auto-answer → " + String(autoAnswerAll ? "ALL" : "SOS only"));
  } else if (cmd == "vol up") {
    GSMSerial.println("AT+CLVL=90");
    delay(300);
    Serial.println("Vol 90");
  } else if (cmd == "vol down") {
    GSMSerial.println("AT+CLVL=50");
    delay(300);
    Serial.println("Vol 50");
  } else if (cmd == "mute") {
    GSMSerial.println("AT+CMUT=1");
    delay(300);
    Serial.println("Muted");
  } else if (cmd == "unmute") {
    GSMSerial.println("AT+CMUT=0");
    delay(300);
    Serial.println("Unmuted");
  } else if (cmd == "help") {
    Serial.println("\n=== Commands ===");
    Serial.println("status      - Full system status (APN, API key, GPS, GSM)");
    Serial.println("send now    - Force Geolinker upload");
    Serial.println("reset gprs  - Reconnect GPRS bearer");
    Serial.println("test sms    - Send test SMS to SOS#1");
    Serial.println("test gsm    - AT ping test");
    Serial.println("signal      - GSM signal strength");
    Serial.println("network     - Network registration status");
    Serial.println("hangup      - End active call");
    Serial.println("call mode   - Toggle ALL / SOS-only auto-answer");
    Serial.println("vol up/down - Speaker volume");
    Serial.println("mute/unmute - Microphone");
    Serial.println("");
    Serial.println("To reconfigure: hold D1 for 3s");
    Serial.println("  WiFi: GPS_Tracker_Setup / gps123456");
    Serial.println("  URL : http://192.168.4.1");
  } else {
    Serial.println("Unknown command. Type 'help'.");
  }
}
