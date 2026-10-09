/*
  Giant Caliper CLOCK — Adafruit ESP32-S3 Feather (no screen)
  24 inches on the caliper = 24 hours.  1 inch (25.4 mm) = 1 hour.
  Shortly before midnight the jaw heads home, timed to press the switch at 00:00.
  Hardware: STEP 5, DIR 6, EN 9, switch 10, BOOT = stop, optional GPIO11 = home
  Web: http://caliper.local   Serial: 115200, "Newline"
       x stop | h home | m test midnight | demo | s switch | d debug | j5 / j-5 jog | ? help
  Board: Adafruit Feather ESP32-S3, USB CDC On Boot: Enabled. Library: FastAccelStepper
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <time.h>
#include <sys/time.h>
#include <FastAccelStepper.h>

// ======================= Wi-Fi =======================
#define WIFI_AP_MODE 0
const char *AP_SSID  = "GiantCaliper";
const char *AP_PASS  = "caliper123";
const char *STA_SSID = "VT PRO";
const char *STA_PASS = "Cool$h1t";

// ======================= time =======================
const char *TZ_INFO = "PST8PDT,M3.2.0,M11.1.0";   // Los Angeles, DST automatic
const char *NTP_1   = "pool.ntp.org";
const char *NTP_2   = "time.nist.gov";

// ======================= pins (Feather ESP32-S3) =======================
const uint8_t PIN_STEP     = 5;
const uint8_t PIN_DIR      = 6;
const uint8_t PIN_EN       = 9;
const uint8_t PIN_HOME_SW  = 10;
const uint8_t PIN_BTN_STOP = 0;   // BOOT button
const uint8_t PIN_BTN_HOME = 11;  // optional button to GND

// ===================== mechanical =====================
const float STEPS_PER_MM    = 39.9;
const float TRAVEL_MM       = 609.6;    // 24 inches = 24 hours
const float MM_PER_INCH     = 25.4;
const double SECONDS_PER_DAY = 86400.0;

// If the jaw drives INTO the start end instead of opening, change to false
const bool OPEN_DIR_IS_HIGH = true;

// ======================= speed =======================
const float CATCHUP_SPEED_MM_S = 25.0;  // big jumps: after power-on, daylight saving, demo
const float RETURN_SPEED_MM_S  = 25.0;  // midnight return
const float ACCEL_MM_S2        = 25.0;
const float HOME_FAST_MM_S     = 15.0;
const float HOME_SLOW_MM_S     = 4.0;
const float RETURN_MARGIN_S    = 3.0;   // extra seconds so it's home a bit before 00:00, never late

// ======================= demo =======================
const double DEMO_SPEED = 120.0;        // 120x: 24 hours in 12 minutes

// ===================== switch =====================
const int   SWITCH_PRESSED_LEVEL = LOW;
const float BACKOFF_MM        = 5.0;
const float HOME_TOLERANCE_MM = 3.0;
const float RELEASE_CHECK_MM  = 15.0;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = nullptr;
WebServer server(80);

bool homed         = false;
bool busy          = false;
bool stopRequested = false;
String serialLine  = "";
String ipText      = "";
String uiStatus    = "";
String uiMsg       = "";
float  lastNightErrorMm = 0;
bool   haveNightError   = false;

enum MoveResult { MR_OK, MR_STOPPED, MR_FAULT, MR_HIT_SWITCH };

const int CMD_STOP = 1, CMD_HOME = 2, CMD_MIDNIGHT = 3, CMD_DEMO_ON = 4, CMD_DEMO_OFF = 5;
const int QUEUE_SIZE = 8;
int queueBuf[QUEUE_SIZE];
int queueHead = 0, queueLen = 0;

bool enqueue(int v) {
  if (queueLen >= QUEUE_SIZE) return false;
  queueBuf[(queueHead + queueLen) % QUEUE_SIZE] = v;
  queueLen++;
  return true;
}
bool dequeue(int &v) {
  if (queueLen == 0) return false;
  v = queueBuf[queueHead];
  queueHead = (queueHead + 1) % QUEUE_SIZE;
  queueLen--;
  return true;
}
void clearQueue() { queueHead = 0; queueLen = 0; }

int32_t mmToSteps(float mm) { return (int32_t)lroundf(mm * STEPS_PER_MM); }
float   posMm()             { return stepper->getCurrentPosition() / STEPS_PER_MM; }

void setSpeedMm(float mmPerS) {
  stepper->setSpeedInHz((uint32_t)(mmPerS * STEPS_PER_MM));
  stepper->setAcceleration((int32_t)(ACCEL_MM_S2 * STEPS_PER_MM));
}

// ======================= time =======================
bool   demoMode       = false;
double demoStartSim   = 0;
unsigned long demoStartMs = 0;

bool timeSynced() { return time(nullptr) > 1700000000; }

double realSecOfDay() {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  time_t t = tv.tv_sec;
  struct tm lt;
  localtime_r(&t, &lt);
  return lt.tm_hour * 3600.0 + lt.tm_min * 60.0 + lt.tm_sec + tv.tv_usec / 1e6;
}

double secOfDay() {
  if (!demoMode) return realSecOfDay();
  double s = demoStartSim + (millis() - demoStartMs) / 1000.0 * DEMO_SPEED;
  return fmod(s, SECONDS_PER_DAY);
}

bool clockAvailable() { return demoMode || timeSynced(); }

String hhmmss(double s) {
  long t = (long)s;
  char buf[12];
  snprintf(buf, sizeof(buf), "%02ld:%02ld:%02ld", t / 3600, (t / 60) % 60, t % 60);
  return String(buf);
}

float returnLeadRealS() {
  return (TRAVEL_MM - BACKOFF_MM) / RETURN_SPEED_MM_S
       + RETURN_SPEED_MM_S / ACCEL_MM_S2
       + BACKOFF_MM / HOME_SLOW_MM_S
       + RETURN_MARGIN_S;
}

bool inReturnWindow(double s) {
  double lead = returnLeadRealS() * (demoMode ? DEMO_SPEED : 1.0);
  return s >= SECONDS_PER_DAY - lead;
}

float clockTargetMm(double s) {
  if (inReturnWindow(s)) return 0;
  float mm = (float)(s / SECONDS_PER_DAY * TRAVEL_MM);
  if (mm < 0) mm = 0;
  if (mm > TRAVEL_MM) mm = TRAVEL_MM;
  return mm;
}

// ---- switch debounce ----
const unsigned long SW_DEBOUNCE_MS = 15;
bool swState = false;
unsigned long swChangeStart = 0;
bool swChanging = false;
unsigned long swGlitches = 0;

bool rawPressed() { return digitalRead(PIN_HOME_SW) == SWITCH_PRESSED_LEVEL; }

void updateSwitch() {
  bool r = rawPressed();
  if (r == swState) {
    if (swChanging) swGlitches++;
    swChanging = false;
    return;
  }
  if (!swChanging) {
    swChanging = true;
    swChangeStart = millis();
  } else if (millis() - swChangeStart >= SW_DEBOUNCE_MS) {
    swState = r;
    swChanging = false;
  }
}

void initSwitch() {
  int yes = 0, total = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 30) {
    if (rawPressed()) yes++;
    total++;
    delay(1);
  }
  swState = (yes * 2 > total);
  swChanging = false;
}

bool homePressed() {
  updateSwitch();
  return swState;
}

bool checkStop() {
  server.handleClient();
  updateSwitch();
  if (digitalRead(PIN_BTN_STOP) == LOW) stopRequested = true;
  if (Serial.available()) {
    int c = Serial.peek();
    if (c == 'x' || c == 'X') {
      Serial.read();
      stopRequested = true;
    }
  }
  return stopRequested;
}

void setUi(const String &status, const String &msg = "") {
  uiStatus = status;
  uiMsg = msg;
  Serial.print("[");
  Serial.print(status);
  Serial.print("] ");
  Serial.println(msg);
}

void enterStopped(const String &why) {
  stepper->forceStop();
  delay(50);
  stepper->disableOutputs();
  homed = false;
  stopRequested = false;
  clearQueue();
  setUi("STOPPED", why + " | press Home");
}

bool waitMove() {
  while (stepper->isRunning()) {
    if (checkStop()) return false;
    delay(2);
  }
  return true;
}

MoveResult moveToMm(float targetMm, float speed, bool stopAtSwitch, float *hitAtMm = nullptr) {
  setSpeedMm(speed);
  bool startedAtHome = homePressed();
  bool seenRelease = !startedAtHome;
  stepper->moveTo(mmToSteps(targetMm));

  while (stepper->isRunning()) {
    if (checkStop()) return MR_STOPPED;
    bool pressed = homePressed();
    if (!pressed) seenRelease = true;

    if (stopAtSwitch && pressed) {
      float at = posMm();
      stepper->forceStopAndNewPosition(0);
      if (hitAtMm) *hitAtMm = at;
      return MR_HIT_SWITCH;
    }
    if (!seenRelease && posMm() > RELEASE_CHECK_MM) {
      stepper->forceStop();
      return MR_FAULT;
    }
    delay(2);
  }
  return MR_OK;
}

bool homeAxis() {
  busy = true;
  stopRequested = false;
  stepper->enableOutputs();
  setUi("HOMING...", "finding home switch");
  initSwitch();
  stepper->setCurrentPosition(0);
  Serial.print("  step 1: switch at start = ");
  Serial.println(homePressed() ? "PRESSED -> backing off" : "released -> moving toward home");

  if (homePressed()) {
    setSpeedMm(HOME_SLOW_MM_S);
    stepper->moveTo(mmToSteps(RELEASE_CHECK_MM));
    while (stepper->isRunning()) {
      if (checkStop()) { enterStopped("stopped while homing"); busy = false; return false; }
      if (!homePressed()) break;
      delay(2);
    }
    if (homePressed()) { enterStopped("switch never released: check DIR"); busy = false; return false; }
    stepper->moveTo(stepper->getCurrentPosition() + mmToSteps(BACKOFF_MM));
    if (!waitMove()) { enterStopped("stopped while homing"); busy = false; return false; }
  } else {
    setSpeedMm(HOME_FAST_MM_S);
    stepper->moveTo(-mmToSteps(TRAVEL_MM + 30));
    bool found = false;
    while (stepper->isRunning()) {
      if (checkStop()) { enterStopped("stopped while homing"); busy = false; return false; }
      if (homePressed()) { stepper->forceStop(); found = true; break; }
      delay(2);
    }
    if (!found) { enterStopped("home switch not found"); busy = false; return false; }
    delay(100);
    stepper->setCurrentPosition(0);
    setSpeedMm(HOME_SLOW_MM_S);
    stepper->moveTo(mmToSteps(BACKOFF_MM));
    if (!waitMove()) { enterStopped("stopped while homing"); busy = false; return false; }
    if (homePressed()) { enterStopped("switch never released: check DIR"); busy = false; return false; }
  }

  Serial.print("  step 2: switch released at pos ");
  Serial.print(posMm(), 1);
  Serial.println(" mm -> slowly coming back to switch");
  setSpeedMm(HOME_SLOW_MM_S);
  stepper->moveTo(stepper->getCurrentPosition() - mmToSteps(BACKOFF_MM * 5));
  while (stepper->isRunning()) {
    if (checkStop()) { enterStopped("stopped while homing"); busy = false; return false; }
    if (homePressed()) {
      Serial.println("  step 3: switch pressed again -> home = 00:00");
      stepper->forceStopAndNewPosition(0);
      homed = true;
      setUi("HOMED", clockAvailable() ? "moving to current time" : "waiting for internet time");
      busy = false;
      return true;
    }
    delay(2);
  }
  enterStopped("slow approach missed switch");
  busy = false;
  return false;
}

bool returnHome() {
  busy = true;
  setUi("MIDNIGHT", "returning to 00:00");
  float hitAt = 0;

  MoveResult r = moveToMm(BACKOFF_MM, RETURN_SPEED_MM_S, true, &hitAt);
  if (r == MR_STOPPED) { enterStopped("stopped while returning"); busy = false; return false; }

  if (r != MR_HIT_SWITCH) {
    r = moveToMm(-20.0, HOME_SLOW_MM_S, true, &hitAt);
    if (r == MR_STOPPED) { enterStopped("stopped while returning"); busy = false; return false; }
    if (r != MR_HIT_SWITCH) { enterStopped("home switch not reached"); busy = false; return false; }
  }

  lastNightErrorMm = hitAt;
  haveNightError = true;
  Serial.print("  re-zeroed on the switch, drift during the day: ");
  Serial.print(hitAt, 2);
  Serial.println(" mm");
  if (fabsf(hitAt) > HOME_TOLERANCE_MM) Serial.println("  warning: drift is large (missed steps?)");
  setUi("00:00", "home - waiting for midnight");
  busy = false;
  return true;
}

// ======================= the clock =======================
bool returnedThisWindow = false;
unsigned long lastTick = 0;
unsigned long lastLog  = 0;
bool waitingShown = false;

void clockTick() {
  if (!homed || busy) return;
  if (millis() - lastTick < 100) return;
  lastTick = millis();

  if (!clockAvailable()) {
    if (!waitingShown) { setUi("WAIT TIME", "waiting for internet time (NTP)"); waitingShown = true; }
    return;
  }
  waitingShown = false;

  double s = secOfDay();

  if (inReturnWindow(s)) {
    if (!returnedThisWindow) {
      returnedThisWindow = true;
      Serial.print("[CLOCK] ");
      Serial.print(hhmmss(s));
      Serial.println(" -> starting midnight return");
      if (!returnHome()) return;
    }
    uiStatus = "00:00";
    uiMsg = String(demoMode ? "DEMO " : "") + "home - clock face " + hhmmss(s);
    return;
  }
  returnedThisWindow = false;

  int32_t target = mmToSteps(clockTargetMm(s));
  if (target != stepper->targetPos()) {
    setSpeedMm(CATCHUP_SPEED_MM_S);
    stepper->moveTo(target);
  }

  uiStatus = hhmmss(s).substring(0, 5);
  uiMsg = String(demoMode ? "DEMO  " : "") + String(posMm(), 1) + " mm  /  " +
          String(posMm() / MM_PER_INCH, 2) + " in";

  if (millis() - lastLog > (demoMode ? 5000UL : 60000UL)) {
    lastLog = millis();
    Serial.print("[CLOCK] ");
    Serial.print(hhmmss(s));
    Serial.print("  ");
    Serial.print(posMm(), 2);
    Serial.println(" mm");
  }
}

void setDemo(bool on) {
  if (on == demoMode) return;
  if (on) {
    demoStartSim = timeSynced() ? realSecOfDay() : 0;
    demoStartMs = millis();
  }
  demoMode = on;
  returnedThisWindow = false;
  setUi(on ? "DEMO ON" : "DEMO OFF", on ? "24 h in 12 min" : "back to real time");
}

// ======================= web page =======================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Caliper Clock</title>
<style>
:root{--bg:#111;--fg:#eee;--mut:#888;--card:#1c1c1c;--acc:#2bd4c4;--red:#e5484d;--yel:#f5c542;--grn:#46c46b}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font-family:-apple-system,system-ui,sans-serif;padding:16px;max-width:480px;margin:auto}
h1{font-size:18px;margin:0 0 12px;color:var(--mut);font-weight:600}
.card{background:var(--card);border-radius:14px;padding:16px;margin-bottom:12px}
#state{font-size:72px;font-weight:800;font-variant-numeric:tabular-nums;line-height:1.1}
#msg{color:var(--mut);font-size:14px;margin-top:6px;min-height:1em}
.bar{height:10px;background:#262626;border-radius:5px;margin-top:14px;overflow:hidden}
.bar>div{height:100%;background:var(--acc);width:0}
.row{display:flex;gap:8px}
button{font-size:18px;font-weight:700;border:0;border-radius:12px;padding:14px;cursor:pointer;flex:1}
#stop{background:var(--red);color:#fff;width:100%;font-size:22px;padding:18px;margin-bottom:12px}
.sec{background:#333;color:var(--fg)}
#demo.on{background:var(--acc);color:#000}
</style></head><body>
<h1>CALIPER CLOCK</h1>
<div class="card"><div id="state">--:--</div><div id="msg">connecting…</div><div class="bar"><div id="fill"></div></div></div>
<button id="stop">STOP</button>
<div class="row"><button class="sec" id="home">Home</button><button class="sec" id="mid">Test midnight</button><button class="sec" id="demo">Demo</button></div>
<script>
const $=id=>document.getElementById(id);
async function get(u){try{const r=await fetch(u);return await r.json()}catch(e){return null}}
function show(s){if(!s)return;
 $('state').textContent=s.state;$('msg').textContent=s.msg;
 $('state').style.color=s.homed?(s.busy?'var(--yel)':'var(--fg)'):'var(--red)';
 $('fill').style.width=s.pct+'%';
 $('demo').className='sec'+(s.demo?' on':'');$('demo').textContent=s.demo?'Demo: ON':'Demo';}
$('stop').onclick=async()=>show(await get('/cmd?c=x'));
$('home').onclick=async()=>show(await get('/cmd?c=h'));
$('mid').onclick=async()=>show(await get('/cmd?c=m'));
$('demo').onclick=async()=>show(await get('/cmd?c=demo'));
setInterval(async()=>show(await get('/status')),500);
</script></body></html>
)rawliteral";

String jsonEscape(const String &s) {
  String o;
  for (unsigned int i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') o += '\\';
    o += c;
  }
  return o;
}

String statusJson() {
  float mm = homed ? posMm() : 0.0f;
  float pct = mm / TRAVEL_MM * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  String j = "{\"state\":\"" + jsonEscape(uiStatus) + "\"";
  j += ",\"msg\":\"" + jsonEscape(uiMsg) + "\"";
  j += ",\"homed\":" + String(homed ? "true" : "false");
  j += ",\"busy\":" + String(busy ? "true" : "false");
  j += ",\"synced\":" + String(timeSynced() ? "true" : "false");
  j += ",\"demo\":" + String(demoMode ? "true" : "false");
  j += ",\"time\":\"" + (clockAvailable() ? hhmmss(secOfDay()) : String("--:--:--")) + "\"";
  j += ",\"mm\":" + String(mm, 2);
  j += ",\"inch\":" + String(mm / MM_PER_INCH, 3);
  j += ",\"pct\":" + String(pct, 2);
  j += "}";
  return j;
}

void sendJson(int code) {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(code, "application/json", statusJson());
}

void handleRoot()   { server.send_P(200, "text/html", INDEX_HTML); }
void handleStatus() { sendJson(200); }

void handleCmd() {
  String c = server.arg("c");
  Serial.print("[WEB] cmd ");
  Serial.println(c);
  if (c == "x") { stopRequested = true; enqueue(CMD_STOP); }
  else if (c == "h") enqueue(CMD_HOME);
  else if (c == "m") enqueue(CMD_MIDNIGHT);
  else if (c == "demo") enqueue(demoMode ? CMD_DEMO_OFF : CMD_DEMO_ON);
  sendJson(200);
}

void setupWifi() {
  bool staOk = false;
#if !WIFI_AP_MODE
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(200);
  staOk = (WiFi.status() == WL_CONNECTED);
  if (staOk) ipText = "http://" + WiFi.localIP().toString();
#endif
  if (!staOk) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    ipText = String("AP ") + AP_SSID + " http://" + WiFi.softAPIP().toString() +
             "  (no internet -> no real time, demo mode still works)";
  }
  if (MDNS.begin("caliper")) {
    MDNS.addService("http", "tcp", 80);
    ipText += " caliper.local";
  }
  Serial.println(ipText);

  configTzTime(TZ_INFO, NTP_1, NTP_2);

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/cmd", handleCmd);
  server.begin();
}

// ======================= serial =======================
void printHelp() {
  Serial.println();
  Serial.println("=== Caliper Clock (Feather ESP32-S3) ===");
  Serial.println("x stop | h home | m test midnight | demo | s switch | d debug | j5 / j-5 jog | ? help");
  Serial.print("Midnight return starts ");
  Serial.print(returnLeadRealS(), 1);
  Serial.println(" s before 00:00");
  Serial.println(ipText);
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  String l = line;
  l.toLowerCase();
  if (l == "h")    { enqueue(CMD_HOME); return; }
  if (l == "x")    { enqueue(CMD_STOP); return; }
  if (l == "m")    { enqueue(CMD_MIDNIGHT); return; }
  if (l == "demo") { enqueue(demoMode ? CMD_DEMO_OFF : CMD_DEMO_ON); return; }
  if (l == "?")    { printHelp(); return; }
  if (l == "s") {
    int raw = digitalRead(PIN_HOME_SW);
    Serial.print("Switch raw = ");
    Serial.print(raw == HIGH ? "HIGH" : "LOW");
    Serial.print("  -> ");
    Serial.println(raw == SWITCH_PRESSED_LEVEL ? "PRESSED" : "released");
    return;
  }
  if (l == "d") {
    Serial.print("homed=");   Serial.print(homed ? "yes" : "no");
    Serial.print(" busy=");   Serial.print(busy ? "yes" : "no");
    Serial.print(" synced="); Serial.print(timeSynced() ? "yes" : "no");
    Serial.print(" demo=");   Serial.print(demoMode ? "yes" : "no");
    Serial.print(" time=");   Serial.print(clockAvailable() ? hhmmss(secOfDay()) : String("--"));
    Serial.print(" pos=");    Serial.print(posMm(), 2);
    Serial.print("mm switch="); Serial.print(homePressed() ? "PRESSED" : "released");
    if (haveNightError) { Serial.print(" lastDrift="); Serial.print(lastNightErrorMm, 2); Serial.print("mm"); }
    Serial.println();
    return;
  }
  if (l.startsWith("j")) {
    float mm = line.substring(1).toFloat();
    stepper->enableOutputs();
    setSpeedMm(HOME_SLOW_MM_S);
    stepper->move(mmToSteps(mm));
    while (stepper->isRunning()) {
      if (checkStop()) { enterStopped("stopped while jogging"); return; }
      delay(2);
    }
    Serial.print("Jogged ");
    Serial.print(mm, 1);
    Serial.println(" mm (the clock will move back to the right time if homed)");
    return;
  }
  Serial.println("Unknown input. Type ? for help.");
}

void readSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      String line = serialLine;
      serialLine = "";
      handleLine(line);
    } else if (serialLine.length() < 32) {
      serialLine += c;
    }
  }
}

void processQueue() {
  int item;
  if (!dequeue(item)) return;
  if (item == CMD_STOP)     { enterStopped("stopped by user"); return; }
  if (item == CMD_HOME)     { homeAxis(); return; }
  if (item == CMD_DEMO_ON)  { setDemo(true); return; }
  if (item == CMD_DEMO_OFF) { setDemo(false); return; }
  if (item == CMD_MIDNIGHT) {
    if (!homed) { Serial.println("Not homed. Press Home first."); return; }
    stepper->stopMove();
    while (stepper->isRunning()) delay(2);
    returnHome();
    return;
  }
}

// ======================= main =======================
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_HOME_SW, INPUT_PULLUP);
  pinMode(PIN_BTN_STOP, INPUT_PULLUP);
  pinMode(PIN_BTN_HOME, INPUT_PULLUP);

  engine.init();
  stepper = engine.stepperConnectToPin(PIN_STEP);
  if (!stepper) {
    Serial.println("Stepper init failed!");
    while (true) delay(1000);
  }
  stepper->setDirectionPin(PIN_DIR, OPEN_DIR_IS_HIGH);
  stepper->setEnablePin(PIN_EN, true);
  stepper->setAutoEnable(false);
  stepper->enableOutputs();

  setupWifi();
  printHelp();
  delay(500);
  homeAxis();
}

void loop() {
  server.handleClient();
  updateSwitch();

  if (digitalRead(PIN_BTN_HOME) == LOW) {
    delay(30);
    if (digitalRead(PIN_BTN_HOME) == LOW) {
      while (digitalRead(PIN_BTN_HOME) == LOW) delay(10);
      enqueue(CMD_HOME);
    }
  }
  if (digitalRead(PIN_BTN_STOP) == LOW) {
    delay(30);
    if (digitalRead(PIN_BTN_STOP) == LOW) {
      while (digitalRead(PIN_BTN_STOP) == LOW) delay(10);
      enterStopped("stopped by button");
    }
  }

  readSerial();
  processQueue();
  clockTick();
  delay(2);
}
