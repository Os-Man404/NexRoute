/*
 * NexRoute - ESP32 4WD LINE FOLLOWER (line following only, ONE control: speed)
 * -------------------------------------------------------------------------
 * Board in Arduino IDE : ESP32 Dev Module   (38-pin ESP32, 240 MHz dual core)
 * Serial Monitor       : 115200 baud
 * Requires ESP32 Arduino core 3.x
 *
 * Motor driver: L298N with INA / INB / INC / IND (no ENA/ENB)
 *   INA(22) + INB(23) -> Left  side motors
 *   INC(18) + IND(19) -> Right side motors
 * IR array (8 channel, left -> right):
 *   S1=13  S2=15  S3=14  S4=27  S5=26  S6=16  S7=33  S8=32
 * WiFi AP: LineBot_ESP32 / 12345678  ->  http://192.168.4.1
 * Serial commands:  g = start line following,  s = stop
 *
 * Only the SPEED is set by you (web slider). Steering strength, turn speed
 * and everything else are fixed or calculated from the speed automatically.
 *
 * First-time checks:
 *   MANUAL > LEFT arrow  -> robot must turn LEFT   (else SWAP_LEFT_RIGHT = true)
 *   Black under LEFT-most sensor -> first "IR:" number = 1, err negative
 *                                                  (else REVERSE_SENSOR_ORDER = true)
 */

#if defined(ARDUINO_ESP32_WROOM_DA)
#error "Wrong board selected. Use Tools > Board > esp32 > ESP32 Dev Module."
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

/* ===================== THE ONLY THINGS YOU SET ===================== */
int baseSpeed   = 150;               // line following speed (web slider)
int manualSpeed = 180;               // manual mode speed (web slider)

/* ===================== WIRING FLAGS (set once) ===================== */
bool SWAP_LEFT_RIGHT      = false;   // left arrow e robot dane ghurle true
bool INVERT_LEFT          = false;   // left side ulta dike ghurle true
bool INVERT_RIGHT         = false;   // right side ulta dike ghurle true
bool REVERSE_SENSOR_ORDER = false;   // sensor order ulta hole true
bool BLACK_LINE           = true;    // true = white floor e black line

/* ============ FIXED PARAMETERS (normally no need to touch) ============ */
const int   MIN_PWM   = 120;         // er niche motor ghore na; chaka atke thakle 140-150 koro
const int   DEADZONE  = 25;          // er cheye choto command = motor bondho
const unsigned long KICK_MS = 40;    // thama theke shuru korte eto ms full power
const int   KICK_PWM  = 255;

const float STEER_P = 0.16;          // steering strength per error unit (speed er sathe nije scale hoy)
const float STEER_D = 0.13;          // zigzag damping
const int   TURN_MIN   = 170;        // line khojar ghurnir minimum speed
const int   TURN_EXTRA = 30;         // ghurnir speed = baseSpeed + eta

const unsigned long LOST_STRAIGHT_MS = 250;   // line majhkhan theke harale eto ms shoja jabe
const unsigned long LOST_STOP_MS     = 3000;  // eto ms line na pele robot theme jabe

const unsigned long IR_LINE_CONFIRM_MS  = 8;  // noise filter
const unsigned long IR_CLEAR_CONFIRM_MS = 3;

const unsigned long SERIAL_STATUS_MS = 250;
/* ====================================================================== */

const char* AP_SSID = "LineBot_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

const int irPin[8] = {13, 15, 14, 27, 26, 16, 33, 32};
const float weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
int irVal[8];                        // filtered: 1 = line, 0 = no line (index 0 = left)
int irRawLast[8];
unsigned long irRawSince[8];
unsigned int irFlipCount = 0;

#define INA 22
#define INB 23
#define INC 18
#define IND 19
#define PWM_FREQ 1000                // low frequency = L298N e kom speed e beshi torque
#define PWM_RES  8

String mode = "STOP";                // LINE / MANUAL / STOP
String manualCmd = "S";

/* steering state */
const unsigned long CONTROL_US = 10000;   // control step = 10 ms
const float D_SCALE = 3.0;
unsigned long lastControlUs = 0;
float lastError = 0, dFilt = 0, lastTurn = 0;
int   lastSide = 1;                  // -1 = line last seen on the left, +1 = right
bool  lineLost = false;
unsigned long lostSince = 0;

/* debug */
int lastLeftCmd = 0, lastRightCmd = 0;
unsigned long lastSerialStatus = 0;
unsigned long loopCount = 0;

int turnSpeedNow() { return constrain(baseSpeed + TURN_EXTRA, TURN_MIN, 255); }

/* ============================ MOTORS ============================ */
void driveSide(int pinF, int pinB, int spd) {
  if (spd >= 0) { ledcWrite(pinF, spd); ledcWrite(pinB, 0); }
  else          { ledcWrite(pinF, 0);   ledcWrite(pinB, -spd); }
}

/* Turns a requested speed into a PWM value the motor can actually move with:
   tiny commands -> 0, anything below MIN_PWM is raised to MIN_PWM,
   and a short full-power kick when a side starts from rest or reverses. */
int           sideSign[2]      = {0, 0};      // 0 = left, 1 = right
unsigned long sideKickUntil[2] = {0, 0};

int shapeSpeed(int spd, int side) {
  spd = constrain(spd, -255, 255);
  int mag  = abs(spd);
  int sign = (spd > 0) - (spd < 0);
  if (mag < DEADZONE) { sideSign[side] = 0; return 0; }
  if (mag < MIN_PWM) mag = MIN_PWM;
  unsigned long now = millis();
  if (sign != sideSign[side]) { sideSign[side] = sign; sideKickUntil[side] = now + KICK_MS; }
  if (now < sideKickUntil[side] && mag < KICK_PWM) mag = KICK_PWM;
  return sign * mag;
}

void leftMotor(int spd) {
  spd = shapeSpeed(spd, 0);
  lastLeftCmd = spd;                           // status e asol PWM dekhay
  if (INVERT_LEFT) spd = -spd;
  if (SWAP_LEFT_RIGHT) driveSide(INC, IND, spd); else driveSide(INA, INB, spd);
}

void rightMotor(int spd) {
  spd = shapeSpeed(spd, 1);
  lastRightCmd = spd;
  if (INVERT_RIGHT) spd = -spd;
  if (SWAP_LEFT_RIGHT) driveSide(INA, INB, spd); else driveSide(INC, IND, spd);
}

void setMotors(int l, int r) { leftMotor(l); rightMotor(r); }
void driveStop() { setMotors(0, 0); }

/* ============================ SENSORS ============================ */
int readRaw(int i) {                              // i = 0 (left) .. 7 (right)
  int pin = irPin[REVERSE_SENSOR_ORDER ? 7 - i : i];
  int raw = digitalRead(pin);
  return BLACK_LINE ? (raw == LOW ? 1 : 0) : (raw == HIGH ? 1 : 0);
}

void updateIR() {
  unsigned long now = millis();
  for (int i = 0; i < 8; i++) {
    int v = readRaw(i);
    if (v != irRawLast[i]) {
      irRawLast[i] = v;
      irRawSince[i] = now;
      irFlipCount++;
    } else if (v != irVal[i]) {
      unsigned long need = v ? IR_LINE_CONFIRM_MS : IR_CLEAR_CONFIRM_MS;
      if (now - irRawSince[i] >= need) irVal[i] = v;
    }
  }
}

/* error: negative = line is on the left, positive = on the right, 0 = centred.
   Returns false when no sensor sees the line. */
bool computeError(float &error) {
  float sum = 0; int count = 0;
  for (int i = 0; i < 8; i++) if (irVal[i]) { sum += weight[i]; count++; }
  if (count == 0) return false;
  error = sum / count;
  return true;
}

/* ============================ LINE FOLLOW ============================ */
void resetSteering() {
  lastError = 0; dFilt = 0; lastTurn = 0; lineLost = false;
}

/* Steering is a number "turn" from -1 (hard left) to +1 (hard right).
   The outer wheel always runs at baseSpeed. The inner wheel goes from
   +baseSpeed (straight) through 0 down to -baseSpeed (spin on the spot).
   Because everything is a fraction of baseSpeed, it scales with speed by itself. */
void lineFollowStep() {                 // runs every 10 ms
  float error;
  unsigned long now = millis();

  if (!computeError(error)) {
    if (!lineLost) { lineLost = true; lostSince = now; }
    unsigned long lostFor = now - lostSince;

    if (lostFor > LOST_STOP_MS) {       // safety
      mode = "STOP";
      driveStop();
      Serial.println("[LINE] Line lost for too long. Robot stopped.");
      return;
    }
    int s = turnSpeedNow();
    if (fabs(lastError) < 1.5 && lostFor < LOST_STRAIGHT_MS) {
      setMotors(baseSpeed, baseSpeed);  // gap hote pare -> shoja jao
    } else if (lastSide < 0) {
      setMotors(-s, s);                 // line bame chilo -> bame ghuro
    } else {
      setMotors(s, -s);                 // line dane chilo -> dane ghuro
    }
    dFilt = 0;
    return;
  }

  if (lineLost) { lineLost = false; lastError = error; dFilt = 0; }
  if (error <= -1.0) lastSide = -1;
  else if (error >= 1.0) lastSide = 1;

  float dRaw = (error - lastError) * D_SCALE;
  dFilt += 0.25f * (dRaw - dFilt);
  lastError = error;

  float turn = STEER_P * error + STEER_D * dFilt;
  turn = constrain(turn, -1.0f, 1.0f);
  lastTurn = turn;

  int inner = (int)(baseSpeed * (1.0f - 2.0f * fabs(turn)));
  if (turn >= 0) setMotors(baseSpeed, inner);     // line dane -> dane bako (dan chaka dhire/ulta)
  else           setMotors(inner, baseSpeed);     // line bame -> bame bako
}

/* ============================ MANUAL ============================ */
void manualDrive() {
  int s = manualSpeed;
  int t = max(s, TURN_MIN);
  if      (manualCmd == "F") setMotors(s, s);
  else if (manualCmd == "B") setMotors(-s, -s);
  else if (manualCmd == "L") setMotors(-t, t);    // turn LEFT (same convention as line follow)
  else if (manualCmd == "R") setMotors(t, -t);    // turn RIGHT
  else driveStop();
}

/* ============================ SERIAL ============================ */
void setMode(String m) {
  if (m != "LINE" && m != "MANUAL") m = "STOP";
  mode = m;
  resetSteering();
  if (mode == "STOP") driveStop();
  Serial.print("[MODE] "); Serial.println(mode);
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'g' || c == 'G') setMode("LINE");
    else if (c == 's' || c == 'S') setMode("STOP");
  }
}

void printStatus() {
  unsigned long now = millis();
  unsigned long dt = now - lastSerialStatus;
  if (dt < SERIAL_STATUS_MS) return;
  lastSerialStatus = now;

  Serial.print("IR:");
  for (int i = 0; i < 8; i++) { Serial.print(irVal[i]); Serial.print(' '); }
  Serial.print("flips="); Serial.print(irFlipCount);
  irFlipCount = 0;

  Serial.print(" | mode="); Serial.print(mode);
  if (mode == "MANUAL") { Serial.print(" cmd="); Serial.print(manualCmd); }
  Serial.print(" speed="); Serial.print(baseSpeed);

  float e;
  Serial.print(" | err=");
  if (computeError(e)) Serial.print(e, 1); else Serial.print("NO-LINE");
  Serial.print(" turn="); Serial.print(lastTurn, 2);
  Serial.print(" L="); Serial.print(lastLeftCmd);
  Serial.print(" R="); Serial.print(lastRightCmd);

  Serial.print(" | loops/s="); Serial.println(loopCount * 1000UL / dt);
  loopCount = 0;
}

/* ============================ WEB PAGE ============================ */
const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>ESP32 LineBot</title>
<style>
 *{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
 body{margin:0;font-family:system-ui,Arial;background:#101418;color:#e8eef2}
 header{padding:14px;text-align:center;background:#161d24;font-weight:600;letter-spacing:1px}
 .tabs{display:flex}
 .tab{flex:1;padding:14px;text-align:center;background:#1b242d;cursor:pointer;font-weight:600}
 .tab.on{background:#0aa06e;color:#fff}
 .panel{display:none;padding:18px}
 .panel.on{display:block}
 button{border:0;border-radius:14px;font-size:17px;font-weight:700;color:#fff;padding:18px;width:100%}
 .go{background:#0aa06e}.stop{background:#d0342c}.dir{background:#2b6cb0;height:78px}
 .pad{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:340px;margin:auto}
 .sp{margin:18px 0}
 input[type=range]{width:100%}
 .sens{display:flex;gap:6px;justify-content:center;margin-top:16px}
 .dot{width:26px;height:26px;border-radius:6px;background:#2a3540}
 .dot.hit{background:#0aa06e}
 .st{text-align:center;margin-top:12px;font-size:14px;color:#8fa3b0}
</style></head><body>
<header>ESP32 4WD LINE BOT</header>
<div class="tabs">
  <div class="tab on" id="t1" onclick="tab(1)">LINE FOLLOW</div>
  <div class="tab" id="t2" onclick="tab(2)">MANUAL</div>
</div>
<div class="panel on" id="p1">
  <button class="go" onclick="send('/mode?m=LINE')">START LINE FOLLOWING</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/mode?m=STOP')">STOP</button>
  <div class="sp">Speed: <b id="bsv">150</b>
    <input type="range" min="120" max="255" value="150" oninput="bsv.innerText=this.value" onchange="send('/set?base='+this.value)">
  </div>
  <div class="sens" id="sens"></div>
  <div class="st" id="st">mode: --</div>
</div>
<div class="panel" id="p2">
  <div class="pad">
    <div></div>
    <button class="dir" onmousedown="mv('F')" ontouchstart="mv('F')" onmouseup="mv('S')" ontouchend="mv('S')">&#9650;</button>
    <div></div>
    <button class="dir" onmousedown="mv('L')" ontouchstart="mv('L')" onmouseup="mv('S')" ontouchend="mv('S')">&#9664;</button>
    <button class="stop" style="height:78px" onclick="mv('S')">&#9632;</button>
    <button class="dir" onmousedown="mv('R')" ontouchstart="mv('R')" onmouseup="mv('S')" ontouchend="mv('S')">&#9654;</button>
    <div></div>
    <button class="dir" onmousedown="mv('B')" ontouchstart="mv('B')" onmouseup="mv('S')" ontouchend="mv('S')">&#9660;</button>
    <div></div>
  </div>
  <div class="sp">Manual Speed: <b id="msv">180</b>
    <input type="range" min="120" max="255" value="180" oninput="msv.innerText=this.value" onchange="send('/set?man='+this.value)">
  </div>
</div>
<script>
let box=document.getElementById('sens');
for(let i=0;i<8;i++){let d=document.createElement('div');d.className='dot';d.id='d'+i;box.appendChild(d);}
function tab(n){
  t1.className='tab'+(n==1?' on':'');t2.className='tab'+(n==2?' on':'');
  p1.className='panel'+(n==1?' on':'');p2.className='panel'+(n==2?' on':'');
  send('/mode?m='+(n==1?'STOP':'MANUAL'));
}
function send(u){fetch(u);}
function mv(c){fetch('/manual?c='+c);}
document.addEventListener('keydown',e=>{const k={ArrowUp:'F',ArrowDown:'B',ArrowLeft:'L',ArrowRight:'R'}[e.key];if(k)mv(k);});
document.addEventListener('keyup',e=>{if(e.key.startsWith('Arrow'))mv('S');});
setInterval(()=>{fetch('/status').then(r=>r.json()).then(j=>{
  for(let i=0;i<8;i++)document.getElementById('d'+i).className='dot'+(j.s[i]?' hit':'');
  st.innerText='mode: '+j.mode+'   err: '+j.err+'   L: '+j.l+'  R: '+j.r;
});},250);
</script></body></html>
)HTML";

void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleMode() {
  setMode(server.arg("m"));
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  String c = server.arg("c");
  if (mode != "MANUAL") setMode("MANUAL");
  manualCmd = c;
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("base")) baseSpeed   = constrain(server.arg("base").toInt(), 0, 255);
  if (server.hasArg("man"))  manualSpeed = constrain(server.arg("man").toInt(), 0, 255);
  Serial.print("[SET] speed="); Serial.print(baseSpeed);
  Serial.print(" manual="); Serial.print(manualSpeed);
  Serial.print(" (turn speed="); Serial.print(turnSpeedNow()); Serial.println(")");
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  float e;
  String errStr = computeError(e) ? String(e, 1) : String("NO-LINE");
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 8; i++) { j += String(irVal[i]); if (i < 7) j += ","; }
  j += "],\"err\":\"" + errStr + "\",\"l\":" + String(lastLeftCmd) + ",\"r\":" + String(lastRightCmd) + "}";
  server.send(200, "application/json", j);
}

/* ============================ SETUP / LOOP ============================ */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("============== NEXROUTE LINE FOLLOWER ==============");
  Serial.print("Chip: "); Serial.print(ESP.getChipModel());
  Serial.print("  rev "); Serial.print(ESP.getChipRevision());
  Serial.print("  cores "); Serial.print(ESP.getChipCores());
  Serial.print("  "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.println("Status: IR (left->right, 1=line) flips | mode speed | err (-=left, +=right) turn L R | loops/s");
  Serial.println("Serial commands: g = start, s = stop");
  Serial.println("=====================================================");

  for (int i = 0; i < 8; i++) pinMode(irPin[i], INPUT);
  for (int i = 0; i < 8; i++) {
    irVal[i] = readRaw(i);
    irRawLast[i] = irVal[i];
    irRawSince[i] = millis();
  }

  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.begin();
}

void loop() {
  server.handleClient();
  handleSerial();
  updateIR();
  loopCount++;

  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_US) {
    lastControlUs = nowUs;
    if      (mode == "LINE")   lineFollowStep();
    else if (mode == "MANUAL") manualDrive();
    else                       driveStop();
  }

  printStatus();
}
