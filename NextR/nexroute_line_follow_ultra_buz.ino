/*
 * NexRoute - ESP32 4WD LINE FOLLOWER + ULTRASONIC OBSTACLE STOP + BUZZER
 * VERSION: v1.3   (2026-10-05)
 *
 * Version history:
 *   v1.0  Line following only (RFID, ultrasonic, delivery removed). Fast 100 Hz
 *         control loop, one left/right convention, slow-down on curves.
 *   v1.1  Fixed motors stalling: minimum PWM, start kick, separate turn speed,
 *         PWM frequency 5000 -> 1000 Hz.   (tested on the robot: works)
 *   v1.2  Experimental speed-only steering (separate file, not used here).
 *   v1.3  Built on v1.1 (line following unchanged). Added HC-SR04 ultrasonic
 *         obstacle stop and beep-beep buzzer. The echo is measured with an
 *         interrupt, so the control loop is NOT slowed down.
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
 * Ultrasonic HC-SR04: TRIG=4  ECHO=35      Buzzer: GPIO5
 * WiFi AP: LineBot_ESP32 / 12345678  ->  http://192.168.4.1
 *
 * Serial commands:  g = start line following,  s = stop
 *
 * ======================= FIRST-TIME SETUP (2 tests) =======================
 * TEST 1 (motor side): web page > MANUAL > press the LEFT arrow.
 *    Robot must turn LEFT. If it turns right -> set SWAP_LEFT_RIGHT = true.
 *    If a side runs backward when you press UP -> set INVERT_LEFT / INVERT_RIGHT.
 * TEST 2 (sensor order): hold something black under the LEFT-most sensor.
 *    Serial "IR:" FIRST number must become 1 and err must be negative.
 *    If the LAST number becomes 1 -> set REVERSE_SENSOR_ORDER = true.
 * ==========================================================================
 */

#if defined(ARDUINO_ESP32_WROOM_DA)
#error "Wrong board selected. Use Tools > Board > esp32 > ESP32 Dev Module."
#endif

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#define FW_NAME    "NexRoute Line Follower"
#define FW_VERSION "v1.3"

/* ============================ SETTINGS ============================ */
bool SWAP_LEFT_RIGHT      = false;   // TEST 1: left arrow e robot dane ghurle true
bool INVERT_LEFT          = false;   // left side ulta dike ghurle true
bool INVERT_RIGHT         = false;   // right side ulta dike ghurle true
bool REVERSE_SENSOR_ORDER = false;   // TEST 2: sensor order ulta hole true
bool BLACK_LINE           = true;    // true = white floor e black line

int   baseSpeed   = 150;             // straight line speed (0-255)
int   manualSpeed = 180;
int   minPWM      = 110;             // er niche motor ghore na (stall). Chaka na ghurle barao, beshi jhaki dile komao
int   turnSpeed   = 180;             // line khojar shomoy ek jaygay ghorar speed (pivot e beshi shokti lage)
const int DEADZONE = 25;             // er cheye choto command = motor bondho
const unsigned long KICK_MS = 40;    // thama obostha theke shuru korte eto ms full power (0 = bondho)
const int KICK_PWM = 255;
float Kp = 25.0;                     // line theke shore gele koto jore ferabe
float Kd = 20.0;                     // jhakuni / zigzag komay

const float CURVE_SLOWDOWN = 0.05;   // error proti speed koto kombe (0 = kombe na)
const int   MAX_REVERSE    = 150;   // sharp turn e vitorer chaka shorbocco koto ulta ghurbe

const unsigned long LOST_STRAIGHT_MS = 250;   // line majhkhan theke harale eto ms shoja jabe (gap par howar jonno)
const unsigned long LOST_STOP_MS     = 3000;  // eto ms line na pele robot theme jabe

const unsigned long IR_LINE_CONFIRM_MS  = 8;  // black confirm korte koto ms (noise filter)
const unsigned long IR_CLEAR_CONFIRM_MS = 3;  // white confirm korte koto ms

const unsigned long SERIAL_STATUS_MS = 250;   // status line koto ms por por

/* ---- Ultrasonic + buzzer ---- */
bool  OBSTACLE_STOP_ENABLED = true;           // false korle obstacle e thambe na (sensor test er jonno)
const float OBSTACLE_DISTANCE_CM = 15.0;      // eto cm ba tar kache kichu thakle robot thambe
const int   OBSTACLE_CLEAR_READS = 3;         // por por eto bar "clear" pele abar cholbe
const unsigned long US_READ_INTERVAL = 60;    // koto ms por por distance mapbe
const unsigned long US_ECHO_TIMEOUT  = 40;    // eto ms e echo na ele "kichu nei"
const unsigned long BEEP_REPEAT_MS   = 10000; // obstacle thakle eto ms por por abar beep-beep
const unsigned long BEEP_ON_MS  = 120;
const unsigned long BEEP_OFF_MS = 120;
/* ================================================================== */

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

#define TRIG_PIN   4
#define ECHO_PIN   35
#define BUZZER_PIN 5

String mode = "STOP";                // LINE / MANUAL / STOP
String manualCmd = "S";

/* PID state */
const unsigned long CONTROL_US = 10000;   // control step = 10 ms (100 times per second)
const float D_SCALE = 3.0;                // keeps Kd on the same scale as the old (slow-loop) code
unsigned long lastControlUs = 0;
float lastError = 0, dFilt = 0;
int   lastSide = 1;                  // -1 = line was last seen on the left, +1 = right
bool  lineLost = false;
unsigned long lostSince = 0;

/* debug */
int lastLeftCmd = 0, lastRightCmd = 0;
unsigned long lastSerialStatus = 0;
unsigned long loopCount = 0;

/* ============================ MOTORS ============================ */
void driveSide(int pinF, int pinB, int spd) {
  if (spd >= 0) { ledcWrite(pinF, spd); ledcWrite(pinB, 0); }
  else          { ledcWrite(pinF, 0);   ledcWrite(pinB, -spd); }
}

/* Turns a requested speed into a PWM value the motor can actually move with:
   - tiny commands -> 0
   - anything below minPWM is raised to minPWM (otherwise the motor just hums and stalls)
   - a short full-power kick when a side starts from rest or changes direction */
int           sideSign[2]      = {0, 0};      // 0 = left, 1 = right
unsigned long sideKickUntil[2] = {0, 0};

int shapeSpeed(int spd, int side) {
  spd = constrain(spd, -255, 255);
  int mag  = abs(spd);
  int sign = (spd > 0) - (spd < 0);
  if (mag < DEADZONE) { sideSign[side] = 0; return 0; }
  if (mag < minPWM) mag = minPWM;
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

/* Reads all sensors. A value is accepted only after it stays the same for a few ms. */
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

/* ============================ ULTRASONIC ============================
   Non-blocking: a trigger pulse is sent every US_READ_INTERVAL ms and the
   echo length is timed by an interrupt, so loop() never waits for the echo. */
volatile uint32_t echoRiseUs = 0;
volatile uint32_t echoDurUs  = 0;
volatile bool     echoDone   = false;

void IRAM_ATTR echoISR() {
  uint32_t t = micros();
  if (digitalRead(ECHO_PIN)) {
    echoRiseUs = t;
  } else if (echoRiseUs != 0) {
    echoDurUs = t - echoRiseUs;
    echoDone  = true;
  }
}

float distanceCM = 999;              // 999 = no echo / nothing in range
bool  obstacleDetected = false;
bool  usWaiting = false;
unsigned long usTrigMs = 0;
int   clearCount = 0;

void applyDistance(float cm) {
  distanceCM = cm;
  bool nearNow = (cm > 0 && cm <= OBSTACLE_DISTANCE_CM);
  if (nearNow) {
    clearCount = 0;
    if (!obstacleDetected) { obstacleDetected = true; Serial.print("[OBSTACLE] "); Serial.print(cm, 1); Serial.println(" cm - stopping"); }
  } else if (obstacleDetected) {
    if (++clearCount >= OBSTACLE_CLEAR_READS) { obstacleDetected = false; Serial.println("[OBSTACLE] cleared - resuming"); }
  }
}

void updateUltrasonic() {
  unsigned long now = millis();
  if (usWaiting) {
    if (echoDone) {
      uint32_t d = echoDurUs;
      usWaiting = false;
      applyDistance(d > 25000UL ? 999 : d * 0.0343f / 2.0f);     // > 25 ms = out of range
    } else if (now - usTrigMs >= US_ECHO_TIMEOUT) {
      usWaiting = false;
      applyDistance(999);                                        // no echo
    }
  }
  if (!usWaiting && now - usTrigMs >= US_READ_INTERVAL) {
    echoDone = false;
    echoRiseUs = 0;
    digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    usTrigMs = now;
    usWaiting = true;
  }
}

/* ============================ BUZZER (non-blocking beep-beep) ============================ */
enum BeepState { BEEP_IDLE, BEEP_ON1, BEEP_OFF1, BEEP_ON2, BEEP_OFF2 };
BeepState beepState = BEEP_IDLE;
unsigned long beepTimer = 0;
unsigned long lastBeepTime = 0;

void updateBuzzer() {
  unsigned long now = millis();
  if (!obstacleDetected) {                       // obstacle nei -> chup
    beepState = BEEP_IDLE;
    lastBeepTime = 0;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }
  if (beepState == BEEP_IDLE && (lastBeepTime == 0 || now - lastBeepTime >= BEEP_REPEAT_MS)) {
    beepState = BEEP_ON1;
    beepTimer = now;
    lastBeepTime = now;
    digitalWrite(BUZZER_PIN, HIGH);
  }
  switch (beepState) {
    case BEEP_ON1:
      if (now - beepTimer >= BEEP_ON_MS)  { digitalWrite(BUZZER_PIN, LOW);  beepTimer = now; beepState = BEEP_OFF1; }
      break;
    case BEEP_OFF1:
      if (now - beepTimer >= BEEP_OFF_MS) { digitalWrite(BUZZER_PIN, HIGH); beepTimer = now; beepState = BEEP_ON2; }
      break;
    case BEEP_ON2:
      if (now - beepTimer >= BEEP_ON_MS)  { digitalWrite(BUZZER_PIN, LOW);  beepTimer = now; beepState = BEEP_OFF2; }
      break;
    case BEEP_OFF2:
      if (now - beepTimer >= BEEP_OFF_MS) { beepState = BEEP_IDLE; }
      break;
    default: break;
  }
}

/* ============================ LINE FOLLOW ============================ */
void resetPID() {
  lastError = 0; dFilt = 0; lineLost = false;
}

void lineFollowStep() {                 // runs every 10 ms
  float error;
  unsigned long now = millis();

  if (!computeError(error)) {
    if (!lineLost) { lineLost = true; lostSince = now; }
    unsigned long lostFor = now - lostSince;

    if (lostFor > LOST_STOP_MS) {       // safety: line khuje paowa jayni
      mode = "STOP";
      driveStop();
      Serial.println("[LINE] Line lost for too long. Robot stopped.");
      return;
    }
    int s = turnSpeed;
    if (fabs(lastError) < 1.5 && lostFor < LOST_STRAIGHT_MS) {
      setMotors(baseSpeed, baseSpeed);  // line majhkhane chilo -> gap hote pare, shoja jao
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
  dFilt += 0.25f * (dRaw - dFilt);      // smoothed derivative
  lastError = error;

  float correction = Kp * error + Kd * dFilt;
  float speed = baseSpeed * (1.0f - CURVE_SLOWDOWN * fabs(error));   // bank e speed kome

  int l = (int)(speed + correction);
  int r = (int)(speed - correction);
  l = constrain(l, -MAX_REVERSE, 255);
  r = constrain(r, -MAX_REVERSE, 255);
  setMotors(l, r);
}

/* ============================ MANUAL ============================ */
void manualDrive() {
  int s = manualSpeed;
  if      (manualCmd == "F") setMotors(s, s);
  else if (manualCmd == "B") setMotors(-s, -s);
  else if (manualCmd == "L") { int t = max(s, turnSpeed); setMotors(-t, t); }   // turn LEFT (same convention as line follow)
  else if (manualCmd == "R") { int t = max(s, turnSpeed); setMotors(t, -t); }   // turn RIGHT
  else driveStop();
}

/* ============================ SERIAL ============================ */
void setMode(String m) {
  if (m != "LINE" && m != "MANUAL") m = "STOP";
  mode = m;
  resetPID();
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

  float e;
  Serial.print(" | err=");
  if (computeError(e)) Serial.print(e, 1); else Serial.print("NO-LINE");
  Serial.print(" L="); Serial.print(lastLeftCmd);
  Serial.print(" R="); Serial.print(lastRightCmd);

  Serial.print(" | dist=");
  if (distanceCM >= 999) Serial.print("--"); else Serial.print(distanceCM, 1);
  Serial.print("cm obs="); Serial.print(obstacleDetected ? 1 : 0);

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
 .dist{text-align:center;margin-top:8px;font-size:15px}
 .dist.warn{color:#ff5b5b;font-weight:700}
</style></head><body>
<header>ESP32 4WD LINE BOT <span id="ver" style="font-weight:400;color:#8fa3b0"></span></header>
<div class="tabs">
  <div class="tab on" id="t1" onclick="tab(1)">LINE FOLLOW</div>
  <div class="tab" id="t2" onclick="tab(2)">MANUAL</div>
</div>
<div class="panel on" id="p1">
  <button class="go" onclick="send('/mode?m=LINE')">START LINE FOLLOWING</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/mode?m=STOP')">STOP</button>
  <div class="sp">Base Speed: <b id="bsv">150</b>
    <input type="range" min="60" max="255" value="150" oninput="bsv.innerText=this.value" onchange="send('/set?base='+this.value)">
  </div>
  <div class="sp">Min PWM (chaka na ghurle barao): <b id="mpv">110</b>
    <input type="range" min="0" max="220" value="110" oninput="mpv.innerText=this.value" onchange="send('/set?min='+this.value)">
  </div>
  <div class="sp">Turn Speed (line khojar ghurni): <b id="tsv">180</b>
    <input type="range" min="80" max="255" value="180" oninput="tsv.innerText=this.value" onchange="send('/set?turn='+this.value)">
  </div>
  <div class="sp">Kp: <b id="kpv">25</b>
    <input type="range" min="0" max="80" value="25" oninput="kpv.innerText=this.value" onchange="send('/set?kp='+this.value)">
  </div>
  <div class="sp">Kd: <b id="kdv">20</b>
    <input type="range" min="0" max="80" value="20" oninput="kdv.innerText=this.value" onchange="send('/set?kd='+this.value)">
  </div>
  <div class="sens" id="sens"></div>
  <div class="st" id="st">mode: --</div>
  <div class="dist" id="dist">distance: -- cm</div>
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
    <input type="range" min="60" max="255" value="180" oninput="msv.innerText=this.value" onchange="send('/set?man='+this.value)">
  </div>
  <div class="dist" id="dist2">distance: -- cm</div>
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
  document.getElementById('ver').innerText=j.ver;
  let txt='distance: '+(j.dist>=999?'--':j.dist.toFixed(1))+' cm'+(j.obs?'  OBSTACLE!':'');
  dist.innerText=txt; dist.className='dist'+(j.obs?' warn':'');
  dist2.innerText=txt; dist2.className='dist'+(j.obs?' warn':'');
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
  if (server.hasArg("kp"))   Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))   Kd = server.arg("kd").toFloat();
  if (server.hasArg("min"))  minPWM    = constrain(server.arg("min").toInt(), 0, 255);
  if (server.hasArg("turn")) turnSpeed = constrain(server.arg("turn").toInt(), 0, 255);
  Serial.print("[SET] minPWM="); Serial.print(minPWM);
  Serial.print(" turn="); Serial.print(turnSpeed);
  Serial.print(" base="); Serial.print(baseSpeed);
  Serial.print(" manual="); Serial.print(manualSpeed);
  Serial.print(" Kp="); Serial.print(Kp, 1);
  Serial.print(" Kd="); Serial.println(Kd, 1);
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  float e;
  String errStr = computeError(e) ? String(e, 1) : String("NO-LINE");
  String j = "{\"ver\":\"" FW_VERSION "\",\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 8; i++) { j += String(irVal[i]); if (i < 7) j += ","; }
  j += "],\"err\":\"" + errStr + "\",\"l\":" + String(lastLeftCmd) + ",\"r\":" + String(lastRightCmd);
  j += ",\"dist\":" + String(distanceCM, 1) + ",\"obs\":" + String(obstacleDetected ? "true" : "false") + "}";
  server.send(200, "application/json", j);
}

/* ============================ SETUP / LOOP ============================ */
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("============== NEXROUTE LINE FOLLOWER ==============");
  Serial.print("Firmware: "); Serial.print(FW_NAME); Serial.print("  "); Serial.println(FW_VERSION);
  Serial.print("Chip: "); Serial.print(ESP.getChipModel());
  Serial.print("  rev "); Serial.print(ESP.getChipRevision());
  Serial.print("  cores "); Serial.print(ESP.getChipCores());
  Serial.print("  "); Serial.print(ESP.getCpuFreqMHz()); Serial.println(" MHz");
  Serial.println("Status: IR (left->right, 1=line) flips | mode | err (-=left, +=right) L R | dist obs | loops/s");
  Serial.println("Serial commands: g = start, s = stop");
  Serial.println("=====================================================");

  for (int i = 0; i < 8; i++) pinMode(irPin[i], INPUT);
  for (int i = 0; i < 8; i++) {
    irVal[i] = readRaw(i);
    irRawLast[i] = irVal[i];
    irRawSince[i] = millis();
  }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

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
  updateUltrasonic();
  updateBuzzer();
  loopCount++;

  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_US) {
    lastControlUs = nowUs;

    // obstacle thakle thamo; shudhu manual e pichone (B) jete deya hoy
    bool blocked = OBSTACLE_STOP_ENABLED && obstacleDetected && !(mode == "MANUAL" && manualCmd == "B");

    if (mode == "LINE") {
      if (blocked) {
        driveStop();
        dFilt = 0;
        if (lineLost) lostSince = millis();     // thama obosthay "line lost" timer cholbe na
      } else {
        lineFollowStep();
      }
    }
    else if (mode == "MANUAL") { if (blocked) driveStop(); else manualDrive(); }
    else                       driveStop();
  }

  printStatus();
}
