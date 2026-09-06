/*
 * ESP32 4WD Line Follower + Manual Control + Ultrasonic Obstacle Stop (v4)
 * -------------------------------------------------------------------------
 * Motor driver: L298N board with INA / INB / INC / IND inputs (NO enable pins)
 *   INA + INB  -> Left  side motors  (OUT1/OUT2)
 *   INC + IND  -> Right side motors  (OUT3/OUT4)
 * Sensor: 8 channel IR array (digital out)
 * NEW in v4: HC-SR04 ultrasonic obstacle detection + buzzer alert
 *
 * WiFi AP:  LineBot_ESP32 / 12345678   ->  http://192.168.4.1
 * Requires ESP32 Arduino core 3.x
 */

#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <MFRC522.h>

/* ================= WIFI ================= */
const char* AP_SSID = "LineBot_ESP32";
const char* AP_PASS = "12345678";
WebServer server(80);

/* ============== IR SENSOR PINS (Left -> Right) ============== */
const int irPin[8] = {13, 15, 14, 27, 26, 25, 33, 32};
int irVal[8];

/* Black line on white floor -> true ; White line on black floor -> false */
bool BLACK_LINE = true;

/* ============== MOTOR DRIVER PINS ==============
   PWM applied directly on input pins (no ENA/ENB on this board) */
#define INA 22   // Left  forward
#define INB 23   // Left  backward
#define INC 18   // Right forward
#define IND 19   // Right backward

#define PWM_FREQ 5000
#define PWM_RES  8      // 0-255

/* If one side spins the wrong way, flip its flag to true */
bool INVERT_LEFT  = false;
bool INVERT_RIGHT = false;

/* ============== RFID (RC522) PINS + UIDs ============== */
#define RFID_SCK  21
#define RFID_MOSI 12
#define RFID_MISO 34
#define RFID_SDA  2     // apnar existing wiring onujayi (GPIO2 e ageo SDA lagano chilo)
#define RFID_RST  UINT8_MAX   // RST tied directly to 3.3V, no GPIO used

MFRC522 rfid(RFID_SDA, RFID_RST);

// Confirmed UIDs from scanning (byte length = 4 for these tags)
byte UID_J1[] = {0xBC, 0x74, 0xC1, 0x01};   // Key ring -> Junction J1
byte UID_X[]  = {0xAC, 0x71, 0x8F, 0x02};   // Card 1   -> Sender X
byte UID_Y[]  = {0x09, 0xB2, 0x30, 0x03};   // Card 2   -> Receiver Y

String lastRfidLocation = "NONE";
unsigned long lastRfidTime = 0;
const unsigned long RFID_STOP_MS = 3000;   // basic test: stay stopped this long after X/Y scan

bool uidMatches(byte *scanned, byte size, byte *known, byte knownSize) {
  if (size != knownSize) return false;
  for (byte i = 0; i < size; i++) if (scanned[i] != known[i]) return false;
  return true;
}

/* ============== ULTRASONIC + BUZZER PINS ============== */
#define TRIG_PIN 4      // <-- age chilo 2, RFID SDA GPIO2 e thakay ekhane sorano hoyeche
#define ECHO_PIN 35     // age chilo 4, RFID SDA er jonno free kora hoyeche
#define BUZZER_PIN 5

const float OBSTACLE_DISTANCE_CM = 15.0;   // fixed stop distance
const unsigned long US_READ_INTERVAL = 60; // ms between ultrasonic reads (non-blocking)
const unsigned long BEEP_REPEAT_MS   = 10000; // re-beep every 10 sec if obstacle persists

float distanceCM = 999;
bool  obstacleDetected = false;

unsigned long lastUSRead   = 0;
unsigned long lastBeepTime = 0;   // when the beep-beep pattern last started

/* Non-blocking beep-beep state machine */
enum BeepState { BEEP_IDLE, BEEP_ON1, BEEP_OFF1, BEEP_ON2, BEEP_OFF2 };
BeepState beepState = BEEP_IDLE;
unsigned long beepTimer = 0;
const unsigned long BEEP_ON_MS  = 120;
const unsigned long BEEP_OFF_MS = 120;

/* ============== CONTROL VARIABLES ============== */
String mode = "STOP";          // LINE / MANUAL / STOP
String manualCmd = "S";        // F B L R S
int baseSpeed   = 150;
int manualSpeed = 180;

float Kp = 25.0, Ki = 0.0, Kd = 15.0;
float lastError = 0, integral = 0;
float lastKnownError = 0;

/* ================= MOTOR HELPERS ================= */
void leftMotor(int spd) {                  // -255 .. 255
  spd = constrain(spd, -255, 255);
  if (INVERT_LEFT) spd = -spd;
  if (spd >= 0) { ledcWrite(INA, spd);   ledcWrite(INB, 0); }
  else          { ledcWrite(INA, 0);     ledcWrite(INB, -spd); }
}

void rightMotor(int spd) {
  spd = constrain(spd, -255, 255);
  if (INVERT_RIGHT) spd = -spd;
  if (spd >= 0) { ledcWrite(INC, spd);   ledcWrite(IND, 0); }
  else          { ledcWrite(INC, 0);     ledcWrite(IND, -spd); }
}

void driveStop() { leftMotor(0); rightMotor(0); }

/* ================= RFID ================= */
bool rfidStopActive = false;   // true = currently in the "stopped after X/Y scan" window

void checkRFID() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_J1, sizeof(UID_J1))) {
    lastRfidLocation = "J1";
    Serial.println("[RFID] J1 (Junction) detected!");
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_X, sizeof(UID_X))) {
    lastRfidLocation = "X";
    Serial.println("[RFID] X (Sender) detected! Robot stopping.");
    rfidStopActive = true;
    lastRfidTime = millis();
  }
  else if (uidMatches(rfid.uid.uidByte, rfid.uid.size, UID_Y, sizeof(UID_Y))) {
    lastRfidLocation = "Y";
    Serial.println("[RFID] Y (Receiver) detected! Robot stopping.");
    rfidStopActive = true;
    lastRfidTime = millis();
  }
  else {
    Serial.print("[RFID] Unknown tag scanned, UID: ");
    for (byte i = 0; i < rfid.uid.size; i++) {
      Serial.print(rfid.uid.uidByte[i], HEX); Serial.print(" ");
    }
    Serial.println();
  }

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
}

/* ================= ULTRASONIC ================= */
float readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000UL); // 25ms timeout (~4m range)
  if (duration == 0) return 999;                    // no echo -> treat as "clear"

  float cm = duration * 0.0343 / 2.0;
  return cm;
}

void updateUltrasonic() {
  unsigned long now = millis();
  if (now - lastUSRead < US_READ_INTERVAL) return;
  lastUSRead = now;

  distanceCM = readUltrasonicCM();
  obstacleDetected = (distanceCM > 0 && distanceCM <= OBSTACLE_DISTANCE_CM);
}

/* ================= BUZZER (non-blocking beep-beep) ================= */
void updateBuzzer() {
  unsigned long now = millis();

  if (obstacleDetected) {
    // Start a new beep-beep pattern if idle and either first time or 10s passed
    if (beepState == BEEP_IDLE && (now - lastBeepTime >= BEEP_REPEAT_MS || lastBeepTime == 0)) {
      beepState  = BEEP_ON1;
      beepTimer  = now;
      lastBeepTime = now;
      digitalWrite(BUZZER_PIN, HIGH);
    }
  } else {
    // obstacle gone -> reset pattern & silence buzzer
    beepState  = BEEP_IDLE;
    lastBeepTime = 0;
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  // Step through beep-beep state machine
  switch (beepState) {
    case BEEP_ON1:
      if (now - beepTimer >= BEEP_ON_MS) { digitalWrite(BUZZER_PIN, LOW); beepTimer = now; beepState = BEEP_OFF1; }
      break;
    case BEEP_OFF1:
      if (now - beepTimer >= BEEP_OFF_MS) { digitalWrite(BUZZER_PIN, HIGH); beepTimer = now; beepState = BEEP_ON2; }
      break;
    case BEEP_ON2:
      if (now - beepTimer >= BEEP_ON_MS) { digitalWrite(BUZZER_PIN, LOW); beepTimer = now; beepState = BEEP_OFF2; }
      break;
    case BEEP_OFF2:
      if (now - beepTimer >= BEEP_OFF_MS) { beepState = BEEP_IDLE; }  // pattern done, wait for next 10s cycle
      break;
    default: break;
  }
}

/* ================= SENSOR READ ================= */
const float weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

bool readSensors(float &error) {
  float sum = 0; int count = 0;
  for (int i = 0; i < 8; i++) {
    int raw = digitalRead(irPin[i]);
    irVal[i] = BLACK_LINE ? (raw == LOW ? 1 : 0) : (raw == HIGH ? 1 : 0);
    if (irVal[i]) { sum += weight[i]; count++; }
  }
  if (count == 0) return false;            // line lost
  error = sum / count;
  return true;
}

/* ================= LINE FOLLOW ================= */
void lineFollow() {
  float error;
  if (!readSensors(error)) {
    int s = baseSpeed * 0.8;               // line hariye gele khuje
    if (lastKnownError < 0) { leftMotor(-s); rightMotor(s); }
    else                    { leftMotor(s);  rightMotor(-s); }
    return;
  }
  lastKnownError = error;

  integral += error;
  integral = constrain(integral, -50, 50);
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  leftMotor(baseSpeed + correction);
  rightMotor(baseSpeed - correction);
}

/* ================= MANUAL DRIVE ================= */
void manualDrive() {
  int s = manualSpeed;
  if      (manualCmd == "F") { leftMotor(s);  rightMotor(s);  }
  else if (manualCmd == "B") { leftMotor(-s); rightMotor(-s); }
  else if (manualCmd == "L") { leftMotor(s);  rightMotor(-s); }
  else if (manualCmd == "R") { leftMotor(-s); rightMotor(s);  }
  else driveStop();
}

/* ================= WEB PAGE ================= */
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
<header>ESP32 4WD LINE BOT</header>
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
  <div class="sp">Kp: <b id="kpv">25</b>
    <input type="range" min="0" max="80" value="25" oninput="kpv.innerText=this.value" onchange="send('/set?kp='+this.value)">
  </div>
  <div class="sp">Kd: <b id="kdv">15</b>
    <input type="range" min="0" max="80" value="15" oninput="kdv.innerText=this.value" onchange="send('/set?kd='+this.value)">
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
  st.innerText='mode: '+j.mode;
  let txt = 'distance: '+j.dist.toFixed(1)+' cm' + (j.obs ? '  OBSTACLE!' : '');
  dist.innerText = txt; dist.className = 'dist' + (j.obs?' warn':'');
  dist2.innerText = txt; dist2.className = 'dist' + (j.obs?' warn':'');
});},250);
</script></body></html>
)HTML";

/* ================= HANDLERS ================= */
void handleRoot() { server.send_P(200, "text/html", PAGE); }

void handleMode() {
  mode = server.arg("m");
  integral = 0; lastError = 0;
  if (mode != "LINE" && mode != "MANUAL") { mode = "STOP"; driveStop(); }
  server.send(200, "text/plain", "ok");
}

void handleManual() {
  manualCmd = server.arg("c");
  mode = "MANUAL";
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("base")) baseSpeed   = server.arg("base").toInt();
  if (server.hasArg("man"))  manualSpeed = server.arg("man").toInt();
  if (server.hasArg("kp"))   Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))   Kd = server.arg("kd").toFloat();
  server.send(200, "text/plain", "ok");
}

void handleStatus() {
  String j = "{\"mode\":\"" + mode + "\",\"s\":[";
  for (int i = 0; i < 8; i++) { j += String(irVal[i]); if (i < 7) j += ","; }
  j += "],\"dist\":" + String(distanceCM, 1) + ",\"obs\":" + String(obstacleDetected ? "true" : "false");
  j += ",\"rfid\":\"" + lastRfidLocation + "\",\"rfidStop\":" + String(rfidStopActive ? "true" : "false") + "}";
  server.send(200, "application/json", j);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 8; i++) pinMode(irPin[i], INPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // ledcAttach handles pinMode internally
  ledcAttach(INA, PWM_FREQ, PWM_RES);
  ledcAttach(INB, PWM_FREQ, PWM_RES);
  ledcAttach(INC, PWM_FREQ, PWM_RES);
  ledcAttach(IND, PWM_FREQ, PWM_RES);
  driveStop();

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  Serial.println("RFID module initialized.");

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());   // 192.168.4.1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.begin();
}

/* ================= LOOP ================= */
void loop() {
  server.handleClient();

  updateUltrasonic();
  updateBuzzer();
  checkRFID();

  // RFID basic test: after scanning X or Y, stay stopped for RFID_STOP_MS
  if (rfidStopActive && (millis() - lastRfidTime >= RFID_STOP_MS)) {
    rfidStopActive = false;   // release stop after the test window
    Serial.println("[RFID] Stop window over, resuming.");
  }

  // Obstacle handling:
  // - LINE mode: always stop if obstacle within range
  // - MANUAL mode: stop for F/L/R, but allow B (reversing away) even with obstacle
  bool blockDrive = (obstacleDetected && !(mode == "MANUAL" && manualCmd == "B")) || rfidStopActive;

  if (blockDrive) {
    driveStop();
  }
  else if (mode == "LINE")        lineFollow();
  else if (mode == "MANUAL") manualDrive();
  else {
    float e; readSensors(e);     // shudhu UI dot update er jonno
    driveStop();
  }
}
