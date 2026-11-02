/*
 * NexRoute - ESP32 4WD LINE FOLLOWER + ULTRASONIC + BUZZER + RFID DELIVERY
 * VERSION: v1.9   (2026-10-06)
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
 *   v1.4  Built on v1.3 (line following, ultrasonic, buzzer unchanged).
 *         Added RFID delivery: at a junction tag the robot takes the turn
 *         written in the route map; everywhere else it is normal line
 *         following. DELIVERY tab with sender / receiver selection; only
 *         routes that are in the map work (now: X -> Y), any other pair
 *         shows "Map is not integrated". RFID is read on the second CPU
 *         core, so the control loop is NOT slowed down.
 *   v1.5  Serial Monitor shows the UID of every tag.
 *   v1.6  RFID reading rewritten (v1.4 read tags in the test sketch but not here):
 *         - the reader is started AFTER WiFi, and everything that talks to
 *           it runs in one task (same call sequence as the RFID test sketch)
 *         - the reader is checked every second; if a power dip has reset it
 *           (antenna off) it is started again automatically
 *         - a tag is reported once when it arrives, not again while it stays
 *         - every tag's UID is shown on Serial and on the web page, also
 *           unknown ones; reader state + polls per second are shown too
 *         - TAG SETUP on the DELIVERY tab: hold a tag near the reader and
 *           press SAVE AS J1 / X / Y (stored in the ESP32, no code edit)
 *         - fixed: a junction turn / U-turn could end after 0.15 s before
 *           the robot had really turned
 *         - junction "straight" now uses only the 2 middle sensors
 *         - new setting JUNCTION_ADVANCE_MS (drive on a little before turning)
 *   v1.7  At the junction tag (J1) the robot now STOPS for 3 seconds
 *         (JUNCTION_STOP_MS) and only then does what the map says (turn left /
 *         right or go straight). Everything else is unchanged.
 *   v1.8  The IR sensors are in front of the RFID reader, so they reach the
 *         junction before the tag is read and the side branch pulled the
 *         robot away. Now, during a delivery, when the IR sensors see a wide
 *         black area (a junction) the robot keeps going STRAIGHT on the 2
 *         middle sensors until the J1 tag is read (or JUNCTION_HOLD_MS ends).
 *         Then it stops 3 s and turns as before. Plain LINE FOLLOW mode is
 *         not changed.
 *   v1.9  DELIVERY tab has its own speed slider (Delivery Speed). It is used
 *         only while a delivery runs and can be changed during the delivery.
 *         The Base Speed of the LINE FOLLOW tab is not touched by it.
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
 * RFID RC522: SCK=21  MOSI=12  MISO=34  SDA=2  (RST tied to 3.3V)
 * Library needed: MFRC522 (by GithubCommunity / miguelbalboa)
 * WiFi AP: LineBot_ESP32 / 12345678  ->  http://192.168.4.1
 *
 * Serial commands:  g = start line following,  s = stop (also cancels a delivery)
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
#include <SPI.h>
#include <MFRC522.h>
#include <Preferences.h>

#define FW_NAME    "NexRoute LineBot"
#define FW_VERSION "v1.9"

/* ---- types (kept at the top so Arduino's auto-prototypes can see them) ---- */
enum RobotState {
  STATE_STANDBY,
  STATE_GOING_TO_SENDER,
  STATE_AT_SENDER,
  STATE_GOING_TO_RECEIVER,
  STATE_AT_RECEIVER,
  STATE_RETURNING_TO_BASE
};
enum Maneuver { MAN_NONE, MAN_PIVOT, MAN_STRAIGHT, MAN_ADVANCE, MAN_WAIT };
enum BeepState { BEEP_IDLE, BEEP_ON1, BEEP_OFF1, BEEP_ON2, BEEP_OFF2 };

struct RfidTag  { const char* name; byte size; byte uid[10]; };
struct TagEvent { byte size; byte uid[10]; };          // one tag read, sent from the RFID task to loop()
struct RouteMap {
  const char* sender;        // tag name of the sender
  const char* receiver;      // tag name of the receiver
  char j1ToSender;           // turn at junction J1 on the way base -> sender   ('L', 'R' or 'S' = straight)
  char j1ToReceiver;         // turn at J1 on the way sender -> receiver
  char j1ToBase;             // turn at J1 on the way receiver -> base
};

/* ======================= RFID TAGS + ROUTE MAP =======================
   Notun tag add korte TAGS e ekta line, notun route add korte ROUTES e ekta line.
   Ei UID gulo default. Web page er TAG SETUP diye save korle sheta ESP32 te
   theke jay ebong ei default er jaygay bebohar hoy. */
RfidTag TAGS[] = {
  {"J1", 4, {0xBC, 0x74, 0xC1, 0x01}},   // key ring -> junction
  {"X",  4, {0xAC, 0x71, 0x8F, 0x02}},   // card 1
  {"Y",  4, {0x09, 0xB2, 0x30, 0x03}},   // card 2
};
const int NUM_TAGS = sizeof(TAGS) / sizeof(TAGS[0]);

const RouteMap ROUTES[] = {
  // sender, receiver, J1: base->sender, sender->receiver, receiver->base
  {"X", "Y", 'L', 'L', 'S'},
};
const int NUM_ROUTES = sizeof(ROUTES) / sizeof(ROUTES[0]);

/* ============================ SETTINGS ============================ */
bool SWAP_LEFT_RIGHT      = false;   // TEST 1: left arrow e robot dane ghurle true
bool INVERT_LEFT          = false;   // left side ulta dike ghurle true
bool INVERT_RIGHT         = false;   // right side ulta dike ghurle true
bool REVERSE_SENSOR_ORDER = false;   // TEST 2: sensor order ulta hole true
bool BLACK_LINE           = true;    // true = white floor e black line

int   baseSpeed   = 150;             // straight line speed (0-255), LINE FOLLOW tab
int   deliverySpeed = 150;           // delivery cholar shomoy er speed, DELIVERY tab er slider
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

/* ---- RFID reader ---- */
bool RFID_TX_BOOST = false;                         // true = reader er transmit power shorbocco (durbol reader hole)
const unsigned long RFID_POLL_GAP_MS = 10;          // dui bar tag khojar majhe biroti
const unsigned long RFID_REARM_MS    = 700;         // tag eto ms na dekha gele tarpor abar elei "notun" dhora hoy
const unsigned long RFID_HEALTH_MS   = 1000;        // koto ms por por reader thik ache kina dekhbe

/* ---- RFID delivery: junction turn + U-turn ---- */
bool TURN_USE_SENSORS = true;                       // true = line abar majhkhane na asha porjonto ghore; false = shudhu shomoy dhore ghore
const int           JUNCTION_IR_COUNT      = 5;     // eto ba beshi sensor eksathe kalo = IR junction dekheche
const unsigned long JUNCTION_HOLD_MS       = 900;   // IR junction dekhar por eto ms porjonto shoja jay, J1 tag er opekkhay
const unsigned long JUNCTION_STOP_MS       = 3000;  // J1 tag pele robot eto ms thame thake, tarpor map onujayi chole (0 = thambe na)
const unsigned long JUNCTION_ADVANCE_MS    = 0;     // J1 tag porar por eto ms shoja giye tarpor ghurbe (robot er majhkhan junction er upor anar jonno)
const unsigned long JUNCTION_TURN_MIN_MS   = 250;   // sensor mode: er age notun line dhora hobe na (90 degree turn er prai ordhek shomoy dao)
const unsigned long JUNCTION_TURN_MAX_MS   = 1500;  // sensor mode: eto ms eo line na pele turn chere dey
const unsigned long JUNCTION_TURN_FIXED_MS = 500;   // timed mode: 90 degree turn er shomoy
const unsigned long UTURN_MIN_MS   = 500;           // sensor mode: U-turn e er age notun line dhora hobe na
const unsigned long UTURN_MAX_MS   = 3000;
const unsigned long UTURN_FIXED_MS = 1000;          // timed mode: 180 degree turn er shomoy
const char          UTURN_DIR      = 'R';           // U-turn kon dike ghurbe ('L' / 'R')
const unsigned long JUNCTION_STRAIGHT_MS = 400;     // 'S' hole eto ms shudhu majher 2 ta sensor dekhe shoja jay
const unsigned long JUNCTION_GUARD_MS    = 1500;    // eki junction tag eto ms er moddhe dui bar dhora hobe na
const unsigned long BASE_MARKER_MS       = 150;     // 8 ta sensor eto ms kalo = base
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

#define RFID_SCK  21
#define RFID_MOSI 12
#define RFID_MISO 34
#define RFID_SDA  2
#define RFID_RST  UINT8_MAX          // RST tied to 3.3V, no GPIO used
MFRC522 rfid(RFID_SDA, RFID_RST);

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
uint32_t lastPollCount = 0;

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

/* ============================ RFID DELIVERY ============================ */
RobotState robotState = STATE_STANDBY;
bool   autoModeActive = false;            // true = delivery cholche
int    routeIndex = -1;                   // ROUTES er kon route cholche
String senderName = "X", receiverName = "Y";
String lastTagName = "NONE";              // last tag er nam (J1 / X / Y / UNKNOWN / NONE)
String lastUidStr  = "";                  // last tag er UID, "AC 71 8F 02"
byte   lastUid[10];
byte   lastUidSize = 0;

Maneuver maneuver = MAN_NONE;
unsigned long manStart = 0, manMinMs = 0, manMaxMs = 0, manFixedMs = 0;
int    pivotDir = 1;                      // -1 = left, +1 = right
char   pendingTurn = 'L';                 // junction e thama / egonor por ki korbe ('L', 'R', 'S')
bool   manIsJunction = false;             // ei maneuver ta junction er (U-turn er na)
bool   holdActive = false;                // IR junction dekheche, J1 tag er opekkhay shoja jacche
bool   holdArmed  = false;                // base er chowra kalo dag chere shoru line e uthar por true hoy
unsigned long holdStart = 0;
bool   pivotLeftLine = false;
unsigned long lastJunctionMs = 0;
unsigned long baseMarkerStart = 0;
bool   baseMarkerTiming = false;

void endDelivery(const char* why);        // defined below

/* ---------------- RFID reader task ----------------
   Everything that talks to the RC522 runs in this one task (CPU core 0), so
   loop() on core 1 never waits for the reader. A tag that has been read is
   passed to loop() through a queue. */
QueueHandle_t rfidQueue = NULL;
volatile bool     rfidReaderOk    = false;   // reader answers and its antenna is on
volatile byte     rfidVersion     = 0;
volatile uint32_t rfidPollCount   = 0;       // how many times the reader has looked for a tag
volatile uint32_t rfidReinitCount = 0;       // how many times the reader had to be restarted

bool rfidInitReader() {
  rfid.PCD_Init();                           // soft reset + default settings
  vTaskDelay(pdMS_TO_TICKS(30));
  if (RFID_TX_BOOST) {
    rfid.PCD_WriteRegister(MFRC522::GsNReg,    0xFF);
    rfid.PCD_WriteRegister(MFRC522::CWGsPReg,  0x3F);
    rfid.PCD_WriteRegister(MFRC522::ModGsPReg, 0x3F);
  }
  rfid.PCD_AntennaOn();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  byte v  = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  byte tx = rfid.PCD_ReadRegister(MFRC522::TxControlReg);
  rfidVersion = v;
  return !(v == 0x00 || v == 0xFF) && ((tx & 0x03) == 0x03);
}

void rfidTask(void* arg) {
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfidReaderOk = rfidInitReader();
  Serial.print("[RFID] reader version 0x"); Serial.print(rfidVersion, HEX);
  Serial.println(rfidReaderOk ? "  -> OK, antenna ON" : "  -> FAIL: reader reply dicche na / antenna ON hocche na (wiring, 3V3, RST check koro)");

  byte seenUid[10];
  byte seenSize = 0;
  unsigned long seenAt = 0;
  unsigned long lastHealth = millis();

  for (;;) {
    unsigned long now = millis();

    if (now - lastHealth >= RFID_HEALTH_MS) {          // reader ekhono thik ache?
      lastHealth = now;
      byte v  = rfid.PCD_ReadRegister(MFRC522::VersionReg);
      byte tx = rfid.PCD_ReadRegister(MFRC522::TxControlReg);
      bool ok = !(v == 0x00 || v == 0xFF) && ((tx & 0x03) == 0x03);
      if (!ok) {                                       // power dip e reset hoye geche -> abar chalu koro
        rfidReinitCount++;
        rfidReaderOk = rfidInitReader();
        Serial.print("[RFID] reader restarted (#"); Serial.print(rfidReinitCount);
        Serial.println(rfidReaderOk ? ") -> OK" : ") -> still FAIL");
      } else {
        rfidReaderOk = true;
      }
    }

    if (rfidReaderOk) {
      rfid.PCD_WriteRegister(MFRC522::TxModeReg,   0x00);   // same reset the library does before looking for a tag
      rfid.PCD_WriteRegister(MFRC522::RxModeReg,   0x00);
      rfid.PCD_WriteRegister(MFRC522::ModWidthReg, 0x26);
      byte atqa[2];
      byte atqaSize = sizeof(atqa);
      MFRC522::StatusCode st = rfid.PICC_WakeupA(atqa, &atqaSize);   // "kono tag acho?"
      rfidPollCount++;
      if (st == MFRC522::STATUS_OK || st == MFRC522::STATUS_COLLISION) {
        if (rfid.PICC_ReadCardSerial()) {
          byte sz = rfid.uid.size;
          if (sz > 10) sz = 10;
          bool stillThere = (sz == seenSize) && (memcmp(rfid.uid.uidByte, seenUid, sz) == 0)
                            && (millis() - seenAt < RFID_REARM_MS);
          memcpy(seenUid, rfid.uid.uidByte, sz);
          seenSize = sz;
          seenAt = millis();
          if (!stillThere && rfidQueue != NULL) {      // notun tag -> loop() ke janao
            TagEvent ev;
            ev.size = sz;
            memcpy(ev.uid, rfid.uid.uidByte, sz);
            xQueueSend(rfidQueue, &ev, 0);
          }
          rfid.PICC_HaltA();
          rfid.PCD_StopCrypto1();
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(RFID_POLL_GAP_MS));
  }
}

/* ---------------- tag table helpers ---------------- */
Preferences prefs;

String uidToString(const byte* uid, byte size) {
  String out = "";
  for (byte i = 0; i < size; i++) {
    if (i) out += " ";
    if (uid[i] < 0x10) out += "0";
    out += String(uid[i], HEX);
  }
  out.toUpperCase();
  return out;
}

int findTagByUid(const byte* uid, byte size) {
  for (int i = 0; i < NUM_TAGS; i++) {
    if (TAGS[i].size == size && memcmp(TAGS[i].uid, uid, size) == 0) return i;
  }
  return -1;
}

int findTagByName(const String& name) {
  for (int i = 0; i < NUM_TAGS; i++) if (name == TAGS[i].name) return i;
  return -1;
}

String tagTableString() {
  String out = "";
  for (int i = 0; i < NUM_TAGS; i++) {
    if (i) out += "   ";
    out += String(TAGS[i].name) + " = " + uidToString(TAGS[i].uid, TAGS[i].size);
  }
  return out;
}

/* Tags saved from the web page replace the defaults written in the code. */
void loadTags() {
  prefs.begin("nexroute", false);
  for (int i = 0; i < NUM_TAGS; i++) {
    byte buf[11];
    size_t n = prefs.getBytesLength(TAGS[i].name);
    if (n >= 2 && n <= sizeof(buf)) {
      prefs.getBytes(TAGS[i].name, buf, n);
      if (buf[0] == n - 1) { TAGS[i].size = buf[0]; memcpy(TAGS[i].uid, buf + 1, buf[0]); }
    }
  }
  prefs.end();
  Serial.print("[RFID] tags: "); Serial.println(tagTableString());
}

bool saveTag(int i) {
  byte buf[11];
  buf[0] = TAGS[i].size;
  memcpy(buf + 1, TAGS[i].uid, TAGS[i].size);
  prefs.begin("nexroute", false);
  size_t written = prefs.putBytes(TAGS[i].name, buf, TAGS[i].size + 1);
  prefs.end();
  return written == (size_t)(TAGS[i].size + 1);
}

const char* stateName(RobotState st) {
  switch (st) {
    case STATE_STANDBY:           return "STANDBY";
    case STATE_GOING_TO_SENDER:   return "GOING_TO_SENDER";
    case STATE_AT_SENDER:         return "AT_SENDER";
    case STATE_GOING_TO_RECEIVER: return "GOING_TO_RECEIVER";
    case STATE_AT_RECEIVER:       return "AT_RECEIVER";
    case STATE_RETURNING_TO_BASE: return "RETURNING_TO_BASE";
  }
  return "UNKNOWN";
}

int findRoute(const String& sender, const String& receiver) {
  for (int i = 0; i < NUM_ROUTES; i++) {
    if (sender == ROUTES[i].sender && receiver == ROUTES[i].receiver) return i;
  }
  return -1;                              // map e nei
}

void startPivot(char dir, unsigned long minMs, unsigned long maxMs, unsigned long fixedMs) {
  holdActive = false;
  maneuver = MAN_PIVOT;
  manStart = millis();
  manMinMs = minMs; manMaxMs = maxMs; manFixedMs = fixedMs;
  pivotDir = (dir == 'L') ? -1 : 1;
  pivotLeftLine = false;
  Serial.print("[TURN] start "); Serial.println(dir);
}

/* What the map says at the junction, done after the junction stop. */
void doJunctionMove(char dir) {
  manIsJunction = true;
  if (dir == 'S') {
    maneuver = MAN_STRAIGHT;
    manStart = millis();
    Serial.println("[J1] going straight");
  } else if (JUNCTION_ADVANCE_MS > 0) {      // age ektu shoja, tarpor ghora
    maneuver = MAN_ADVANCE;
    manStart = millis();
    pendingTurn = dir;
  } else {
    startPivot(dir, JUNCTION_TURN_MIN_MS, JUNCTION_TURN_MAX_MS, JUNCTION_TURN_FIXED_MS);
  }
}

void startJunctionAction(char dir) {
  holdActive = false;
  lastJunctionMs = millis();
  manIsJunction = true;
  Serial.print("[J1] map: "); Serial.println(dir == 'S' ? "straight" : (dir == 'L' ? "turn L" : "turn R"));
  if (JUNCTION_STOP_MS > 0) {                // age thamo, tarpor map er kaj
    maneuver = MAN_WAIT;
    manStart = millis();
    pendingTurn = dir;
    driveStop();
    Serial.print("[J1] stopping for "); Serial.print(JUNCTION_STOP_MS); Serial.println(" ms");
  } else {
    doJunctionMove(dir);
  }
}

void endDelivery(const char* why) {
  autoModeActive = false;
  robotState = STATE_STANDBY;
  maneuver = MAN_NONE;
  holdActive = false;
  routeIndex = -1;
  baseMarkerTiming = false;
  mode = "STOP";
  driveStop();
  Serial.print("[DELIVERY] ended: "); Serial.println(why);
}

/* Called from loop(): reacts to every tag that the RFID task has just read. */
void processTag(const TagEvent& ev);

void handleRfidEvents() {
  if (rfidQueue == NULL) return;
  TagEvent ev;
  while (xQueueReceive(rfidQueue, &ev, 0) == pdTRUE) processTag(ev);
}

void processTag(const TagEvent& ev) {
  lastUidSize = ev.size;
  memcpy(lastUid, ev.uid, ev.size);
  lastUidStr = uidToString(ev.uid, ev.size);
  int idx = findTagByUid(ev.uid, ev.size);
  lastTagName = (idx >= 0) ? TAGS[idx].name : "UNKNOWN";
  Serial.print("[RFID] UID: "); Serial.print(lastUidStr);
  Serial.print("  -> "); Serial.println(idx >= 0 ? lastTagName.c_str() : "UNKNOWN (web page er TAG SETUP diye save koro)");
  if (idx < 0) return;

  if (!autoModeActive || routeIndex < 0) return;      // delivery na cholle shudhu dekhay
  const RouteMap& r = ROUTES[routeIndex];

  if (lastTagName == "J1") {                          // junction -> map onujayi turn
    if (maneuver != MAN_NONE) return;
    if (lastJunctionMs != 0 && millis() - lastJunctionMs < JUNCTION_GUARD_MS) return;
    if      (robotState == STATE_GOING_TO_SENDER)   startJunctionAction(r.j1ToSender);
    else if (robotState == STATE_GOING_TO_RECEIVER) startJunctionAction(r.j1ToReceiver);
    else if (robotState == STATE_RETURNING_TO_BASE) startJunctionAction(r.j1ToBase);
  }
  else if (robotState == STATE_GOING_TO_SENDER && lastTagName == r.sender) {
    robotState = STATE_AT_SENDER;
    maneuver = MAN_NONE;
    driveStop();
    Serial.println("[DELIVERY] at sender - waiting for ITEM LOADED");
  }
  else if (robotState == STATE_GOING_TO_RECEIVER && lastTagName == r.receiver) {
    robotState = STATE_AT_RECEIVER;
    maneuver = MAN_NONE;
    driveStop();
    Serial.println("[DELIVERY] at receiver - waiting for RECEIVED");
  }
}

bool checkBaseMarker() {
  bool allBlack = true;
  for (int i = 0; i < 8; i++) if (irVal[i] == 0) { allBlack = false; break; }
  if (!allBlack) { baseMarkerTiming = false; return false; }
  if (!baseMarkerTiming) { baseMarkerTiming = true; baseMarkerStart = millis(); }
  return millis() - baseMarkerStart >= BASE_MARKER_MS;
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
      if (autoModeActive) endDelivery("line lost");
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

/* ============================ DELIVERY MOTION ============================ */
/* Turn on the spot. Sensor mode: keep turning until the middle sensors have
   left the old line and found a line again. Timed mode: turn for a fixed time. */
void pivotStep() {
  unsigned long elapsed = millis() - manStart;
  int t = turnSpeed;
  if (pivotDir < 0) setMotors(-t, t); else setMotors(t, -t);

  bool done;
  if (!TURN_USE_SENSORS) {
    done = elapsed >= manFixedMs;
  } else {
    bool centre = irVal[3] || irVal[4];              // majher duto sensor
    if (!centre) pivotLeftLine = true;               // purono line theke shore geche
    done = (pivotLeftLine && centre && elapsed >= manMinMs)   // shore giye abar ekta line peyeche
           || elapsed >= manMaxMs;                            // khub beshi shomoy -> chere dao
  }
  if (done) {
    maneuver = MAN_NONE;
    resetPID();
    lastSide = pivotDir;                     // line na pele eki dike khujbe
    if (manIsJunction) { lastJunctionMs = millis(); manIsJunction = false; }   // eki J1 tag abar dhora theke rokkha
    Serial.print("[TURN] done in "); Serial.print(elapsed); Serial.println(" ms");
  }
}

/* Junction "straight": follow with the 2 middle sensors only, so the side
   branch of the junction cannot pull the robot off the main line. */
void centreFollowStep() {
  float sum = 0; int count = 0;
  for (int i = 3; i <= 4; i++) if (irVal[i]) { sum += weight[i]; count++; }
  if (count == 0) { setMotors(baseSpeed, baseSpeed); return; }
  float correction = Kp * (sum / count);
  setMotors(constrain((int)(baseSpeed + correction), 0, 255), constrain((int)(baseSpeed - correction), 0, 255));
}

void deliveryStepInner(bool blocked) {
  if (maneuver == MAN_PIVOT) { pivotStep(); return; }       // ghorar shomoy obstacle e thame na

  if (maneuver == MAN_WAIT) {                               // junction e thame ache
    driveStop();
    lastJunctionMs = millis();
    if (millis() - manStart >= JUNCTION_STOP_MS) doJunctionMove(pendingTurn);
    return;
  }

  if (robotState == STATE_AT_SENDER || robotState == STATE_AT_RECEIVER || robotState == STATE_STANDBY) {
    driveStop();                             // app er button er opekkha
    return;
  }
  if (blocked) {
    driveStop();
    dFilt = 0;
    if (lineLost) lostSince = millis();
    if (maneuver == MAN_STRAIGHT || maneuver == MAN_ADVANCE) manStart += CONTROL_US / 1000;   // thama obosthay timer cholbe na
    if (holdActive) holdStart += CONTROL_US / 1000;
    return;
  }
  if (maneuver == MAN_ADVANCE) {
    if (millis() - manStart < JUNCTION_ADVANCE_MS) { centreFollowStep(); return; }
    startPivot(pendingTurn, JUNCTION_TURN_MIN_MS, JUNCTION_TURN_MAX_MS, JUNCTION_TURN_FIXED_MS);
    return;
  }
  if (maneuver == MAN_STRAIGHT) {
    if (millis() - manStart < JUNCTION_STRAIGHT_MS) { centreFollowStep(); return; }
    maneuver = MAN_NONE;
    lastJunctionMs = millis();
    manIsJunction = false;
    resetPID();
  }
  if (robotState == STATE_RETURNING_TO_BASE && checkBaseMarker()) {
    endDelivery("base reached");
    return;
  }

  // IR sensors reach the junction before the RFID reader does. A wide black
  // area = junction: keep going straight until the J1 tag is read.
  int blackCount = 0;
  for (int i = 0; i < 8; i++) blackCount += irVal[i];
  bool recentJunction = (lastJunctionMs != 0 && millis() - lastJunctionMs < JUNCTION_GUARD_MS);   // ei junction er kaj hoye geche
  if (blackCount >= 1 && blackCount <= 3) holdArmed = true;                 // shadharon shoru line e ache
  if (holdArmed && blackCount >= JUNCTION_IR_COUNT && !recentJunction) {
    if (!holdActive) Serial.println("[IR] junction seen - going straight until the J1 tag is read");
    holdActive = true;
    holdStart = millis();
  }
  if (holdActive) {
    if (millis() - holdStart < JUNCTION_HOLD_MS) { centreFollowStep(); return; }
    holdActive = false;
    resetPID();
    Serial.println("[IR] no J1 tag after the junction - back to normal line following");
  }
  lineFollowStep();                          // junction chara baki shob jaygay normal line following
}

void deliveryStep(bool blocked) {          // runs every 10 ms while a delivery is active
  int lineSpeed = baseSpeed;                 // delivery te nijer speed bebohar hoy,
  baseSpeed = deliverySpeed;                 // line following er code eki thake
  deliveryStepInner(blocked);
  baseSpeed = lineSpeed;
}

/* ============================ SERIAL ============================ */
void setMode(String m) {
  if (m != "LINE" && m != "MANUAL") m = "STOP";
  if (autoModeActive) {                    // delivery cholar shomoy: STOP = cancel, baki ignore
    if (m == "STOP") endDelivery("stopped by user");
    return;
  }
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

  Serial.print(" | mode="); Serial.print(autoModeActive ? "DELIVERY" : mode.c_str());
  if (mode == "MANUAL") { Serial.print(" cmd="); Serial.print(manualCmd); }
  if (autoModeActive) {
    Serial.print(" state="); Serial.print(stateName(robotState));
    Serial.print(" spd="); Serial.print(deliverySpeed);
    if (maneuver == MAN_PIVOT) Serial.print(" [TURN]");
    if (maneuver == MAN_STRAIGHT) Serial.print(" [J-STRAIGHT]");
    if (maneuver == MAN_ADVANCE) Serial.print(" [J-ADVANCE]");
    if (maneuver == MAN_WAIT) Serial.print(" [J-STOP]");
    if (holdActive) Serial.print(" [J-HOLD]");
  }
  Serial.print(" rfid="); Serial.print(lastTagName);
  Serial.print(rfidReaderOk ? " rd=OK" : " rd=OFF");
  uint32_t polls = rfidPollCount;
  Serial.print(" rp/s="); Serial.print((polls - lastPollCount) * 1000UL / dt);
  lastPollCount = polls;

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
  <div class="tab" id="t3" onclick="tab(3)">DELIVERY</div>
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
<div class="panel" id="p3">
  <div class="sp" style="text-align:center;font-size:16px">Robot state: <b id="rstate">--</b></div>
  <div class="st" id="rtag">last RFID tag: --</div>
  <div class="sp">Sender:
    <select id="senderSel" onchange="lbl()" style="width:100%;padding:10px;border-radius:8px;margin-top:6px;font-size:16px">
      <option>A</option><option>B</option><option>C</option><option>D</option><option>E</option><option>F</option>
      <option selected>X</option><option>Y</option><option>Z</option>
    </select>
  </div>
  <div class="sp">Receiver:
    <select id="receiverSel" onchange="lbl()" style="width:100%;padding:10px;border-radius:8px;margin-top:6px;font-size:16px">
      <option>A</option><option>B</option><option>C</option><option>D</option><option>E</option><option>F</option>
      <option>X</option><option selected>Y</option><option>Z</option>
    </select>
  </div>
  <div class="sp">Delivery Speed: <b id="dsv">150</b>
    <input type="range" id="dss" min="90" max="255" value="150" oninput="dsv.innerText=this.value;dsTouched=true" onchange="send('/set?dspeed='+this.value)">
  </div>
  <button class="go" onclick="callRobot()">CALL ROBOT</button>
  <div class="sp"></div>
  <button class="go" id="btnLoaded" onclick="send('/itemLoaded')" disabled>ITEM LOADED (at X)</button>
  <div class="sp"></div>
  <button class="go" id="btnReceived" onclick="send('/received')" disabled>RECEIVED (at Y)</button>
  <div class="sp"></div>
  <button class="stop" onclick="send('/cancelAuto')">CANCEL / RESET TO STANDBY</button>
  <div class="dist" id="dist3">distance: -- cm</div>
  <div class="sp" style="border-top:1px solid #2a3540;padding-top:16px">
    <div style="font-weight:600">TAG SETUP</div>
    <div class="st" style="text-align:left">Hold a tag near the reader, then save it.</div>
    <div class="st" style="text-align:left" id="luid">last tag UID: --</div>
    <div class="st" style="text-align:left" id="tags">saved: --</div>
    <div style="display:flex;gap:8px;margin-top:10px">
      <button class="dir" style="height:auto;padding:12px;font-size:14px" onclick="learn('J1')">SAVE AS J1</button>
      <button class="dir" style="height:auto;padding:12px;font-size:14px" onclick="learn('X')">SAVE AS X</button>
      <button class="dir" style="height:auto;padding:12px;font-size:14px" onclick="learn('Y')">SAVE AS Y</button>
    </div>
  </div>
</div>
<script>
let box=document.getElementById('sens');
for(let i=0;i<8;i++){let d=document.createElement('div');d.className='dot';d.id='d'+i;box.appendChild(d);}
let autoOn=false, curS='X', curR='Y', dsTouched=false;
function tab(n){
  t1.className='tab'+(n==1?' on':'');t2.className='tab'+(n==2?' on':'');t3.className='tab'+(n==3?' on':'');
  p1.className='panel'+(n==1?' on':'');p2.className='panel'+(n==2?' on':'');p3.className='panel'+(n==3?' on':'');
  if(n!=3 && !autoOn) send('/mode?m='+(n==1?'STOP':'MANUAL'));
}
function lbl(){
  const s=autoOn?curS:senderSel.value, r=autoOn?curR:receiverSel.value;
  btnLoaded.innerText='ITEM LOADED (at '+s+')';
  btnReceived.innerText='RECEIVED (at '+r+')';
}
function callRobot(){
  const s=senderSel.value, r=receiverSel.value;
  if(s===r){alert('Sender and Receiver cannot be the same.');return;}
  fetch('/call?sender='+s+'&receiver='+r).then(resp=>{
    if(resp.status==404) alert('Map is not integrated for '+s+' -> '+r+'.');
    else if(!resp.ok) resp.text().then(t=>alert(t));
  });
}
function learn(n){
  fetch('/learn?name='+n).then(resp=>resp.text().then(t=>alert(resp.ok?('Saved: this tag is now '+n):t)));
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
  dist3.innerText=txt; dist3.className='dist'+(j.obs?' warn':'');
  autoOn=j.auto; curS=j.sender; curR=j.receiver;
  if(!dsTouched){dss.value=j.dspeed;dsv.innerText=j.dspeed;}
  rstate.innerText=j.state+(j.auto?' ('+j.sender+' -> '+j.receiver+')':'')+(j.jstop?'  - stopped at junction':'');
  rtag.innerText='last RFID tag: '+j.rfid+'    reader: '+(j.rd?'OK':'NOT RESPONDING');
  luid.innerText='last tag UID: '+(j.uid?j.uid+'  ('+j.rfid+')':'--');
  tags.innerText='saved: '+j.tags;
  btnLoaded.disabled=(j.state!=='AT_SENDER');
  btnReceived.disabled=(j.state!=='AT_RECEIVER');
  lbl();
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
  if (autoModeActive) { server.send(409, "text/plain", "delivery running"); return; }
  if (mode != "MANUAL") setMode("MANUAL");
  manualCmd = c;
  server.send(200, "text/plain", "ok");
}

void handleSet() {
  if (server.hasArg("base")) baseSpeed   = constrain(server.arg("base").toInt(), 0, 255);
  if (server.hasArg("man"))  manualSpeed = constrain(server.arg("man").toInt(), 0, 255);
  if (server.hasArg("dspeed")) deliverySpeed = constrain(server.arg("dspeed").toInt(), 0, 255);
  if (server.hasArg("kp"))   Kp = server.arg("kp").toFloat();
  if (server.hasArg("kd"))   Kd = server.arg("kd").toFloat();
  if (server.hasArg("min"))  minPWM    = constrain(server.arg("min").toInt(), 0, 255);
  if (server.hasArg("turn")) turnSpeed = constrain(server.arg("turn").toInt(), 0, 255);
  Serial.print("[SET] minPWM="); Serial.print(minPWM);
  Serial.print(" turn="); Serial.print(turnSpeed);
  Serial.print(" base="); Serial.print(baseSpeed);
  Serial.print(" delivery="); Serial.print(deliverySpeed);
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
  j += ",\"dist\":" + String(distanceCM, 1) + ",\"obs\":" + String(obstacleDetected ? "true" : "false");
  j += ",\"state\":\"" + String(stateName(robotState)) + "\",\"auto\":" + String(autoModeActive ? "true" : "false");
  j += ",\"jstop\":" + String(maneuver == MAN_WAIT ? "true" : "false") + ",\"dspeed\":" + String(deliverySpeed);
  j += ",\"sender\":\"" + senderName + "\",\"receiver\":\"" + receiverName + "\",\"rfid\":\"" + lastTagName + "\"";
  j += ",\"uid\":\"" + lastUidStr + "\",\"rd\":" + String(rfidReaderOk ? "true" : "false") + ",\"tags\":\"" + tagTableString() + "\"}";
  server.send(200, "application/json", j);
}

void handleCall() {
  String sender = server.arg("sender"), receiver = server.arg("receiver");
  if (autoModeActive) { server.send(409, "text/plain", "A delivery is already running. Cancel it first."); return; }
  int ri = findRoute(sender, receiver);
  if (ri < 0) {
    Serial.print("[APP] Map is not integrated: "); Serial.print(sender); Serial.print(" -> "); Serial.println(receiver);
    server.send(404, "text/plain", "Map is not integrated");
    return;
  }
  setMode("STOP");
  routeIndex = ri;
  senderName = sender; receiverName = receiver;
  maneuver = MAN_NONE;
  holdActive = false;
  holdArmed = false;                         // base er kalo dag ke junction dhorbe na
  lastJunctionMs = 0;
  baseMarkerTiming = false;
  resetPID();
  robotState = STATE_GOING_TO_SENDER;
  autoModeActive = true;
  Serial.print("[APP] Call: "); Serial.print(sender); Serial.print(" -> "); Serial.println(receiver);
  server.send(200, "text/plain", "ok");
}

void handleItemLoaded() {
  if (!autoModeActive || robotState != STATE_AT_SENDER) { server.send(400, "text/plain", "Robot is not at the sender."); return; }
  robotState = STATE_GOING_TO_RECEIVER;
  lastJunctionMs = 0;
  manIsJunction = false;
  startPivot(UTURN_DIR, UTURN_MIN_MS, UTURN_MAX_MS, UTURN_FIXED_MS);
  Serial.println("[APP] Item loaded - U-turn, then going to receiver");
  server.send(200, "text/plain", "ok");
}

void handleReceived() {
  if (!autoModeActive || robotState != STATE_AT_RECEIVER) { server.send(400, "text/plain", "Robot is not at the receiver."); return; }
  robotState = STATE_RETURNING_TO_BASE;
  lastJunctionMs = 0;
  baseMarkerTiming = false;
  manIsJunction = false;
  startPivot(UTURN_DIR, UTURN_MIN_MS, UTURN_MAX_MS, UTURN_FIXED_MS);
  Serial.println("[APP] Received - U-turn, then returning to base");
  server.send(200, "text/plain", "ok");
}

/* TAG SETUP: the tag that was read last becomes J1 / X / Y and is stored in the ESP32. */
void handleLearn() {
  String name = server.arg("name");
  if (autoModeActive) { server.send(409, "text/plain", "A delivery is running. Cancel it first."); return; }
  int ti = findTagByName(name);
  if (ti < 0) { server.send(400, "text/plain", "Unknown tag name."); return; }
  if (lastUidSize == 0) { server.send(400, "text/plain", "No tag has been read yet. Hold the tag near the reader first."); return; }
  int other = findTagByUid(lastUid, lastUidSize);
  if (other >= 0 && other != ti) {
    server.send(409, "text/plain", String("This tag is already saved as ") + TAGS[other].name + ". Use a different tag.");
    return;
  }
  TAGS[ti].size = lastUidSize;
  memcpy(TAGS[ti].uid, lastUid, lastUidSize);
  bool ok = saveTag(ti);
  lastTagName = TAGS[ti].name;
  Serial.print("[RFID] saved "); Serial.print(TAGS[ti].name); Serial.print(" = "); Serial.print(lastUidStr);
  Serial.println(ok ? "" : "  (WARNING: could not store, will be lost after restart)");
  server.send(200, "text/plain", "ok");
}

void handleCancelAuto() {
  if (autoModeActive) endDelivery("cancelled from app"); else { robotState = STATE_STANDBY; driveStop(); }
  server.send(200, "text/plain", "ok");
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
  Serial.println("Status: IR (left->right, 1=line) flips | mode state rfid rd(reader) rp/s(tag checks per s) | err L R | dist obs | loops/s");
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

  // RFID is started AFTER WiFi: the current spike when WiFi switches on can reset the reader
  loadTags();
  rfidQueue = xQueueCreate(8, sizeof(TagEvent));
  xTaskCreatePinnedToCore(rfidTask, "rfid", 6144, NULL, 1, NULL, 0);   // core 0; loop() runs on core 1

  server.on("/", handleRoot);
  server.on("/mode", handleMode);
  server.on("/manual", handleManual);
  server.on("/set", handleSet);
  server.on("/status", handleStatus);
  server.on("/call", handleCall);
  server.on("/itemLoaded", handleItemLoaded);
  server.on("/received", handleReceived);
  server.on("/cancelAuto", handleCancelAuto);
  server.on("/learn", handleLearn);
  server.begin();
}

void loop() {
  server.handleClient();
  handleSerial();
  updateIR();
  updateUltrasonic();
  updateBuzzer();
  handleRfidEvents();
  loopCount++;

  unsigned long nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_US) {
    lastControlUs = nowUs;

    // obstacle thakle thamo; shudhu manual e pichone (B) jete deya hoy
    bool blocked = OBSTACLE_STOP_ENABLED && obstacleDetected && !(mode == "MANUAL" && manualCmd == "B");

    if (autoModeActive) {
      deliveryStep(OBSTACLE_STOP_ENABLED && obstacleDetected);
    }
    else if (mode == "LINE") {
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
