/*
 * NexRoute - RFID (RC522) TEST
 * VERSION: v1.2   (2026-10-05)
 *
 * Version history:
 *   v1.0  Checks that the RC522 answers, prints the UID of every tag.
 *   v1.1  SPI write/read-back check, antenna check, shows the reader's answer.
 *   v1.2  v1.1 proved SPI + antenna switch are OK but no tag ever answers.
 *         This version tries 12 different radio settings one after another
 *         (6 receiver gains x normal / boosted transmit power) and shows
 *         which setting, if any, gets an answer from a tag.
 * -------------------------------------------------------------------------
 * Board in Arduino IDE : ESP32 Dev Module
 * Serial Monitor       : 115200 baud
 * Library needed       : MFRC522
 *
 * Wiring: SDA(SS)=GPIO2  SCK=GPIO21  MOSI=GPIO12  MISO=GPIO34  RST=3V3  3.3V=3V3  GND=GND
 *
 * HOW TO USE: tag ta reader er antenna er thik majhkhane, gaye lagiye
 * 30 second sthir kore dhore rakho. Protita setting er jonno ekta line ashbe.
 */

#include <SPI.h>
#include <MFRC522.h>

#define FW_NAME    "NexRoute RFID Test"
#define FW_VERSION "v1.2"

#define RFID_SCK  21
#define RFID_MOSI 12
#define RFID_MISO 34
#define RFID_SDA  2
#define RFID_RST  UINT8_MAX

MFRC522 rfid(RFID_SDA, RFID_RST);

const byte GAINS[6]       = {0x00, 0x10, 0x40, 0x50, 0x60, 0x70};   // RxGain register values
const char* GAIN_NAMES[6] = {"18dB", "23dB", "33dB", "38dB", "43dB", "48dB"};
const int TRIES_PER_SETTING = 12;

int  setting = 0;                    // 0..11
int  roundNo = 1;
unsigned long totalAnswers = 0;

void applySetting(int s) {
  bool boost = (s >= 6);
  int  g = s % 6;
  rfid.PCD_Init();                                   // soft reset + library defaults
  delay(20);
  if (boost) {                                       // transmit driver shorbocco shokti
    rfid.PCD_WriteRegister(MFRC522::GsNReg,    0xFF);
    rfid.PCD_WriteRegister(MFRC522::CWGsPReg,  0x3F);
    rfid.PCD_WriteRegister(MFRC522::ModGsPReg, 0x3F);
  }
  rfid.PCD_AntennaOn();
  rfid.PCD_SetAntennaGain(GAINS[g]);
  delay(10);
}

void printUid() {
  Serial.print("   >>> TAG UID:");
  for (byte i = 0; i < rfid.uid.size; i++) {
    Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(rfid.uid.uidByte[i], HEX);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("================ NEXROUTE RFID TEST ================");
  Serial.print("Firmware: "); Serial.print(FW_NAME); Serial.print("  "); Serial.println(FW_VERSION);
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfid.PCD_Init();
  delay(50);
  Serial.print("version register: 0x"); Serial.println(rfid.PCD_ReadRegister(MFRC522::VersionReg), HEX);
  Serial.println("Tag ta antenna er majhkhane gaye lagiye 30 second dhore rakho.");
  Serial.println("=====================================================");
}

void loop() {
  applySetting(setting);

  int answered = 0, uidOk = 0, errors = 0;
  bool printed = false;
  for (int t = 0; t < TRIES_PER_SETTING; t++) {
    byte atqa[2];
    byte atqaSize = sizeof(atqa);
    MFRC522::StatusCode st = rfid.PICC_WakeupA(atqa, &atqaSize);
    if (st == MFRC522::STATUS_OK || st == MFRC522::STATUS_COLLISION) {
      answered++;
      if (rfid.PICC_ReadCardSerial()) {
        uidOk++;
        if (!printed) { printUid(); printed = true; }
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
      }
    } else if (st != MFRC522::STATUS_TIMEOUT) {
      errors++;
    }
    delay(40);
  }
  totalAnswers += answered;

  Serial.print("round "); Serial.print(roundNo);
  Serial.print("  setting "); Serial.print(setting + 1); Serial.print("/12  power=");
  Serial.print(setting >= 6 ? "BOOST " : "normal");
  Serial.print("  gain="); Serial.print(GAIN_NAMES[setting % 6]);
  Serial.print("  -> answered "); Serial.print(answered); Serial.print("/"); Serial.print(TRIES_PER_SETTING);
  Serial.print("  UID read "); Serial.print(uidOk);
  Serial.print("  errors "); Serial.println(errors);

  setting++;
  if (setting >= 12) {
    setting = 0;
    Serial.print("=== round "); Serial.print(roundNo); Serial.print(" finished: total answers so far = ");
    Serial.print(totalAnswers);
    Serial.println(totalAnswers == 0 ? "  (kono setting ei tag uttor dey ni)" : "");
    roundNo++;
  }
}
