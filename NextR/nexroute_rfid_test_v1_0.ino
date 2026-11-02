/*
 * NexRoute - RFID (RC522) TEST
 * VERSION: v1.1   (2026-10-05)
 *
 * Version history:
 *   v1.0  Checks that the RC522 answers, prints the UID of every tag.
 *   v1.1  Deeper check, because v1.0 showed "reader OK" but no tag was read:
 *         - reads the version register 20 times (same value every time?)
 *         - writes 8 bytes into the reader and reads them back (SPI wires OK?)
 *         - checks that the antenna is switched on
 *         - shows what the reader answers when it looks for a tag
 * -------------------------------------------------------------------------
 * Board in Arduino IDE : ESP32 Dev Module
 * Serial Monitor       : 115200 baud
 * Library needed       : MFRC522
 *
 * Wiring: SDA(SS)=GPIO2  SCK=GPIO21  MOSI=GPIO12  MISO=GPIO34  RST=3V3  3.3V=3V3  GND=GND
 * MOSI er tar GPIO25 e shoriye thakle niche RFID_MOSI 25 koro.
 */

#include <SPI.h>
#include <MFRC522.h>

#define FW_NAME    "NexRoute RFID Test"
#define FW_VERSION "v1.1"

#define RFID_SCK  21
#define RFID_MOSI 12                 // MOSI GPIO25 e lagale ekhane 25 likho
#define RFID_MISO 34
#define RFID_SDA  2
#define RFID_RST  UINT8_MAX          // RST tied to 3.3V

MFRC522 rfid(RFID_SDA, RFID_RST);

struct KnownTag { const char* name; byte uid[4]; };
const KnownTag KNOWN[] = {
  {"J1", {0xBC, 0x74, 0xC1, 0x01}},
  {"X",  {0xAC, 0x71, 0x8F, 0x02}},
  {"Y",  {0x09, 0xB2, 0x30, 0x03}},
};
const int NUM_KNOWN = sizeof(KNOWN) / sizeof(KNOWN[0]);

unsigned long tagCount = 0;
unsigned long lastReport = 0;
unsigned long cntTimeout = 0, cntAnswer = 0, cntOther = 0;
int lastStatus = -1;

void printHex(byte v) {
  Serial.print(v < 0x10 ? "0x0" : "0x");
  Serial.print(v, HEX);
}

void selfCheck() {
  Serial.println("--- CHECK 1: version register, 20 bar ---");
  byte first = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  int different = 0;
  for (int i = 0; i < 20; i++) {
    byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    if (v != first) different++;
    delay(2);
  }
  Serial.print("value = "); printHex(first);
  Serial.print("   onno rokom elo: "); Serial.print(different); Serial.println(" / 20");
  if (first == 0x00 || first == 0xFF) Serial.println(">> FAIL: reader reply dicche na (wiring / power)");
  else if (different > 0)            Serial.println(">> PROBLEM: man bodlacche = SPI er tar dhila ba beshi lomba");
  else if (first == 0x91 || first == 0x92) Serial.println(">> OK: original MFRC522");
  else Serial.println(">> reply shothik thake, kintu chip ta clone (0x91/0x92 na)");

  Serial.println("--- CHECK 2: 8 byte likhe abar pora (SPI er duto dik) ---");
  byte pattern[8] = {0x00, 0xFF, 0xAA, 0x55, 0x01, 0x80, 0x3C, 0xC3};
  byte back[8];
  rfid.PCD_WriteRegister(MFRC522::CommandReg, MFRC522::PCD_Idle);
  rfid.PCD_WriteRegister(MFRC522::FIFOLevelReg, 0x80);            // FIFO khali koro
  rfid.PCD_WriteRegister(MFRC522::FIFODataReg, 8, pattern);
  byte level = rfid.PCD_ReadRegister(MFRC522::FIFOLevelReg);
  rfid.PCD_ReadRegister(MFRC522::FIFODataReg, 8, back, 0);
  int wrong = 0;
  Serial.print("likhechi:");
  for (int i = 0; i < 8; i++) { Serial.print(' '); printHex(pattern[i]); }
  Serial.println();
  Serial.print("porechi :");
  for (int i = 0; i < 8; i++) { Serial.print(' '); printHex(back[i]); if (back[i] != pattern[i]) wrong++; }
  Serial.println();
  Serial.print("FIFO level = "); Serial.print(level); Serial.print(" (8 howar kotha), bhul byte = "); Serial.println(wrong);
  if (level == 8 && wrong == 0) Serial.println(">> OK: SPI communication thik");
  else Serial.println(">> PROBLEM: SPI te bhul hocche -> SCK/MOSI/MISO/SDA tar choto koro, shokto kore lagao");
  rfid.PCD_WriteRegister(MFRC522::FIFOLevelReg, 0x80);

  Serial.println("--- CHECK 3: antenna ---");
  rfid.PCD_AntennaOn();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  byte tx = rfid.PCD_ReadRegister(MFRC522::TxControlReg);
  Serial.print("TxControlReg = "); printHex(tx);
  Serial.println((tx & 0x03) == 0x03 ? "  >> OK: antenna ON" : "  >> PROBLEM: antenna ON hocche na");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("================ NEXROUTE RFID TEST ================");
  Serial.print("Firmware: "); Serial.print(FW_NAME); Serial.print("  "); Serial.println(FW_VERSION);
  Serial.print("Pins: SDA="); Serial.print(RFID_SDA);
  Serial.print(" SCK=");  Serial.print(RFID_SCK);
  Serial.print(" MOSI="); Serial.print(RFID_MOSI);
  Serial.print(" MISO="); Serial.println(RFID_MISO);

  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SDA);
  rfid.PCD_Init();
  delay(50);
  selfCheck();
  Serial.println("--- CHECK 4: ekhon ekta tag reader er 1-3 cm kache dhoro ---");
  Serial.println("=====================================================");
}

void loop() {
  byte atqa[2];
  byte atqaSize = sizeof(atqa);
  MFRC522::StatusCode st = rfid.PICC_WakeupA(atqa, &atqaSize);    // "kono tag acho?"

  if (st == MFRC522::STATUS_TIMEOUT) cntTimeout++;
  else if (st == MFRC522::STATUS_OK || st == MFRC522::STATUS_COLLISION) cntAnswer++;
  else cntOther++;

  if ((int)st != lastStatus) {                       // answer bodlale shathe shathe dekhao
    lastStatus = (int)st;
    Serial.print("reader answer: "); Serial.println(rfid.GetStatusCodeName(st));
  }

  if (st == MFRC522::STATUS_OK || st == MFRC522::STATUS_COLLISION) {
    if (rfid.PICC_ReadCardSerial()) {
      tagCount++;
      Serial.print("TAG #"); Serial.print(tagCount); Serial.print("  UID:");
      for (byte i = 0; i < rfid.uid.size; i++) {
        Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
        Serial.print(rfid.uid.uidByte[i], HEX);
      }
      const char* name = "UNKNOWN (code er TAGS list e nei)";
      for (int k = 0; k < NUM_KNOWN; k++) {
        if (rfid.uid.size == 4 && memcmp(rfid.uid.uidByte, KNOWN[k].uid, 4) == 0) name = KNOWN[k].name;
      }
      Serial.print("   -> "); Serial.println(name);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
      delay(800);                                    // eki tag bar bar na chapanor jonno
    } else {
      Serial.println("tag uttor diyeche, kintu UID pora jayni (tag aro kache / sthir kore dhoro)");
    }
  }

  if (millis() - lastReport >= 5000) {
    lastReport = millis();
    Serial.print("last 5 s: no tag="); Serial.print(cntTimeout);
    Serial.print("  tag answered="); Serial.print(cntAnswer);
    Serial.print("  error="); Serial.print(cntOther);
    Serial.print("   (tags read so far: "); Serial.print(tagCount); Serial.println(")");
    cntTimeout = cntAnswer = cntOther = 0;
  }
  delay(50);
}
