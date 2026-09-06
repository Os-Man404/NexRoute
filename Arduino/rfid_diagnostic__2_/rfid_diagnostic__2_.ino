/*
 * RC522 CONNECTION DIAGNOSTIC
 * ----------------------------
 * Card scan er age RC522 module er sathe SPI communication
 * thik ache kina check kore. Version register porbe.
 *
 * UPDATED wiring - SDA aikhon GPIO2 e REAL chip-select hisebe
 * (temporary, testing er jonno - GPIO0 board e nai bole GPIO2 use
 * kora hocche, jeta ekhon ultrasonic er sathe connected na):
 *   SCK  -> GPIO 21
 *   MOSI -> GPIO 12
 *   MISO -> GPIO 34
 *   SDA  -> GPIO 2   (temporary, testing)
 *   RST  -> 3.3V (direct)
 *   VCC  -> 3.3V
 *   GND  -> GND
 */

#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN   2    // TEMPORARY chip-select pin for testing (GPIO0 nai bole)
#define SCK_PIN  21
#define MOSI_PIN 12
#define MISO_PIN 34
#define RST_PIN  UINT8_MAX

MFRC522 rfid(SS_PIN, RST_PIN);

void setup() {
  Serial.begin(115200);
  delay(1000);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();
  delay(100);
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);   // antenna sensitivity max kora hocche

  Serial.println();
  Serial.println("=== RC522 DIAGNOSTIC ===");

  byte version = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  Serial.print("Version register: 0x");
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF) {
    Serial.println("FAIL: RC522 er sathe communication hocche na.");
    Serial.println("Check: VCC=3.3V?, GND thik?, SCK/MOSI/MISO pin thik?");
    Serial.println("Check: SDA sotti GND e soldered/connected?");
  } else {
    Serial.println("OK: RC522 shonakto hoyeche. Card dhorun ekhon.");
  }
  Serial.println("========================");

  Serial.println();
  Serial.println("=== SELF TEST (official MFRC522 test) ===");
  bool selfTestResult = rfid.PCD_PerformSelfTest();
  Serial.print("Self-test result: ");
  Serial.println(selfTestResult ? "PASS" : "FAIL");
  Serial.println("==========================================");

  // Self-test resets some registers, so re-init before normal use
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(MFRC522::RxGain_max);
  rfid.PCD_AntennaOn();   // antenna sure kore chalu kora hocche
}

unsigned long lastHeartbeat = 0;

void loop() {
  // Heartbeat - eta print hote thakle bujhben loop() thikmoto cholche
  if (millis() - lastHeartbeat > 2000) {
    lastHeartbeat = millis();
    Serial.println("... waiting for card ...");
  }

  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  Serial.print("UID: ");
  String uidStr = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) { Serial.print("0"); uidStr += "0"; }
    Serial.print(rfid.uid.uidByte[i], HEX);
    uidStr += String(rfid.uid.uidByte[i], HEX);
    if (i < rfid.uid.size - 1) { Serial.print(" "); uidStr += " "; }
  }
  Serial.println();
  Serial.print("Copy-paste format: {0x");
  uidStr.replace(" ", ", 0x");
  Serial.print(uidStr);
  Serial.println("}");
  Serial.println("------------------------");

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  delay(800);
}
