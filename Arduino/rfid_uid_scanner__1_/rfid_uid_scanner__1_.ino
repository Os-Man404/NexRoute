// /*
//  * RFID UID SCANNER
//  * -----------------
//  * Eta diye apnar 3 ta RFID item (key ring = J1, Card1 = X, Card2 = Y)
//  * er UID ber korun. Serial Monitor (115200 baud) open kore ekta ekta
//  * tag scan korun ebong UID gulo note kore rakhun.
//  *
//  * Wiring (ESP32) - existing IR/motor/ultrasonic/buzzer pin OPORIBORTITO,
//  * GPIO0 O AVOID kora hoyeche:
//  *   RC522 SDA(SS) -> sorasori GND e (kono GPIO na, always-selected)
//  *   RC522 RST     -> sorasori 3.3V e (kono GPIO na)
//  *   RC522 SCK     -> GPIO 21
//  *   RC522 MOSI    -> GPIO 12
//  *   RC522 MISO    -> GPIO 34
//  *   RC522 VCC     -> 3.3V  (5V na!)
//  *   RC522 GND     -> ESP32 GND
//  *   RC522 IRQ     -> kicu lagabe na
//  *
//  * SS_PIN=35 nicher code e "dummy" - eta RC522 er sathe physically
//  * connect kora na, library ke satisfied rakhar jonno lagche matro.
//  * Asol SS control hocche hardware e GND e tie kora die (always active).
//  *
//  * Library: "MFRC522" by GithubCommunity (Library Manager theke install korun)
//  */

// #include <SPI.h>
// #include <MFRC522.h>

// #define SS_PIN   35   // DUMMY - RC522 er SDA physically GND e tied
// #define SCK_PIN  21
// #define MOSI_PIN 12
// #define MISO_PIN 34
// #define RST_PIN  UINT8_MAX   // RST 3.3V e tied, GPIO use hocche na

// MFRC522 rfid(SS_PIN, RST_PIN);

// void setup() {
//   Serial.begin(115200);
//   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
//   rfid.PCD_Init();
//   delay(100);

//   Serial.println();
//   Serial.println("=== RFID UID SCANNER ===");
//   Serial.println("Ekta tag/card reader er kache dhorun...");
// }

// void loop() {
//   if (!rfid.PICC_IsNewCardPresent()) return;
//   if (!rfid.PICC_ReadCardSerial())   return;

//   Serial.print("UID: ");
//   String uidStr = "";
//   for (byte i = 0; i < rfid.uid.size; i++) {
//     if (rfid.uid.uidByte[i] < 0x10) { Serial.print("0"); uidStr += "0"; }
//     Serial.print(rfid.uid.uidByte[i], HEX);
//     uidStr += String(rfid.uid.uidByte[i], HEX);
//     if (i < rfid.uid.size - 1) { Serial.print(" "); uidStr += " "; }
//   }
//   Serial.println();
//   Serial.print("Copy-paste format: {0x");
//   uidStr.replace(" ", ", 0x");
//   Serial.print(uidStr);
//   Serial.println("}");
//   Serial.println("------------------------");

//   rfid.PICC_HaltA();
//   rfid.PCD_StopCrypto1();
//   delay(800);
// }
/*
 * RC522 CONNECTION DIAGNOSTIC
 * ----------------------------
 * Card scan er age RC522 module er sathe SPI communication
 * thik ache kina check kore. Version register porbe.
 *
 * Same wiring:
 *   SCK  -> GPIO 21
 *   MOSI -> GPIO 12
 *   MISO -> GPIO 34
 *   SDA  -> GND (direct)
 *   RST  -> 3.3V (direct)
 *   VCC  -> 3.3V
 *   GND  -> GND
 */

#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN   0    // DUMMY - RC522 er sathe kono tar diye jukto na
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
}

void loop() {
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
