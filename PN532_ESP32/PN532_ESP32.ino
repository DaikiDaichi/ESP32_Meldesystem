#include <Wire.h>
#include <Adafruit_PN532.h>

#define SDA_PIN 21
#define SCL_PIN 22

// IRQ und RESET nicht verwendet → -1
Adafruit_PN532 nfc(-1, -1, &Wire);

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("Initializing PN532...");

  Wire.begin(SDA_PIN, SCL_PIN);

  nfc.begin();

  uint32_t version = nfc.getFirmwareVersion();
  if (!version) {
    Serial.println("❌ Didn't find PN532 board");
    while (1);
  }

  Serial.print("✅ PN532 Firmware: 0x");
  Serial.println(version, HEX);

  nfc.SAMConfig();
  Serial.println("PN532 initialized!");
}

void loop() {
  Serial.println("Waiting for NFC tag...");

  uint8_t uid[7];
  uint8_t uidLength;

  bool success = nfc.readPassiveTargetID(
    PN532_MIFARE_ISO14443A,
    uid,
    &uidLength
  );

  if (success) {
    Serial.print("Found NFC tag UID: ");
    for (uint8_t i = 0; i < uidLength; i++) {
      if (uid[i] < 0x10) Serial.print("0");
      Serial.print(uid[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }

  delay(1000);
}
