#include <Wire.h>
#include <Adafruit_PN532.h>
#include <LD2412.h>

// === Pin‑Definitionen ===
#define PIN_RED    27
#define PIN_GREEN  26
#define PIN_BLUE   25
#define RXD2 16
#define TXD2 17
#define SDA_PIN 21
#define SCL_PIN 22

// === Objekte ===
HardwareSerial RadarSerial(2);
LD2412 radar(RadarSerial);
Adafruit_PN532 nfc(-1, -1, &Wire);

// === Autorisierte UID ===
const uint8_t allowedUID[] = {0x1D, 0x34, 0x64, 0x06};
const uint8_t allowedUIDLength = 4;

// === Radar‑Parameter ===
const int MIN_DETECTION_CM = 80;
const int WINDOW_SIZE = 5;
const int DETECTION_COUNT = 3;

// === Gleitmittel‑Buffer ===
int movingBuffer[WINDOW_SIZE] = {0};
int staticBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;
int detectionCounter = 0;

// Zustände
bool radarActive = false;

// === Hilfsfunktionen ===
int getAverage(int* buffer, int size) {
  long sum = 0;
  for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

void setLED(int r, int g, int b) {
  analogWrite(PIN_RED, r);
  analogWrite(PIN_GREEN, g);
  analogWrite(PIN_BLUE, b);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("=== Start: NFC + LD2412 ===");

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  setLED(0, 0, 0);

  // Radar‑Schnittstelle
  RadarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(200);
  (void)radar.movingDistance();   // Module „aufwecken“
  (void)radar.staticDistance();
  Serial.println("Radar‑Serial aktiv, warte auf Daten...");

  // NFC‑Init
  Wire.begin(SDA_PIN, SCL_PIN);
  nfc.begin();

  uint32_t version = nfc.getFirmwareVersion();
  if (!version) {
    Serial.println("❌ PN532 nicht gefunden");
    while (true) { digitalWrite(PIN_RED, HIGH); delay(300); digitalWrite(PIN_RED, LOW); delay(300); }
  }

  nfc.SAMConfig();
  Serial.println("✅ PN532 bereit – Lege autorisierte Karte auf.");
}

// === Radar‑Routine ===
void updateRadar() {
  while (RadarSerial.available() > 64) RadarSerial.read();

  int movingDist = radar.movingDistance();
  int staticDist = radar.staticDistance();

  if (movingDist < MIN_DETECTION_CM) movingDist = 0;
  if (staticDist < MIN_DETECTION_CM) staticDist = 0;

  movingBuffer[bufferIndex] = movingDist;
  staticBuffer[bufferIndex] = staticDist;
  bufferIndex = (bufferIndex + 1) % WINDOW_SIZE;

  int avgMoving = getAverage(movingBuffer, WINDOW_SIZE);
  int avgStatic = getAverage(staticBuffer, WINDOW_SIZE);

  if (avgMoving > 0 || avgStatic > 0) detectionCounter++;
  else detectionCounter = 0;

  if (detectionCounter >= DETECTION_COUNT) {
    if (avgMoving > 0) {
      Serial.printf("➡️ Bewegung erkannt bei %d cm\n", avgMoving);
    } else if (avgStatic > 0) {
      Serial.printf("🧍 Statische Person bei %d cm\n", avgStatic);
    }
    setLED(0, 0, 255);    // Blau bei Anwesenheit
  } else {
    Serial.println("❌ Keine Anwesenheit");
    setLED(0, 255, 0);    // Grün = aktiv, aber leer
  }
}

// === NFC‑Routine ===
void checkNFC() {
  uint8_t uid[7]; uint8_t uidLength;

  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 20); // kurzer Timeout
  if (!success) return;  // nichts erkannt, schnell zurück

  Serial.print("NFC UID: ");
  for (uint8_t i = 0; i < uidLength; i++) Serial.printf("%02X ", uid[i]);
  Serial.println();

  if (uidLength == allowedUIDLength && memcmp(uid, allowedUID, allowedUIDLength) == 0) {
    radarActive = !radarActive;
    if (radarActive) {
      Serial.println("✅ UID erlaubt – Radar aktiviert");
      setLED(0, 255, 0);  // Grün
    } else {
      Serial.println("🛑 Radar deaktiviert");
      setLED(0, 0, 0);
    }
  } else {
    Serial.println("❌ Nicht erlaubte UID");
    setLED(255, 0, 0);
    delay(200);
    setLED(0, 0, 0);
  }
}

// === Haupt‑Loop ===
void loop() {
  static unsigned long lastNFC = 0;
  static unsigned long lastRadar = 0;
  unsigned long now = millis();

  // NFC alle 200 ms
  if (now - lastNFC >= 200) {
    checkNFC();
    lastNFC = now;
  }

  // Radar alle 100 ms
  if (radarActive && now - lastRadar >= 100) {
    updateRadar();
    lastRadar = now;
  }
}