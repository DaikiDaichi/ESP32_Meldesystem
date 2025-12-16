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

// Sliding‑Window‑Buffer
int movingBuffer[WINDOW_SIZE] = {0};
int staticBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;
int detectionCounter = 0;

// Zustände
bool radarActive = false;
bool radarScanning = false;
unsigned long lastScanChange = 0;       // für 5s/3s‑Zyklus
unsigned long lastAuthorizedUID = 0;    // Anti‑Repeat‑Timer
const unsigned long NFC_COOLDOWN = 5000; // 5 s Sperre

// Hilfsfunktionen
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

  RadarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(200);

  // Radar dauerhaft an (Engineering‑Mode)
  const uint8_t cmdStart[]  = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04};
  const uint8_t cmdEngMod[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0x62,0x00,0x01,0x00,0x67};
  for (uint8_t b : cmdStart)  RadarSerial.write(b);
  delay(100);
  for (uint8_t b : cmdEngMod) RadarSerial.write(b);
  delay(100);
  Serial.println("Radar in Engineering Mode gesetzt.");

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
    if (avgMoving > 0)
      Serial.printf("➡️ Bewegung erkannt bei %d cm\n", avgMoving);
    else if (avgStatic > 0)
      Serial.printf("🧍 Statische Person bei %d cm\n", avgStatic);
    setLED(0, 0, 255);    // Blau = Anwesenheit
  } else {
    Serial.println("❌ Keine Anwesenheit");
    setLED(0, 255, 0);    // Grün = aktiv
  }
}

// === NFC‑Routine mit Anti‑Repeat ===
void checkNFC() {
  uint8_t uid[7]; uint8_t uidLength;
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 20);
  if (!success) return;

  // Sperre: nur alle 5 Sekunden reagieren
  if (millis() - lastAuthorizedUID < NFC_COOLDOWN) {
    Serial.println("⏳ NFC‑Karte ignoriert (Cool‑Down aktiv)");
    return;
  }

  Serial.print("NFC UID: ");
  for (uint8_t i = 0; i < uidLength; i++) Serial.printf("%02X ", uid[i]);
  Serial.println();

  if (uidLength == allowedUIDLength && memcmp(uid, allowedUID, allowedUIDLength) == 0) {
    radarActive = !radarActive;
    lastAuthorizedUID = millis(); // Sperre starten
    if (radarActive) {
      Serial.println("✅ UID erlaubt – Radar aktiviert");
      setLED(0, 255, 0);
      lastScanChange = millis();  // Starte sofort Scannen
      radarScanning = true;
    } else {
      Serial.println("🛑 Radar deaktiviert");
      radarScanning = false;
      setLED(0, 0, 0);
    }
  } else {
    Serial.println("❌ Nicht erlaubte UID");
    setLED(255, 0, 0);
    delay(200);
    setLED(0, 0, 0);
  }
}

// === Loop ===
void loop() {
  static unsigned long lastNFC = 0;
  static unsigned long lastRadar = 0;
  static unsigned long lastWake = 0;
  unsigned long now = millis();

  // Lege Zeitsteuerung für Scannen fest: alle 5 s aktivieren, 3 s messen
  if (radarActive) {
    if (!radarScanning && now - lastScanChange >= 5000) {
      radarScanning = true;
      lastScanChange = now;
      Serial.println("🔄 Radar‑Scan gestartet (3 s)");
    }
    if (radarScanning && now - lastScanChange >= 3000) {
      radarScanning = false;
      lastScanChange = now;
      Serial.println("⏹️ Radar Scan pausiert");
      setLED(0, 100, 0);  // leichtes Grün in Pause
    }
  }

  // Radar‑Weckimpuls alle 2 s
  if (radarActive && now - lastWake > 2000) {
    const uint8_t cmdStart[]  = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04};
    for (uint8_t b : cmdStart) RadarSerial.write(b);
    lastWake = now;
  }

  // NFC alle 200 ms prüfen
  if (now - lastNFC >= 200) {
    checkNFC();
    lastNFC = now;
  }

  // Radar nur während der aktiven 3 Sekunden messen
  if (radarActive && radarScanning && now - lastRadar >= 100) {
    updateRadar();
    lastRadar = now;
  }
}