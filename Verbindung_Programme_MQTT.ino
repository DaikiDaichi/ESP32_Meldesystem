#include <Wire.h>
#include <Adafruit_PN532.h>
#include <LD2412.h>
#include <WiFi.h>
#include <PubSubClient.h>   // MQTT hinzufügen

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

// === WiFi & MQTT ===
const char* WIFI_SSID = "DeinWLAN";
const char* WIFI_PASS = "DeinPasswort";
const char* MQTT_SERVER = "192.168.1.100"; // IP des Pi Zero
const int   MQTT_PORT = 1883;
const char* MQTT_TOPIC = "radar/movement";

WiFiClient espClient;
PubSubClient client(espClient);

// === Autorisierte UID ===
const uint8_t allowedUID[] = {0x1D, 0x34, 0x64, 0x06};
const uint8_t allowedUIDLength = 4;

// === Radarparameter ===
const int MIN_DETECTION_CM = 80;
const int WINDOW_SIZE = 5;
const int DETECTION_COUNT = 3;

// === Buffer & Zustände ===
int movingBuffer[WINDOW_SIZE] = {0};
int staticBuffer[WINDOW_SIZE] = {0};
int bufferIndex = 0;
int detectionCounter = 0;

bool radarActive = false;
bool radarScanning = false;
bool alertSent = false; // Schutz vor Mehrfachalarm

unsigned long lastScanChange = 0;
unsigned long lastAuthorizedUID = 0;
unsigned long lastNFC = 0;
unsigned long lastRadar = 0;
unsigned long lastWake = 0;

const unsigned long NFC_COOLDOWN = 5000;   // 5 s Sperre
const unsigned long SCAN_ON_TIME = 5000;   // 5 s messen
const unsigned long SCAN_OFF_TIME = 60000;  // 60 s Pause

// === LED‑Hilfen ===
int getAverage(int* buffer, int size) {
  long sum = 0; for (int i = 0; i < size; i++) sum += buffer[i];
  return sum / size;
}

void setLED(int r, int g, int b) {
  analogWrite(PIN_RED, r);
  analogWrite(PIN_GREEN, g);
  analogWrite(PIN_BLUE, b);
}

// „Atmende“ LED von Gelb (rot + grün) nach Rot
void breathingTransition(unsigned long duration) {
  unsigned long start = millis();
  while (millis() - start < duration) {
    float phase = float(millis() - start) / duration;  // 0.0 – 1.0
    float brightness = 0.5f * (1 + sin(phase * PI * 2 * 15)); // 15 Atmungen
    int red = int(255 * brightness);
    int green = int(255 * (1.0 - phase) * brightness); // nimmt Richtung Rot ab
    setLED(red, green, 0);
    delay(30);
  }
  setLED(255, 0, 0);
}

// === WiFi verbinden ===
void setup_wifi() {
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Verbinde mit WLAN");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("✅ Verbunden!");
}

// === MQTT reconnect ===
void reconnect() {
  while (!client.connected()) {
    Serial.print("Versuche MQTT zu verbinden...");
    if (client.connect("ESP32_Radar")) {
      Serial.println("✅ MQTT verbunden");
    } else {
      Serial.print("Fehler, rc=");
      Serial.print(client.state());
      Serial.println(" -> erneut versuchen in 5s");
      delay(5000);
    }
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Start: NFC + LD2412 ===");

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  setLED(0, 0, 0);

  RadarSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(200);

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

  // --- WLAN + MQTT starten ---
  setup_wifi();
  client.setServer(MQTT_SERVER, MQTT_PORT);
}

// === Radar‑Routine (mit MQTT + Schutz vor Mehrfachalarm) ===
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

  // Alarm nur senden, wenn Erkennung über DETECTION_COUNT und noch kein Alarm gesendet
  if (detectionCounter >= DETECTION_COUNT) {
    if (!alertSent) { // Schutz vor Mehrfachalarm
      if (avgMoving > 0) {
        Serial.printf("➡️ Bewegung erkannt bei %d cm\n", avgMoving);
        client.publish(MQTT_TOPIC, "Bewegung erkannt");
      } else if (avgStatic > 0) {
        Serial.printf("🧍 Statische Person bei %d cm\n", avgStatic);
        client.publish(MQTT_TOPIC, "Statische Person erkannt");
      }
      alertSent = true; // Alarm gesendet
    }
  } else {
    alertSent = false; // Reset für nächste Erkennung
    Serial.println("❌ Keine Anwesenheit");
    setLED(255, 0, 0); // rot
  }
}

// === NFC‑Routine ===
void checkNFC() {
  uint8_t uid[7]; uint8_t uidLength;
  bool success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 20);
  if (!success) return;

  if (millis() - lastAuthorizedUID < NFC_COOLDOWN) {
    Serial.println("⏳ Karte ignoriert (Cool‑Down)");
    return;
  }

  Serial.print("NFC UID: ");
  for (uint8_t i = 0; i < uidLength; i++) Serial.printf("%02X ", uid[i]);
  Serial.println();

  if (uidLength == allowedUIDLength && memcmp(uid, allowedUID, allowedUIDLength) == 0) {
    radarActive = !radarActive;
    lastAuthorizedUID = millis();

    if (radarActive) {
      Serial.println("✅ UID erlaubt – Radar wird aktiviert");
      breathingTransition(15000);
      setLED(255, 0, 0);
      lastScanChange = millis();
      radarScanning = true;
    } else {
      Serial.println("🛑 Radar deaktiviert");
      radarScanning = false;
      setLED(255, 255, 0);
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
  if (!client.connected()) reconnect();
  client.loop();

  unsigned long now = millis();

  if (radarActive) {
    if (!radarScanning && now - lastScanChange >= SCAN_OFF_TIME) {
      radarScanning = true;
      lastScanChange = now;
      Serial.println("🔄 Radar‑Scan gestartet (3 s)");
    }
    if (radarScanning && now - lastScanChange >= SCAN_ON_TIME) {
      radarScanning = false;
      lastScanChange = now;
      Serial.println("⏹️ Radar‑Scan pausiert");
      setLED(255, 255, 0);
    }
  }

  if (radarActive && now - lastWake > 2000) {
    const uint8_t cmdStart[]  = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04};
    for (uint8_t b : cmdStart) RadarSerial.write(b);
    lastWake = now;
  }

  if (now - lastNFC >= 200) {
    checkNFC();
    lastNFC = now;
  }

  if (radarActive && radarScanning && now - lastRadar >= 100) {
    updateRadar();
    lastRadar = now;
  }
}
