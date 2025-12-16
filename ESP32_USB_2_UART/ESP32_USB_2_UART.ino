#include <LD2412.h>

#define RXD2 16
#define TXD2 17
HardwareSerial mySerial(2);

void setup(){
  Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
}

void loop() {
  while (Serial.available()){
    mySerial.write(Serial.read());
  }

  while (mySerial.available()){
    Serial.write(mySerial.read());
  }
  delay(10);
}