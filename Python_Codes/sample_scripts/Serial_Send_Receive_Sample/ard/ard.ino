// Slave
#include <SoftwareSerial.h>

SoftwareSerial BT(10, 11);

void setup() {
  BT.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  if (BT.available() > 0) {
    char data = char(BT.read());

    Serial.println(data);
    if (data == '1') {
      digitalWrite(13, 1);
    } else if (data == '0') {
      digitalWrite(13, 0);
    }
  }
}
