// Master
#include <SoftwareSerial.h>

const byte RX = 10;
const byte TX = 11;
SoftwareSerial BT(RX, TX);

void setup() {
  BT.begin(9600);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  String directions = "";
  bool rcv_data = false;

  while (Serial.available() > 0) {
    char serial_data = char(Serial.read());
    if (serial_data != '\n' and serial_data != '\r')
      directions += serial_data;
    rcv_data = true;
  }

  if (rcv_data) {
    directions.trim();
    directions += "E";
    BT.println(directions);
  }
}
