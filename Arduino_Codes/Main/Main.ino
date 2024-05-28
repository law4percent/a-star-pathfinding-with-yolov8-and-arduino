#include <SoftwareSerial.h>

SoftwareSeraial BT;
const byte EN1 = 2;
const byte EN2 = 3;
const byte EN3 = 4;
const byte EN4 = 5;

void setup() {
  Serial.begin(9600);
  BT.begin(115200);

  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(EN3, OUTPUT);
  pinMode(EN4, OUTPUT);
}

void loop() {
  String receiveString = "";
  bool BTreading = false;
  int stringLength = 0;

  while (BT.available > 0) {
    BTreading = true;
    char BTdata = BT.read();
    if (BTdata != '\n' and BTdata != '\r')
      receiveString += BTdata; 
  }

  if (BTreading) {
    receiveString.trim();
    stringLength = receiveString.length();
  }

  int count = 0;
  while (count < stringLength) {
    unsigned long currentMillis = millis();
    /*
      * Bug:
      *   1. Indentify its current position.
      *   2. Adjust the current position, depending on the next move.
      *   3. Make a move with millis.
      *   4. If there an interruption, like obstacles, during on its way, send a signal ('C') to Camera to create a new path and break this while-loop.
      *   5. Repeat.
      *
    */
    switch(receiveString[count]) {
      case 'T':
        MoveTop();
        break;

      case 'R':
        MoveRight();
        break;

      case 'L':
        MoveLeft();
        break;

      case 'B':
        MoveBottom();
        break;

      default:
        Stop();
        break;
    }
  }
}

void MoveForward() {
  digitalRead(EN1, 1);
  digitalRead(EN2, 0);

  digitalRead(EN3, 0);
  digitalRead(EN4, 1);
}

void MoveTop() {
  digitalRead(EN1, 1);
  digitalRead(EN2, 0);

  digitalRead(EN3, 0);
  digitalRead(EN4, 1);
}

void MoveRight() {
  digitalRead(EN1, 1);
  digitalRead(EN2, 0);

  digitalRead(EN3, 1);
  digitalRead(EN4, 0);
}

void MoveLeft() {
  digitalRead(EN1, 0);
  digitalRead(EN2, 1);

  digitalRead(EN3, 0);
  digitalRead(EN4, 1);
}

void MoveBottom() {
  digitalRead(EN1, 0);
  digitalRead(EN2, 1);

  digitalRead(EN3, 1);
  digitalRead(EN4, 0);
}

void Stop() {
  digitalRead(EN1, 0);
  digitalRead(EN2, 0);

  digitalRead(EN3, 0);
  digitalRead(EN4, 0);
}