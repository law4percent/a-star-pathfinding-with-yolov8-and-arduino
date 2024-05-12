#ifndef myTouchPad_h
#define myTouchPad_h

#include "Arduino.h"

class myTouchPad {
private:
  byte pinUsed;
  bool lastReadDigitalValue = 0;
  bool btnState = false;

public:
  myTouchPad(const byte pinUsed) {
    this->pinUsed = pinUsed;
    pinMode(pinUsed, INPUT);
  }

  bool nowDigitalValue() {
    bool readPin = digitalRead(pinUsed);
    lastReadDigitalValue = readPin;
    return readPin;
  }

  bool lastDigitalValue() {
    return lastReadDigitalValue;
  }

  int NowAnalogValue() {
    return analogRead(pinUsed);
  }

  void setButtonStateTo(const bool btnState) {
    this->btnState = btnState;
  }

  bool isActive() {
    return btnState;
  }
};

#endif