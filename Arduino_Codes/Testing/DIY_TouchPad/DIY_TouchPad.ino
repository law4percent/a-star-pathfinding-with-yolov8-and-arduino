#include "myTouchPad.h"
#include <RH_ASK.h>
#include <SPI.h>

myTouchPad RequestTouchPad(A1);
myTouchPad CancelTouchPad(A2);
RH_ASK RFmodule;

unsigned long previousMillis_1sec = 0;
const long interval_1sec = 1000;
byte fiveSecHold = 5;
bool startCount = false;

const char* msg_rqst = "rqst_A!";
const char* msg_cncl = "cncl_A!";


// #define SERIAL_DEBUG // Uncomment this line to debug the code

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
#endif

  if (!RFmodule.init())
    Serial.println("init failed");
}

void loop() {
  unsigned long currentMillis = millis();
  bool requestTP_State = RequestTouchPad.nowDigitalValue();

  if (currentMillis - previousMillis_1sec >= interval_1sec) {
    if (requestTP_State and fiveSecHold == 5) {
      startCount = true;
    }

    if (requestTP_State and startCount) {
      fiveSecHold--;
    } else if (!requestTP_State and startCount) {
      ResetData();
    }

    if (fiveSecHold <= 0) {
      // Request to the camera and wait for the confirmation
      RequestTouchPad.setButtonStateTo(true);
      ResetData();
    }
    previousMillis_1sec = currentMillis;
  }

  if (RequestTouchPad.isActive() and CancelTouchPad.nowDigitalValue()) {
    // Request to the camera and wait for the confirmation
  }

#ifdef SERIAL_DEBUG
    Serial.println("TouchPadA: " + String(RequestTouchPad.digitalValue()));
  Serial.println("TouchPadB: " + String(CancelTouchPad.digitalValue()) + "\n");
  delay(1000);
#endif
}

void ResetData() {
  fiveSecHold = 5;
  startCount = false;
}

void WaitForReply() {
  while (true) {
    // sent many times until get reply
    RFmodule.send((uint8_t *)msg_rqst, strlen(msg_rqst));
    RFmodule.waitPacketSent();
    delay(500);

    uint8_t buf[5];
    uint8_t buflen = sizeof(buf);

    if (RFmodule.recv(buf, &buflen)) {
      String data = cleanString((char*)buf);

      if (data.equals("ok_A")) {
        
      }
    }
  }
}

String cleanString(const char* data) {
  String cleanData = "";
  for (int i = 0; data[i] != '!'; i++) {
    cleanData += data[i];
  }
  return cleanData;
}