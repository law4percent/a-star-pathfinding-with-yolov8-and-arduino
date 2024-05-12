#include <RH_ASK.h>
#include <SPI.h>

RH_ASK RFsender;

void setup() {
  Serial.begin(9600);
  if (!RFsender.init()) {
    Serial.println("Init Failed");
  }
}

void loop() {
  const char *msg = "Hello World!";
  RFsender.send((uint8_t *)msg, strlen(msg));
  RFsender.waitPacketSent();
  delay(1000);
}