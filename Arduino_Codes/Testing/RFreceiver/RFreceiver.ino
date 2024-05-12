#include <RH_ASK.h>
#include <SPI.h>  // Not actualy used but needed to compile

RH_ASK driver;

void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);  // Debugging only
  if (!driver.init())
    Serial.println("init failed");
}

void loop() {
  uint8_t buf[12];
  uint8_t buflen = sizeof(buf);
  if (driver.recv(buf, &buflen)) {
    String data = cleanString((char*)buf);
    Serial.println("Message: " + data);

    if (data.equals("Hello-World0")) {
      digitalWrite(13, 0);
    } else if (data.equals("Hello-World1")) {
      digitalWrite(13, 1);
    }
  }

  Serial.println(F("Waiting"));
}

String cleanString(const char* data) {
  String cleanData = "";
  for (int i = 0; data[i] != '!'; i++) {
    cleanData += data[i];
  }
  return cleanData;
}