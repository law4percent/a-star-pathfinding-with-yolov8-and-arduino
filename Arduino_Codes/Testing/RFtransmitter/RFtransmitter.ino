#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

RH_ASK driver;

void setup()
{
  pinMode(13, OUTPUT);
    Serial.begin(9600);    // Debugging only
    if (!driver.init())
         Serial.println("init failed");

  delay(5000);
}

void loop()
{
    const char *msg1 = "Hello-World0!";
    driver.send((uint8_t *)msg1, strlen(msg1));
    driver.waitPacketSent();
    Serial.println("Sent");
    digitalWrite(13, 1);
    delay(2000);
    digitalWrite(13, 0);
    delay(2000);
    
    const char *msg2 = "Hello-World1!";
    driver.send((uint8_t *)msg2, strlen(msg2));
    driver.waitPacketSent();
    Serial.println("Sent");
    digitalWrite(13, 1);
    delay(2000);
    digitalWrite(13, 0);
    delay(2000);
}