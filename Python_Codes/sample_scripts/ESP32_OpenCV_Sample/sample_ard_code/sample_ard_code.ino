#include <Servo.h>

Servo servoMotor;
int servoPin = 9; // Modify this with the appropriate pin number
int Sflag = 0;

void setup()
{
    servoMotor.attach(servoPin);
    Serial.begin(9600);
    servoMotor.write(0); // Move servo to 0 degrees initially
    delay(1000);         // Delay for 1 second
}

void loop()
{
    if (Serial.available() > 0)
    {
        int angle = Serial.parseInt();

        if ((angle == 1) && (Sflag == 0))
        {
            servoMotor.write(90); // Move servo to 90 degrees
            Sflag = 1;
            delay(2000);
        }

        if ((angle == 2) && (Sflag == 1))
        {
            servoMotor.write(0);
            Sflag = 0;
            delay(2000);
        }
    }
}