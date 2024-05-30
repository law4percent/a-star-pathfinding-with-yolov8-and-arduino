// Slave
#include <SoftwareSerial.h>
#include <Wire.h>

// #define DEBUG

// Pin used
const byte IN1 = 3;
const byte IN2 = 4;
const byte IN3 = 5;
const byte IN4 = 6;
const byte echo = 7;
const byte trig = 8;
const byte RX = 10;
const byte TX = 11;

SoftwareSerial BT(RX, TX);

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

unsigned long previousMillis = 0;

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(trig, OUTPUT);
  Wire.begin();
  setupMPU();
  BT.begin(9600);
  
  #ifdef DEBUG
    Serial.begin(9600);
  #endif
}
void loop() {
  String directions = "";
  bool rcv_data = false;

  while (BT.available() > 0) {
    char bt_data = char(BT.read());
    if (bt_data != '\n' and bt_data != '\r')
      directions += bt_data;
    rcv_data = true;
  }

  #ifdef DEBUG
    Serial.println(directions);
  #endif

  if (rcv_data) {
    directions.trim();
    unsigned long startMillis;
    const int interval = 1800;
    const int interval_100micros = 100000;
    unsigned long prevMicros = 0;
    bool nextState = true;
    bool driving = false;

    for (int index = 0; directions[index] != 'E';) {
      if (nextState) {
        if (directions[index] == 'T') {
          // Move Forward
          moveForward();
          nextState = false;
          startMillis = millis();
          driving = true;
        } else if (directions[index] == 'R') {
          // Rotate to right + Move Forward
          turnRight();
          delay(2300);
          nextState = false;
          moveForward();
          delay(1500);
          startMillis = millis();
          moveForward();
          driving = true;
        } else if (directions[index] == 'L') {
          // Rotate to left + Move Forward
          turnLeft();
          delay(2300);
          nextState = false;
          moveForward();
          delay(1500);
          startMillis = millis();
          moveForward();
          driving = true;
        } 
        /*
          else if (directions[index] == 'B') {
            // Rotate to bottom + Move Forward
            turnLeft();
            delay(1000);
            nextState = false;
            startMillis = millis();
          }
        */
      }

      if (driving) {
        unsigned long currentMicros = micros();
        if (currentMicros - prevMicros >= interval_100micros) {
          prevMicros = currentMicros;
          recordAccelRegisters();
          recordGyroRegisters();
          
          if (rotZ > 50.0) {
            // Rotate to right
          } else if (rotZ < -50.0) {
            // Rotate to left
          }
        }
      }

      unsigned long currentMillis = millis();
      if (currentMillis - startMillis >= interval) {  // Interval may adjuston
        stop();
        index++;
        nextState = true;
        driving = false;
      }
    } // for-loop
  } // outer if
} // loop()


void moveForward() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 1);
  digitalWrite(IN3, 1);
  digitalWrite(IN4, 0);
}

void moveBackward() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}

void turnRight() {
  digitalWrite(IN1, 1);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

void turnLeft() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}

void stop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

void setupMPU() {
  Wire.beginTransmission(0b1101000);  //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);                   //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000);             //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1B);                   //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x00000000);             //Setting the gyro to full scale +/- 250deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x1C);                   //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0b00000000);             //Setting the accel to +/- 2g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x3B);                   //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //Request Accel Registers (3B - 40)
  while (Wire.available() < 6)
    ;
  accelX = Wire.read() << 8 | Wire.read();  //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read();  //Store last two bytes into accelZ
  processAccelData();
}

void processAccelData() {
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0;
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x43);                   //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6)
    ;
  gyroX = Wire.read() << 8 | Wire.read();  //Store first two bytes into accelX
  gyroY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into accelY
  gyroZ = Wire.read() << 8 | Wire.read();  //Store last two bytes into accelZ
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

#ifdef DEBUG
  void printData() {
    Serial.print("Gyro (deg)");
    Serial.print(" X=");
    Serial.print(rotX);
    Serial.print(" Y=");
    Serial.print(rotY);
    Serial.print(" Z=");
    Serial.println(rotZ);
    Serial.print(" Accel (g)");
    Serial.print(" X=");
    Serial.print(gForceX);
    Serial.print(" Y=");
    Serial.print(gForceY);
    Serial.print(" Z=");
    Serial.println(gForceZ);
  }
#endif
/*
  long ultrasonic() {
    unsigned long currentMillis = millis();            //time in milliseconds from which the code was started
    if (currentMillis - previousMillis >= interval) {  //check "blink without delay" code
      previousMillis = currentMillis;
      if (trigState == LOW) {
        (trigState = HIGH);
      } else {
        (trigState = LOW);
      }
    }
    digitalWrite(trigPin, trigState);
    long duration = pulseIn(echoPin, HIGH);
    return (duration / 2) / 29.1;
  }
*/