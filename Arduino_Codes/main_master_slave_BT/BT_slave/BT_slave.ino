// Slave
#include <SoftwareSerial.h>
#include <Wire.h>

#define DEBUG
// #define MPU

// Pin used
const byte IN1 = 2;
const byte IN2 = 3;
const byte IN3 = 5;
const byte IN4 = 6;
// const byte echo = 7;
// const byte trig = 8;
const byte RX = 10;
const byte TX = 11;

SoftwareSerial BT(RX, TX);

#ifdef MPU
  long accelX, accelY, accelZ;
  float gForceX, gForceY, gForceZ;

  long gyroX, gyroY, gyroZ;
  float rotX, rotY, rotZ;
#endif

unsigned long previousMillis = 0;

String directions = "";
bool rcv_data = false;
unsigned long startTime = 0;
bool startRead = true;
const int oneSecInterval = 1000;
char currentPosition = 'T';

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // pinMode(echo, INPUT);
  // pinMode(trig, OUTPUT);
  Wire.begin();
  BT.begin(9600);
  #ifdef MPU
    setupMPU();
  #endif

  #ifdef DEBUG
    Serial.begin(9600);
  #endif
}

void loop() {
  while (BT.available() > 0) {
    char bt_data = char(BT.read());
    if (bt_data != '\n' and bt_data != '\r' and bt_data != 'E')
      directions += bt_data;
    rcv_data = true;

    if (startRead) {
      startRead = false;
      startTime = millis();
    }
  }

  unsigned outerCurrMillis = millis();
  if (rcv_data and outerCurrMillis - startTime >= oneSecInterval) {
    directions += 'E';
    #ifdef DEBUG
        Serial.println(directions);
    #endif
    directions.trim();

    bool nextState = true;
    unsigned long startMillis;
    const int interval = 1800;

    bool driving = false;
    unsigned long startMicros;
    const int interval_100micros = 100000;

    for (int index = 0; directions[index] != 'E';) {
      char nextMove = directions[index];

      if (nextState) {
        // Rotate for the next position
        if (currentPosition != nextMove) {
          if (currentPosition == 'B') {
            if (nextMove == 'R') {
              rotateToLeft();
            } else if (nextMove == 'L') {
              rotateToRight();
            } else if (nextMove == 'T') {
              rotateToRight();
              rotateToRight();
            }
          } else if (currentPosition == 'R') {
            if (nextMove == 'B') {
              rotateToRight();
            } else if (nextMove == 'L') {
              rotateToRight();
              rotateToRight();
            } else if (nextMove == 'T') {
              rotateToLeft();
            }
          } else if (currentPosition == 'L') {
            if (nextMove == 'B') {
              rotateToLeft();
            } else if (nextMove == 'R') {
              rotateToRight();
              rotateToRight();
            } else if (nextMove == 'T') {
              rotateToRight();
            }
          } else if (currentPosition == 'T') {
            if (nextMove == 'B') {
              rotateToRight();
              rotateToRight();
            } else if (nextMove == 'R') {
              rotateToRight();
            } else if (nextMove == 'L') {
              rotateToLeft();
            }
          }

          currentPosition = nextMove;
        }

        // if this is true, move forward
        if (currentPosition == nextMove) {
          driving = true;
          nextState = false;
          startMillis = millis();
          // startMicros = micros();
          moveForward();
        }
        /*
          if (directions[index] == 'T') {
            // Move Forward
            moveForward();
            nextState = false;
            startMillis = millis();
            startMicros = micros();
            driving = true;
          } else if (directions[index] == 'R') {
            // Rotate to right + Move Forward
            turnRight();
            delay(2300);
            nextState = false;
            moveForward();
            delay(1500);
            startMillis = millis();
            startMicros = micros();
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
            startMicros = micros();
            moveForward();
            driving = true;
          } 
        */
        /*
          // else if (directions[index] == 'B') {
          //   // Rotate to bottom + Move Forward
          //   turnLeft();
          //   delay(1000);
          //   nextState = false;
          //   startMillis = millis();
          // }
        */
      }

      #ifdef MPU
        if (driving) {
          unsigned long currentMicros = micros();
          if (currentMicros - startMicros >= interval_100micros) {
            startMicros = currentMicros;
            recordAccelRegisters();
            recordGyroRegisters();

            if (rotZ > 10.0) {
              // Rotate to right
              turnRight_analog();
            } else if (rotZ < -10.0) {
              // Rotate to left
              turnLeft_analog();
            }
          }
        }
      #endif

      unsigned long currentMillis = millis();
      if (currentMillis - startMillis >= interval) {  // Interval may adjuston
        stop();
        // delay(1000);
        startMillis = 0;
        startMicros = 0;
        index++;
        nextState = true;
        driving = false;
      }
    }

    startTime = 0;
    startRead = true;
    rcv_data = false;
    directions = "";
  }
}

void rotateToRight() {
  turnRight();
  delay(2100);
  moveForward();
  delay(1500);
}

void rotateToLeft() {
  turnLeft();
  delay(2300);
  moveForward();
  delay(1500);
}

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

void turnRight_analog() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 0);
  analogWrite(IN3, 120);
  analogWrite(IN4, 0);
}

void turnLeft() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 1);
}

void turnLeft_analog() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 120);
  analogWrite(IN3, 0);
  analogWrite(IN4, 255);
}

void stop() {
  digitalWrite(IN1, 0);
  digitalWrite(IN2, 0);
  digitalWrite(IN3, 0);
  digitalWrite(IN4, 0);
}

#ifdef MPU
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