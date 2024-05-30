#include <Wire.h>

int gyro_x, gyro_y, gyro_z;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
bool set_gyro_angles = false;

long acc_x, acc_y, acc_z, acc_total_vector;
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;
int display = 0;

#define DEBUG

void setup() {
  Wire.begin();
  setup_mpu_6050_registers();

  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    read_mpu_6050_data();

    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }

  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;

#ifdef DEBUG
  Serial.begin(115200);
#endif

  loop_timer = micros();
}

void loop() {
  read_mpu_6050_data();

  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  angle_pitch += gyro_x * 0.0000611;
  angle_roll += gyro_y * 0.0000611;

  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);

  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));
  angle_pitch_acc = asin((float) acc_y / acc_total_vector) * 57.296;
  angle_roll_acc = asin((float) acc_x / acc_total_vector) * -57.296;

  angle_pitch_acc -= 0.0;
  angle_roll_acc -= 0.0;

  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
  } else {
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;
  }

  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;

  display++;
  if (display == 50) {
    display = 0;

    if (angle_pitch_output < -2.01) {
      Serial.println(F("1 0 0 0 0"));
    } else if ((angle_pitch_output > -2.00) && (angle_pitch_output < -1.01)) {
      Serial.println(F("0 1 0 0 0"));
    } else if ((angle_pitch_output < 1.00) && (angle_pitch_output > -1.00)) {
      Serial.println(F("0 0 1 0 0"));
    } else if ((angle_pitch_output > 1.01) && (angle_pitch_output < 2.00)) {
      Serial.println(F("0 0 0 1 0"));
    } else if (angle_pitch_output > 2.01) {
      Serial.println(F("0 0 0 0 1"));
    }
  }

  while (micros() - loop_timer < 4000);
  loop_timer = micros();
} 

void setup_mpu_6050_registers() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}

void read_mpu_6050_data() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,14);
  while (Wire.available() < 14);

  acc_x = Wire.read()<<8|Wire.read();
  acc_y = Wire.read()<<8|Wire.read();
  acc_z = Wire.read()<<8|Wire.read();
  int temp = Wire.read()<<8|Wire.read();
  gyro_x = Wire.read()<<8|Wire.read();
  gyro_y = Wire.read()<<8|Wire.read();
  gyro_z = Wire.read()<<8|Wire.read();
}