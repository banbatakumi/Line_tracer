#include "Arduino.h"
#include "MPU6050_6Axis_MotionApps612.h"

#define LINE_AVERAGE_NUMBER 100
#define LINE_REACTION_VALUE 100
#define MOTOR_RC 0.25

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;   // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;   // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
VectorInt16 aa;   // [x, y, z]            accel sensor measurements
VectorInt16 gy;   // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;   // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;   // [x, y, z]            gravity vector
float euler[3];   // [psi, theta, phi]    Euler angle container
float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;   // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
      mpuInterrupt = true;
}

void imu_get();

int16_t yaw = 0;

const int motor_L_1 = 5;
const int motor_L_2 = 6;
const int motor_R_1 = 9;
const int motor_R_2 = 10;
const int line_1_pin = A7;
const int line_2_pin = A6;
const int line_3_pin = A0;
const int line_4_pin = A1;
const int line_5_pin = A3;
const int line_6_pin = A2;
const int led = 7;
const int button_pin = 3;

int l, r;

uint16_t line_value[6], pre_line_value[6];
uint16_t line_reaction[6];
int lap = 0;

bool line_tf[6];
bool button = 0;
bool main_move = 0;

void line_read();
void motor_move(int left, int right, int wait_time = 0);

void setup() {
      Serial.begin(9600);

      pinMode(motor_L_1, OUTPUT);
      pinMode(motor_L_2, OUTPUT);
      pinMode(motor_R_1, OUTPUT);
      pinMode(motor_R_2, OUTPUT);
      pinMode(led, OUTPUT);
      pinMode(button_pin, INPUT);

      for (int count = 0; count < 6; count++) {
            for (int count_1 = 0; count_1 < LINE_AVERAGE_NUMBER; count_1++) {
                  line_read();
                  line_reaction[count] += line_value[count];
            }
            line_reaction[count] /= LINE_AVERAGE_NUMBER;
            line_reaction[count] += LINE_REACTION_VALUE;
            digitalWrite(led, HIGH);
            delay(50);
            digitalWrite(led, LOW);
            delay(50);
      }

      digitalWrite(led, HIGH);

      // IMU
      //  join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000);   // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
#endif

      mpu.initialize();
      if (mpu.testConnection() != true) {
            return;   // 接続失敗
      }

      devStatus = mpu.dmpInitialize();

      if (devStatus != 0) {
            return;   // 初期化失敗
      }

      // supply your own gyro offsets here, scaled for min sensitivity
      mpu.setXGyroOffset(-29);
      mpu.setYGyroOffset(7);
      mpu.setZGyroOffset(-14);
      mpu.setXAccelOffset(-3412);
      mpu.setYAccelOffset(340);
      mpu.setZAccelOffset(556);

      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();

      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
}

void loop() {
      button = digitalRead(button_pin);
      if (button == 0) {
            main_move = 1 - main_move;
            delay(500);
            if (main_move == 1) {
                  for (int count = 0; count < 3; count++) {
                        digitalWrite(led, HIGH);
                        delay(100);
                        digitalWrite(led, LOW);
                        delay(400);
                  }
                  motor_move(200, 200, 200);
            }
      }

      line_read();
      // imu_get();

      if (main_move) {
            digitalWrite(led, HIGH);

            if (line_tf[1] && line_tf[2] && line_tf[3] && line_tf[0] && line_tf[5]) {
                  digitalWrite(led, LOW);
                  lap += 1;
                  delay(150);
            }

            if (lap == 3) {
                  motor_move(-200, -200, 300);
                  main_move = 0;
            }else{
                  if (line_tf[0] || line_tf[1] || (line_tf[2] && !line_tf[3])) l = 0;
                  if (line_tf[2]) r += 1;
                  if (line_tf[1]) r += 2;
                  if (line_tf[0]) r += 5;
                  if ((line_tf[3] && !line_tf[2]) || line_tf[4] || line_tf[5]) r = 0;
                  if (line_tf[3]) l += 1;
                  if (line_tf[4]) l += 2;
                  if (line_tf[5]) l += 5;

                  if (line_tf[2] && line_tf[3]) {
                        l = 190;
                        r = 190;
                  }

                  if (l > 230) l = 230;
                  if (r > 230) r = 230;
                  motor_move(-90 + l, -90 + r);
            }
      } else {
            digitalWrite(led, LOW);
            motor_move(0, 0);
            lap = 0;

            Serial.print(line_tf[0]);
            Serial.print(", ");
            Serial.print(line_tf[1]);
            Serial.print(", ");
            Serial.print(line_tf[2]);
            Serial.print(", ");
            Serial.print(line_tf[3]);
            Serial.print(", ");
            Serial.print(line_tf[4]);
            Serial.print(", ");
            Serial.println(line_tf[5]);
      }
}

void line_read() {
      line_value[0] = analogRead(line_1_pin);
      line_value[1] = analogRead(line_2_pin);
      line_value[2] = analogRead(line_3_pin);
      line_value[3] = analogRead(line_4_pin);
      line_value[4] = analogRead(line_5_pin);
      line_value[5] = analogRead(line_6_pin);
      for (int count = 0; count < 6; count++) {
            line_value[count] = (line_value[count] + pre_line_value[count]) / 2;
            pre_line_value[count] = line_value[count];
            line_tf[count] = 0;
            if (line_value[count] > line_reaction[count]) line_tf[count] = 1;
      }
}

void motor_move(int left, int right, int wait_time) {
      static int pre_left, pre_right;
      left = left * (1 - MOTOR_RC) + pre_left * MOTOR_RC;
      right = right * (1 - MOTOR_RC) + pre_right * MOTOR_RC;
      pre_left = left;
      pre_right = right;
      if (left > 0) {
            analogWrite(motor_L_1, left);
            analogWrite(motor_L_2, 0);
      } else {
            analogWrite(motor_L_1, 0);
            analogWrite(motor_L_2, left * -1);
      }
      if (right > 0) {
            analogWrite(motor_R_1, right);
            analogWrite(motor_R_2, 0);
      } else {
            analogWrite(motor_R_1, 0);
            analogWrite(motor_R_2, right * -1);
      }
      if (wait_time != 0) delay(wait_time);
}

void imu_get() {
      if (!dmpReady) return;
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {   // Get the Latest packet
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            yaw = ypr[0] * 180 / M_PI;
      }
}
