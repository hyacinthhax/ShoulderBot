/*
  Robotic Arm Self Balance Camera(Adafruit PCA9685 + MPU6050)
  by Hyacinthhax
  Based on Gimbal Project by Dejan, www.HowToMechatronics.com
  Code based on the MPU6050_DMP6 example from the i2cdevlib library by Jeff Rowberg:
  https://github.com/jrowberg/i2cdevlib
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define SERVOMIN  150 // Minimum pulse length
#define SERVOMAX  600 // Maximum pulse length
#define SERVO_FREQ 50 // PWM frequency for servos

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // Assuming the PCA9685 is at address 0x40

MPU6050 mpu;
volatile bool dmpReady = false;
uint16_t packetSize;
volatile bool mpuInterrupt = false;
uint16_t fifoCount;
uint8_t mpuIntStatus;
uint8_t fifoBuffer[64]; // FIFO storage buffer


uint8_t devStatus;
void dmpDataReady() {
    // This function should be empty because it's meant to be an interrupt service routine.
    // It's called automatically when the MPU6050 generates a data-ready interrupt.
    // You can add code here to handle the interrupt, if needed.
}

void setup() {
  Serial.begin(38400);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  #define INTERRUPT_PIN 2 // Example pin number for interrupt from MPU6050
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(17);
  mpu.setYGyroOffset(-69);
  mpu.setZGyroOffset(27);
  mpu.setZAccelOffset(1551);

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  // Define the pins to which the 5 servo motors are connected
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Set the PWM frequency for the servos
}

void loop() {
  if (!dmpReady) return;

  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      fifoCount = mpu.getFIFOCount();
    }
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    ypr[0] = ypr[0] * 180 / M_PI;
    ypr[1] = ypr[1] * 180 / M_PI;
    ypr[2] = ypr[2] * 180 / M_PI;

    if (j <= 300) {
      correct = ypr[0];
      j++;
    } else {
      ypr[0] = ypr[0] - correct;
      
      // Map the values of the MPU6050 sensor from -90 to 90 to values suitable for the servo control from 0 to 180
      int servo0Value = map(ypr[0], -90, 90, 0, 180); // Servo 0 - Y plane (Pitch)
      int servo1Value = map(ypr[1], -90, 90, 0, 180); // Servo 1 - X plane (Yaw)
      int servo2Value = map(ypr[0], -90, 90, 0, 180); // Servo 2 - Oppositional to Servo 0 (Pitch)
      int servo3Value = map(ypr[2], -90, 90, 0, 180); // Servo 3 - Rotational plane of Y (Roll)
      int servo4Value = map(ypr[1], -90, 90, 180, 0); // Servo 4 - Opposite of Servo 1 on X plane (Yaw)

      // Control the servos according to the MPU6050 orientation
      pwm.setPWM(0, 0, servo0Value); // Set pulse for servo 0
      pwm.setPWM(1, 0, servo1Value); // Set pulse for servo 1
      pwm.setPWM(2, 0, servo2Value); // Set pulse for servo 2
      pwm.setPWM(3, 0, servo3Value); // Set pulse for servo 3
      pwm.setPWM(4, 0, servo4Value); // Set pulse for servo 4
    }
#endif
  }
}
