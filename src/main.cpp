#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050.h>
#include <NewPing.h>
#include <Wire.h>
#include <math.h>

// define motor pins
#define leftMotorPWMPin 5
#define leftMotorDirPin 4
#define rightMotorPWMPin 6
#define rightMotorDirPin 7

// define ultrasonic sensor pins
#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

// define PID values
#define Kp 40 * 1.3
#define Ki 20 * 1.6
#define Kd 0.05 * 1.1
#define sampleTime 0.005
#define targetAngle 1

// initialize sensors
MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile byte count = 0;
int distanceCm;

void init_PID();

void setMotors(int leftMotorSpeed, int rightMotorSpeed);

void setup() {

  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  mpu.initialize();
  mpu.setYAccelOffset(2521);
  mpu.setZAccelOffset(1545);
  mpu.setXGyroOffset(171);

  init_PID();
}

void loop() {

  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);

  // measure distance every 100 milliseconds
  if ((count % 20) == 0) {
    distanceCm = sonar.ping_cm();
  }

  if ((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower);
  }
}

void init_PID() {

  // initialize Timer1
  cli();      // disable global interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B

  // set compare match register to set sample time 5ms
  OCR1A = 9999;

  // turn on CTC mode
  TCCR1B |= (1 << WGM12);

  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);

  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); // enable global interrupts
}

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {

  rightMotorSpeed = -rightMotorSpeed; // make speed negative since motor connections are inverted

  if (leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  } else {
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }

  if (rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  } else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

// ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect) {

  // calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);

  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

  // calculate output from P, I and D values
  motorPower = Kp * (error) + Ki * (errorSum)*sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;

  prevAngle = currentAngle;

  // toggle the builtin LED every second
  count++;
  if (count == 200) {
    count = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}