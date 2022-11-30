#include <Arduino.h>
#include <DFRobot_VL6180X.h>
#include <Wire.h>
#include <AS5600.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <WiFi.h>

AS5600 encoder;
Servo servo;

MPU6050 mpu;

float_t acceleration;
bool runBegan = false;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

#define MOTOR_PIN 32
#define SERVO_PIN 33
#define INTERRUPT_PIN 2
#define TCAADDR 0x70

uint8_t ret = 1;
volatile uint8_t flag = 0;

uint32_t previousSampleTime = 0;
uint32_t delayForMotorTriggerTime = 0;
const uint16_t sampleInterval = 10000;
const uint32_t delayForMotor = 4000000; // Gives motor RPM time to settle before starting run

const int freq = 30000;
const int pwmChannel = 10;
const int resolution = 8;
uint16_t dutyCycle = 150;
uint16_t targetRPM = 150;

double_t previousAngle = 0;
// DFRobot_VL6180X VL6180X;

// Motor PID Settings
const float_t kP = .023;
const float_t kI = .000000001;
const float_t kD = .075;

void processMPU();
float_t processEccentricEncoder();
float_t processSuspensionArmEncoder();
float_t processSprungMassEncoder();
void processDistanceSensor();
void selectEncoder(uint8_t i);
float_t calculateDistance(uint16_t angle);
void moveSuspensionDistance(float_t distance);
uint16_t calculateRPM(uint32_t timeElapsed, double_t currentAngle);
void followRoller(float_t currentAngle);
void controlMotorSpeed(uint16_t targetRPM, uint16_t currentRPM, uint32_t dt);
void receiveConfigParameters();

// ISR for VL6180X
void interrupt()
{
  if (flag == 0)
  {
    flag = 1;
  }
}

float EMA_function(float alpha, float latest, float stored);

void selectEncoder(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  WiFi.mode(WIFI_OFF);
  Serial.begin(500000);
  Wire.begin();
  Wire.setClock(400000);

  //******************************************
  // Servo
  //******************************************
  //servo.setPeriodHertz(330);
  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(1450);

  /*
    //******************************************
    // MPU
    //******************************************
    tcaselect(3);
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    mpu.CalibrateAccel();
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setDLPFMode(MPU6050_DLPF_BW_20);

    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    */

  //******************************************
  // Motor PWM
  //******************************************
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_PIN, pwmChannel);

  //******************************************
  // Distance Sensor
  //******************************************
  // tcaselect(0);

  //  while (!(VL6180X.begin()))
  // {
  //    Serial.println("Please check that the IIC device is properly connected!");
  //   delay(1000);
  // }
  // VL6180X.setInterrupt(/*mode*/ VL6180X_HIGH_INTERRUPT);

  // VL6180X.rangeConfigInterrupt(VL6180X_OUT_OF_WINDOW);

  /*Set the range measurement period*/
  // VL6180X.rangeSetInterMeasurementPeriod(/* periodMs 0-25500ms */ 30);

  /*Set threshold value*/
  // VL6180X.setRangeThresholdValue(/*thresholdL 0-255mm */ 40, /*thresholdH 0-255mm*/ 100);

  //#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_SAM_ZERO)
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN) , interrupt, FALLING);
  //#else
  // attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, FALLING); // Enable the external interrupt 0, connect INT1/2 to the digital pin of the main control:
  // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
  //#endif

  /*Start continuous range measuring mode */
  // VL6180X.rangeStartContinuousMode();
  // delay(500);

  //******************************************
  // Encoders
  //******************************************
  selectEncoder(1);

  // Take in Parameters from Serial
  receiveConfigParameters();

  // Temp fix due to PID tuning issues
  // ledcWrite(pwmChannel, targetRPM);
}

void loop()
{
  uint32_t timeElapsed = micros() - previousSampleTime;

  if (delayForMotorTriggerTime > 0 && micros() - delayForMotorTriggerTime > delayForMotor)
  {
    runBegan = true;
  }

  if (timeElapsed > sampleInterval)
  {
    float_t currentAngle = processEccentricEncoder();
    uint16_t rpm = calculateRPM(timeElapsed, currentAngle);
    controlMotorSpeed(targetRPM, rpm, timeElapsed);

    if (runBegan)
    {
      Serial.print(currentAngle);
      Serial.print(",");
      Serial.print(rpm);
      Serial.print(",");
      Serial.print(processSuspensionArmEncoder());
      Serial.print(",");
      Serial.print(processSprungMassEncoder());
      Serial.print(",");
      // processDistanceSensor();
      // processMPU();
       followRoller(currentAngle);

      Serial.print(timeElapsed);
      Serial.print(",");

      Serial.println();
    }
    previousSampleTime = micros();
  }
}

float_t processEccentricEncoder()
{
  selectEncoder(2);
  float_t angle = encoder.readAngle() * 0.0879;
  return angle;
}
float_t processSuspensionArmEncoder()
{
  selectEncoder(1);
  float_t angle = encoder.readAngle() * 0.0879;
  return angle;
}
float_t processSprungMassEncoder()
{
  selectEncoder(0);
  float_t angle = encoder.readAngle() * 0.0879;
  return angle;
}

uint16_t calculateRPM(uint32_t timeElapsed, double_t currentAngle)
{
  static uint16_t previousRPM;

  if (fabs(currentAngle - previousAngle) < 0.8)
  {
    return 0;
  }

  /*
  Serial.println("CurrentAngle, Previous Angle, Time Elapsed");
  Serial.print(currentAngle);
  Serial.print(",");
  Serial.print(previousAngle);
  Serial.print(",");
  Serial.print(timeElapsed);

  Serial.println("");
*/
  // Need to check for exceeding 360 as a roll over
  double_t tempAngle = currentAngle;
  if (tempAngle > previousAngle)
  {
    tempAngle -= 360;
  }
  /*
  Serial.print("Delta T");
  Serial.print(((double)timeElapsed /1.00e06),6);
  Serial.print(",");
  Serial.print("Part of Rotation");
  Serial.print(.1667 *(currentAngle - previousAngle),6);
  Serial.print(",");
  */

  float_t rpm = -.1667 * (tempAngle - previousAngle) / ((double)timeElapsed / 1.00e06);
  rpm = EMA_function(0.6, rpm, previousRPM);
  previousRPM = rpm;
  previousAngle = currentAngle;
  return rpm;
}

void controlMotorSpeed(uint16_t targetRPM, uint16_t currentRPM, uint32_t dt)
{
  static int32_t cumulativeError;
  static int32_t previousError;
  float rateError;
  int32_t error;
  static int16_t output;

  error = targetRPM - currentRPM;

  rateError = (float)(error - previousError) / dt;

  cumulativeError += error * dt;
  float_t kIComponent = kI * cumulativeError;
  kIComponent = constrain(kIComponent, -50, 50);

  float_t correction = kP * error + kIComponent + kD * rateError;

  output += correction;
  // Make sure the ESC doesn't recieve a signal it can't use and cap the upper limit
  output = constrain(output, 0, 255);
#ifdef DEBUG
  Serial.print("Mapped Output: ");
  Serial.println(outputESC);

  Serial.print("Current RPM: ");
  Serial.println(currentRPM);
#endif
  // TODO:  Fix PID Calc
  ledcWrite(pwmChannel, output);

  previousError = error;
}

uint16_t distance;
/*
void processDistanceSensor()
{
  tcaselect(0);
  if (flag == 1)
  {
    flag = 0;
    if (VL6180X.rangeGetInterruptStatus() == VL6180X_OUT_OF_WINDOW)
    {
      uint8_t range = VL6180X.rangeGetMeasurement();
      distance = range;
         VL6180X.clearRangeInterrupt();
    }
  }
  Serial.print(distance);
  Serial.print(",");
}
*/
float_t previousAcceleration;
void processMPU()
{
  selectEncoder(3);
  acceleration = (mpu.getAccelerationZ() / 1671.83) - 9.8; // m/s2
  acceleration = EMA_function(0.4, acceleration, previousAcceleration);
  Serial.print(acceleration);
  previousAcceleration = acceleration;
}

/*************************************************************************************
  EMA Function (Simple Filter)
***********************************************************************************/
float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}

// 18ms loop time

float_t calculateRollerDistance(uint16_t angle)
{
  const uint16_t offsetAngle = 215; // 145 base offset plus some lag angle
  return sin((angle - offsetAngle) * 0.0174533) * 5;
}

void moveSuspensionDistance(float_t distance)
{
  const float_t armRatio = 1.83;  //Ratio between control arm total length and the of the pivot arm and where it is actuated
  float_t servoToControlArmDistance = distance / armRatio;
  const float_t servoArmPivotLenght = 20;  //Distance from center to pivot point on servo arm itself

  float_t servoAngle = atan(servoToControlArmDistance / servoArmPivotLenght);  // In radians
  int16_t microseconds = 318.33 * servoAngle + 1450; // Converte angle in radians to microseconds for servo
  servo.writeMicroseconds(microseconds);
  Serial.print(microseconds);
  Serial.print(",");
}

void followRoller(float_t currentAngle)
{
  float_t distance = calculateRollerDistance(currentAngle);
  moveSuspensionDistance(distance);
}

const byte numChars = 32;
char receivedChars[numChars];

void receiveConfigParameters()
{
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (true)
  {
    if (Serial.available() > 0)
    {
      rc = Serial.read();

      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        ndx = 0;
        break;
      }
    }
  }

  // Convert character array into an integer for RPM
  targetRPM = atoi(receivedChars);
  // Start the timer to wait for motor to reach target speed
  delayForMotorTriggerTime = micros();

  Serial.print("Target RPM: ");
  Serial.println(targetRPM);
}