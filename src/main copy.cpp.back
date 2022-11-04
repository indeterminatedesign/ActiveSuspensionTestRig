#include <Arduino.h>
#include <DFRobot_VL6180X.h>
#include <Wire.h>
#include <AS5600.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

AS5600 encoder;
Servo servo;

MPU6050 mpu;

float_t acceleration;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

#define MOTOR_PIN 32
#define INTERRUPT_PIN 2
#define TCAADDR 0x70

uint8_t ret = 1;
volatile uint8_t flag = 0;

uint32_t previousTime = 0;
uint16_t sampleInterval = 10000;

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
uint16_t dutyCycle = 150;
uint16_t targetRPM = 150;

double_t previousAngle = 0;
// DFRobot_VL6180X VL6180X;

// Motor PID Settings
const float_t kP = .5;
const float_t kI = 0.0;
const float_t kD = 0.3;

void processMPU();
float_t processEccentricEncoder();
float_t processSuspensionEncoder();
float_t processSprungMassEncoder();
void processDistanceSensor();
void tcaselect(uint8_t i);
float_t calculateDistance(uint16_t angle);
void moveSuspensionDistance(float_t distance);
uint16_t calculateRPM(uint32_t timeElapsed, double_t currentAngle);
void followRoller();
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

void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(500000);
  Wire.begin();
  Wire.setClock(400000);

  /*
  //******************************************
  // Servo
  //******************************************
  servo.attach(0);
  servo.writeMicroseconds(1500);

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
  pinMode(MOTOR_PIN, OUTPUT);
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(MOTOR_PIN, pwmChannel);

  ledcWrite(pwmChannel, dutyCycle);

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
  tcaselect(1);

  // Take in Parameters from Serial
  receiveConfigParameters();
}

void loop()
{
  uint32_t timeElapsed = micros() - previousTime;
  if (timeElapsed > sampleInterval)
  {
    float_t currentAngle = processEccentricEncoder();
    uint16_t rpm = calculateRPM(timeElapsed, currentAngle);
    // followRoller();
    controlMotorSpeed(100, rpm, timeElapsed);
    processSuspensionEncoder();
    processSprungMassEncoder();
    // processDistanceSensor();
    // processMPU();

    Serial.print(timeElapsed);
    Serial.print(",");

    Serial.println();

    previousTime = micros();
  }
}

float_t processEccentricEncoder()
{
  tcaselect(2);
  float_t angle = encoder.readAngle() * 0.0879;
  Serial.print(angle);
  Serial.print(",");
  return angle;
}
float_t processSuspensionEncoder()
{
  tcaselect(1);
  float_t angle = encoder.readAngle() * 0.0879;
  Serial.print(angle);
  Serial.print(",");
  return angle;
}
float_t processSprungMassEncoder()
{
  tcaselect(0);
  float_t angle = encoder.readAngle() * 0.0879;
  Serial.print(angle);
  Serial.print(",");
  return angle;
}

uint16_t calculateRPM(uint32_t timeElapsed, double_t currentAngle)
{
  static uint16_t previousRPM;
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
  rpm = EMA_function(0.8, rpm, previousRPM);
  previousRPM = rpm;
  previousAngle = currentAngle;
  Serial.print(rpm);
  Serial.print(",");
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

  // Constrain that integral
  kIComponent = constrain(kIComponent, -150, 150);

  int16_t correction = kP * error + kIComponent + kD * rateError;

  output += correction;

  // Make sure the ESC doesn't recieve a signal it can't use and cap the upper limit
  output = constrain(output, 0, 255);
#ifdef DEBUG
  Serial.print("Mapped Output: ");
  Serial.println(outputESC);

  Serial.print("Current RPM: ");
  Serial.println(currentRPM);
#endif

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
  tcaselect(3);
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
  const float_t armRatio = 1.69;
  float_t servoArmDistance = distance / armRatio;

  float_t servoAngle = atan(servoArmDistance / 25);  // In radians
  int16_t microseconds = 318.33 * servoAngle + 1450; // Converte angle in radians to microseconds for servo
  servo.writeMicroseconds(microseconds);
  Serial.print(microseconds);
  Serial.print(",");
}

void followRoller()
{
  tcaselect(2);
  float_t currentAngle = encoder.readAngle() * 0.0879;
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

  Serial.print("Target RPM: ");
  Serial.println(targetRPM);
}