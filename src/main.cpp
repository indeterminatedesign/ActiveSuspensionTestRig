#include <Arduino.h>
#include <DFRobot_VL6180X.h>
#include <Wire.h>
#include <AS5600.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

AS5600 encoder;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float_t acceleration;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

#define INTERRUPT_PIN 2
#define TCAADDR 0x70

uint8_t ret = 1;
volatile uint8_t flag = 0;

DFRobot_VL6180X VL6180X;

void processMPU();
void processEccentricEncoder();
void processSuspensionEncoder();
void processDistanceSensor();
void tcaselect(uint8_t i);

// ISR for VL6180X
void interrupt()
{
  if (flag == 0)
  {
    flag = 1;
  }
}

// ISR for MPU
void dmpDataReady()
{
  mpuInterrupt = true;
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

  //******************************************
  // MPU
  //******************************************
  tcaselect(3);
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read())
    ; // empty buffer
  while (!Serial.available())
    ; // wait for data
  while (Serial.available() && Serial.read())
    ; // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    mpu.setDLPFMode(MPU6050_DLPF_BW_188);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2); // divide by 16384 to get G's

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  //******************************************
  // Distance Sensor
  //******************************************
  tcaselect(0);

  while (!(VL6180X.begin()))
  {
    Serial.println("Please check that the IIC device is properly connected!");
    delay(1000);
  }
  VL6180X.setInterrupt(/*mode*/ VL6180X_HIGH_INTERRUPT);

  VL6180X.rangeConfigInterrupt(VL6180X_OUT_OF_WINDOW);

  /*Set the range measurement period*/
  VL6180X.rangeSetInterMeasurementPeriod(/* periodMs 0-25500ms */ 30);

  /*Set threshold value*/
  VL6180X.setRangeThresholdValue(/*thresholdL 0-255mm */ 40, /*thresholdH 0-255mm*/ 100);

#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_SAM_ZERO)
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN) /*Query the interrupt number of the D9 pin*/, interrupt, FALLING);
#else
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), interrupt, FALLING); // Enable the external interrupt 0, connect INT1/2 to the digital pin of the main control:
                                                                             // UNO(2), Mega2560(2), Leonardo(3), microbit(P0).
#endif

  /*Start continuous range measuring mode */
  VL6180X.rangeStartContinuousMode();
  delay(500);

  //******************************************
  // Encoders
  //******************************************
  tcaselect(1);

  encoder.begin();

}

void loop()
{
  processEccentricEncoder();
  processSuspensionEncoder();
  processDistanceSensor();
  processMPU();
  Serial.println();
}

void processEccentricEncoder()
{
  tcaselect(2);
  Serial.print(encoder.readAngle() * 0.0879);
  Serial.print(",");
}
void processSuspensionEncoder()
{
  tcaselect(1);
  Serial.print(encoder.readAngle() * 0.0879);
  Serial.print(",");
}

uint16_t distance;

void processDistanceSensor()
{
  tcaselect(0);
  if (flag == 1)
  {
    flag = 0;
    if (VL6180X.rangeGetInterruptStatus() == VL6180X_OUT_OF_WINDOW)
    {
      /*Get the measured distance data*/
      uint8_t range = VL6180X.rangeGetMeasurement();
      distance = range;
      /*Clear interrupts generated by measuring range*/
      VL6180X.clearRangeInterrupt();
    }
  }
  Serial.print(distance);
  Serial.print(",");
}

void processMPU()
{
  tcaselect(3);
  // !!!!!!!!!! MPU6050_DMP_FIFO_RATE_DIVISOR is set to 0x4 to prevent overflows in the file MPU6050_6Axis_MotionApps20.cpp
  if (!dmpReady)
    return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    // Get the Latest packet
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    // Serial.print("aworld\t");
    // Serial.print(aaWorld.x);
    // Serial.print("\t");
    // Serial.print(aaWorld.y);
    // Serial.print("\t");
    acceleration = (aaWorld.z) / 1671.83; // m/s2
  }
  Serial.print(acceleration);
}

/*************************************************************************************
  EMA Function (Simple Filter)
***********************************************************************************/
float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}

// 18ms loop time