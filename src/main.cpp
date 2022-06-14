#include <Arduino.h>
#include <VL6180X.h>
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

#define TCAADDR 0x70
#define INTERRUPT_PIN 1

void processMPU();
void processEccentricEncoder();
void processSuspensionEncoder();
void processDistanceSensor();

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

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

VL6180X sensor;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  tcaselect(0);

  sensor.init();
  sensor.configureDefault();

  sensor.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 16);

  sensor.setTimeout(500);

  // stop continuous mode if already active
  sensor.stopContinuous();
  // in case stopContinuous() triggered a single-shot
  // measurement, wait for it to complete
  delay(300);
  // start interleaved continuous mode with period of 100 ms
  sensor.startRangeContinuous(30);

  tcaselect(2);
  encoder.begin();

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
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);  //divide by 16384 to get G's

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
}
float_t previousDistance;

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

void processDistanceSensor()
{
  // VL6180
  tcaselect(0);
  float_t currentDistance = sensor.readRangeContinuousMillimeters();
  currentDistance = EMA_function(0.8, currentDistance, previousDistance);
  Serial.print(currentDistance);
  if (sensor.timeoutOccurred())
  {
    Serial.println("Timeout");
  }
  previousDistance = currentDistance;

  Serial.print(",");
}

void processMPU()
{
  /*
  //----------------------------------------------------------------
  // MPU
  // if programming failed, don't try to do anything
  tcaselect(3);
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize)
    return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    */
  tcaselect(3);
  // if programming failed, don't try to do anything
  // if programming failed, don't try to do anything

  // MPU6050_DMP_FIFO_RATE_DIVISOR is set to 0x4 to prevent overflows in the file MPU6050_6Axis_MotionApps20.cpp
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
    Serial.print(aaWorld.z);
  }
}

/*************************************************************************************
  EMA Function (Simple Filter)
***********************************************************************************/
float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}

// 18ms loop time