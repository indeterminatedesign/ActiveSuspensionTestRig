#include <Arduino.h>
#include <VL6180X.h>
#include <Wire.h>
#include <AS5600.h>

AS5600 encoder;

#define TCAADDR 0x70
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
}
float_t previousDistance;

void loop()
{
 
  tcaselect(0);
  float_t currentDistance = sensor.readRangeContinuousMillimeters();
  currentDistance = EMA_function(0.8, currentDistance, previousDistance);
  Serial.print(currentDistance*10);  //!!!!!!!!!!!!!!!!!Multiplied by 10x for plotting  
  if(sensor.timeoutOccurred())
  {
Serial.println("Timeout");
  }
  previousDistance = currentDistance;

  Serial.print(",");

  tcaselect(2);
  Serial.println(encoder.readAngle()*0.0879);
  delay(10);
  
}

/*************************************************************************************
  EMA Function (Simple Filter)
***********************************************************************************/
float EMA_function(float alpha, float latest, float stored)
{
  return (alpha * latest) + ((1 - alpha) * stored);
}

// 18ms loop time