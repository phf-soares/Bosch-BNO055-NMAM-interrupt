#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define interrupt_pin 2
bool nm_interrupt;

const uint32_t BAUD_RATE = 921600;

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup(void)
{
  Serial.begin(BAUD_RATE);
  Serial.println("Acc Only Sensor Raw Data Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("No connection.");
    while (1);
  }
  nm_interrupt = false;
  pinMode(interrupt_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), nm_interrupt_callback, RISING);
  //bno.enableSlowNoMotion(5, 1, NO_MOTION);
  bno.enableAnyMotion(1, 0);
  bno.enableInterruptsOnXYZ(ENABLE, ENABLE, ENABLE);
  bno.setExtCrystalUse(true);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_ACCONLY);
}

void loop(void)
{
  if (nm_interrupt) {
    imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    /* Display the floating point data */
    Serial.print("-60.,"); //set lower scale
    Serial.print(acc.x());  //x acceleration
    Serial.print(",");
    Serial.print(acc.y()); //y accel
    Serial.print(",");
    Serial.print(acc.z()); //z accel
    Serial.println(",60.0"); //set upper scale
  }
  else
  {
    bno.resetInterrupts();
    nm_interrupt = false;
  }
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void nm_interrupt_callback(void)
{
  nm_interrupt = true;
}
