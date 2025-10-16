#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "imumaths.h"

#define BNO055_SAMPLERATE_DELAY_MS (10)
#define interrupt_pin 2
bool nm_interrupt;

const uint32_t BAUD_RATE = 921600;
const uint8_t ACC_1000HZ_2G = 0x1C;

Adafruit_BNO055 bno = Adafruit_BNO055();

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data)  // write one BNO register
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);  // send stop
}

void setup(void)
{
  Serial.begin(BAUD_RATE);
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_ACCONLY);  
  if (!bno.begin())
  {
    Serial.print("No connection.");
    while (1);
  }
  bno.setAcelerometerConfig(ACC_1000HZ_2G);  
  nm_interrupt = false;
  pinMode(interrupt_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interrupt_pin), nm_interrupt_callback, RISING);   
  bno.enableAnyMotion(0, 0);  
  bno.enableInterruptsOnXYZ(ENABLE, ENABLE, ENABLE);
  bno.setExtCrystalUse(true);    
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
