#include "PressureSensor.h"

PressureSensor::PressureSensor()
{
  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
  seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
}
void PressureSensor::init()
{
  if(!bmp.begin())
  {
    while(1);
  }
  bmp.getEvent(&event); 
}
boolean PressureSensor::getData(float array[])
{
  if (event.pressure)
  {
    float temperature;
    bmp.getTemperature(&temperature);
    array[0] = event.pressure;
    array[1] = temperature;
    array[2] = bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature);
    return 1; 
  }
  return 0;
}
boolean PressureSensor::sendData(float pressureToSet)
{
  seaLevelPressure = pressureToSet;
}
