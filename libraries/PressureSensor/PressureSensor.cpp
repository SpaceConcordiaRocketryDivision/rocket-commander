#include "PressureSensor.h"

PressureSensor::PressureSensor()
{
  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
  seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
}
void PressureSensor::Init()
{
  if(!bmp.begin())
  {
    while(1);
  }
}
boolean PressureSensor::GetData(float array[])
{
  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure)
  {
    float temperature;
    bmp.getTemperature(&temperature);
	array[0] = millis();
    array[1] = event.pressure;
    array[2] = temperature;
    array[3] = bmp.pressureToAltitude(seaLevelPressure,
                                        event.pressure,
                                        temperature);
    return 1; 
  }
  return 0;
}
boolean PressureSensor::SendData(float pressureToSet)
{
  seaLevelPressure = pressureToSet;
}
