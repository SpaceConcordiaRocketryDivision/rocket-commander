#include "PressureSensor.h"

PressureSensor::PressureSensor()
{
}
void PressureSensor::Init()
{
	Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
	seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
	if(!bmp.begin())
	{
	while(1);
	}
}
bool PressureSensor::GetData(float array[])
{
  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure)
  {
	float temperature = 25.0f;

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
bool PressureSensor::SendData(float pressureToSet)
{
  seaLevelPressure = pressureToSet;
} 