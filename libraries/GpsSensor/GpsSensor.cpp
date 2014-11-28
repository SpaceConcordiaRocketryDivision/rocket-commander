#include "GpsSensor.h"

GpsSensor::GpsSensor()
{
}
void GpsSensor::Init()
{
	if(!gps.begin())
		while(1);
}
boolean GpsSensor::GetData(float array[])
{
  sensors_event_t event;
  gps.GetEvent(&event);

  array[0] = millis();
  array[1] = timer;
  array[2] = latitude;
  array[3] = longitude;
  array[4] = fixquality;
  array[5] = satellites;
  array[6] = altitude;
  array[7] = speed;

  return 1;
}
boolean GpsSensor::SendData(float pressureToSet)
{
  return 0;
}
