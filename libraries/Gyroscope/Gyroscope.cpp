#include "Gyroscope.h"

Gyroscope::Gyroscope()
{
  Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(00002);

}
void Gyroscope::Init()
{
  if(!gyro.begin())
  {
    while(1);
  }
}
boolean Gyroscope::GetData(float array[])
{
  sensors_event_t event;
  gyro.getEvent(&event);

  //TODO: event if
	array[0] = event.gyro.x;
	array[1] = event.gyro.y;
	array[2] = event.gyro.z;

  return 1;
}
