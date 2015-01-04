#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#define GYROSCOPE_ARRAY_SIZE 4

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>

class Gyroscope
{
  public:
    Gyroscope();
	  boolean GetData(float array[]);
    void Init();
        
  private:
    Adafruit_L3GD20_Unified gyro;
    sensors_event_t event;
};
#endif
