
#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H
#define PRESSURE_ARRAY_SIZE 3

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>


class PressureSensor
{
	public:
          PressureSensor();
	  boolean getData (float array[]);
          boolean sendData(float pressureToSet);
          void init();
        
        private:
          Adafruit_BMP085_Unified bmp;
          sensors_event_t event;
          float seaLevelPressure;
};
#endif
