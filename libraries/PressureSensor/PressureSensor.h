#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H
#define PRESSURE_ARRAY_SIZE 4

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

class PressureSensor {

public:
	PressureSensor();
	bool GetData (float array[]);
	bool SendData(float pressureToSet);
	bool Init();
        
private:
	Adafruit_BMP085_Unified bmp;
	sensors_event_t event;
	float seaLevelPressure;
};

#endif
