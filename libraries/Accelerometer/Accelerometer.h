#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#define ACCELEROMETER_ARRAY_SIZE 4

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

class Accelerometer {

public:
	Accelerometer();
	boolean GetData(float array[]);
	bool Init();

private:
	Adafruit_LSM303_Accel_Unified accel;
	sensors_event_t event;
};

#endif
