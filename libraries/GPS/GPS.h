#include <Wire.h>
#include <Adafruit_GPS.h>

#ifndef GPS_H
#define GPS_H
#define GPS_ARRAY_SIZE 8

class GPS {

public:
	GPS();
	void init();
	bool getData (float array[]);
	bool sendData(float pressureToSet);

private:
	Adafruit_GPS gps;
	float timer;
	float latitude;
	float longitude;
	float fixquality;
	float satellites;
	float altitude;
	float speed;
};

#endif
