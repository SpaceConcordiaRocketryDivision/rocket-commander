
#ifndef GPSSENSOR_H
#define GPSSENSOR_H_G
#define GPS_ARRAY_SIZE 8

#include <Wire.h>
#include <Adafruit_GPS.h>

class GpsSensor
{
	public:
		GpsSensor();
		boolean GetData (float array[]);
        boolean SendData(float pressureToSet);
        void Init();

	private:

		sensors_event_t event;

        
        
};
#endif
