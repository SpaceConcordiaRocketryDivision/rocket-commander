
#ifndef GPSSENSOR_H
#define GPSSENSOR_H_G
#define GPS_ARRAY_SIZE 9

#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

class GpsSensor
{
	public:
		GpsSensor();
		boolean GetData(float[]);
        boolean SendData(float pressureToSet);
        void Init();
		void useInterrupt(boolean);

	private:
		SoftwareSerial mySerial;
		Adafruit_GPS GPS;
		boolean usingInterrupt;
		//sensors_event_t event;

        
        
};
#endif
