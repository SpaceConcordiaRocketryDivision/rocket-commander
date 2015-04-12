
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
		bool GetData(float[]);
        bool SendData(float pressureToSet);
        void Init();
		void useInterrupt(bool);

	private:
		//SoftwareSerial mySerial;
		//Adafruit_GPS GPS;
		bool usingInterrupt;
		//sensors_event_t event;

        
        
};
#endif
