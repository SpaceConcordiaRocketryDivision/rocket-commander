
#ifndef GPSSENSOR_H
#define GPSSENSOR_H_G
#define GPS_ARRAY_SIZE 3

#include <Wire.h>


class GpsSensor
{
	public:
          GpsSensor();
		  boolean GetData (float array[]);
          boolean SendData(float pressureToSet);
          void Init();
        
        
};
#endif
