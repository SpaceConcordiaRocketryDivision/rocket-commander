
#ifndef DATALOGGER_H
#define DATALOGGER_H_G


#include <Wire.h>


class DataLogger
{
	public:
          DataLogger();
		  boolean GetData (float array[]);
          boolean SendData(float pressureToSet);
          void Init();
        
        
};
#endif
