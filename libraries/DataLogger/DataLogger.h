#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <Wire.h>
#include <Arduino.h>

class DataLogger
{
	public:
          DataLogger();
		  bool GetData (float array[]);
		  bool SendData(char deviceID, float array[], int size);
          void Init();
		  char filename[12];
	private:
		  void CommandMode();
        
        
};
#endif
