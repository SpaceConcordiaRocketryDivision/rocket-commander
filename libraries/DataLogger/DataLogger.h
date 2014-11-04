#ifndef DATALOGGER_H
#define DATALOGGER_H_G

#include <Wire.h>
#include <Arduino.h>

class DataLogger
{
	public:
          DataLogger();
		  bool GetData (float array[]);
		  bool SendData(char deviceID, float array[], int size);
          void Init();
	private:
		  void CommandMode();
        
        
};
#endif
