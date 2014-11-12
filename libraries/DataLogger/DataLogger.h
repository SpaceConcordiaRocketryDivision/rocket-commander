#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <Wire.h>
#include <Arduino.h>

class DataLogger
{
	public:
          DataLogger();
		  bool GetData (char array[]);
		  bool SendData(char deviceID, float array[], int size);
          void Init();
		  int fileNumber;
		  char filename[12];
		  char buff[50];
	private:
		  void CommandMode();
        
        
};
#endif
