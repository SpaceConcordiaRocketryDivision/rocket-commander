#ifndef TRANSCEIVERMODULE_H
#define TRANSCEIVERMODULE_H

#define START_BYTE 0xFF
#define END_BYTE 0XFE

#include "Arduino.h"
#include <String.h>

class TransceiverModule
{
	public:
          TransceiverModule();
		  boolean GetData (char currentCommand[] ,int maxArraySize);
		  boolean SendData(float arrayToSend[], int arraySize, char commandId, char rocketStage);
		  void Init(char transceiverId);
		  
		  char transceiverId;
        

};
#endif
