#ifndef TRANSCEIVERMODULE_H
#define TRANSCEIVERMODULE_H

#include "Arduino.h"
#include <String.h>
class TransceiverModule
{
	public:
          TransceiverModule();
		  boolean GetData (char* currentCommand[] ,int maxArraySize);
          boolean SendData(float arrayToSend[],int arraySize,char componentId);
          void Init();
        

};
#endif
