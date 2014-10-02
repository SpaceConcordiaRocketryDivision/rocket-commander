#ifndef TRANSCEIVERMODULE_H
#define TRANSCEIVERMODULE_H

#include "Arduino.h"

class TransceiverModule
{
	public:
          TransceiverModule();
	  boolean GetData (float array[]);
          boolean SendData();
          void Init();
        

};
#endif
