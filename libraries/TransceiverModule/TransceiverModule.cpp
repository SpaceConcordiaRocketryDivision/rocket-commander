#include "TransceiverModule.h"
#include "Arduino.h"

TransceiverModule::TransceiverModule()
{
	Serial.begin(9600);
}
void TransceiverModule::Init()
{

}
boolean TransceiverModule::GetData(char* currentCommand[] , int maxArraySize)
{
	if (Serial.available() > 0 )
	{
		int arraySize = 0;
		while (Serial.available() > 0 && arraySize < maxArraySize ) {
			currentCommand[arraySize++] = (char*)Serial.read();
		}
		return 1;
    }
  return 0;
}
boolean TransceiverModule::SendData(float arrayToSend[],int arraySize, char componentId )
{
  if(Serial)
  {
	  Serial.print(componentId);
	  Serial.print(":");
	  Serial.print((long)arrayToSend[0]);
	  for(int i = 1; i < arraySize; i++)
	  {
	    Serial.print(":");
		Serial.print(arrayToSend[i]);
	  }
	  Serial.println();
	  return 1;
  }
  return 0;
}
