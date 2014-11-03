#include "DataLogger.h"
char fileName[12];
DataLogger::DataLogger()
{
	Serial.begin(9600);
	int fileNumber = random(999);
	commandMode();
	sprintf(filename, "log%03d.txt", fileNumber);
	Serial.println("new " + filename);
	//Wait for OpenLog to return to waiting for a command
	while (1) {
		if (OpenLog.available())
			if (OpenLog.read() == '>') break;
	}
	
}
void DataLogger::Init()
{
}
boolean DataLogger::GetData(float array[])
{
  return 0;
}
boolean DataLogger::SendData(String deviceID, String array[], int size)
{
	commandMode();
	Serial.print("append " + filename + "\r");
	//Wait for OpenLog to indicate file is open and ready for writing
	while (1) {
		if (Serial.available())
			if (Serial.read() == '<') break;
	}
	Serial.print(deviceID + ":");
	for (int i = 0; i < size; i++)
	{
		Serial.print(array[i] + ":");
	}
	Serial.println();
	return true;
}
void commandMode()
{
	while (1) {
		Serial.write(26);
		Serial.write(26);
		Serial.write(26);
		if (Serial.available())
			if (Serial.read() == '>') break;
	}
}

