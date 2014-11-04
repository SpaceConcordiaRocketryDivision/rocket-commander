#include "DataLogger.h"
char filename[12];
DataLogger::DataLogger()
{
	Serial.begin(9600);
	int fileNumber = 1;
	CommandMode();
	sprintf(filename, "log%03d.txt", fileNumber);
	Serial.print("new ");
	Serial.println(filename);
	//Wait for OpenLog to return to waiting for a command
	while (1) {
		if (Serial.available())
			if (Serial.read() == '>') break;
	}
	
}
void DataLogger::Init()
{
}
bool DataLogger::GetData(float array[])
{
  return 0;
}
bool DataLogger::SendData(char deviceID, float array[], int size)
{
	CommandMode();
	Serial.print("append ");
	Serial.println(filename);
	//Wait for OpenLog to indicate file is open and ready for writing
	while (1) { // Do you need this? When you do serial.begin it should be ready to write
		if (Serial.available())
			if (Serial.read() == '<') break;
	}
	Serial.print(deviceID + ":");
	for (int i = 0; i < size; i++)
	{
		Serial.print(array[i]);
		Serial.print(":");
	}
	Serial.println();
	return true;
}
void DataLogger::CommandMode()
{
	while (1) {
		Serial.write(26);
		Serial.write(26);
		Serial.write(26);
		if (Serial.available())
			if (Serial.read() == '>') break;
	}
}
