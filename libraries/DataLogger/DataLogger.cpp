#include "DataLogger.h"

DataLogger::DataLogger()
{

}
void DataLogger::Init()
{
	Serial.begin(115200);
	Serial1.begin(115200);
	delay(1000);
	fileNumber = random(999);
	CommandMode();
	sprintf(buff, "new data%03d.txt\r", fileNumber);
	Serial1.print(buff);

	//Wait for OpenLog to return to waiting for a command
	while (1) {
		if (Serial1.available())
			if (Serial1.read() == '>') break;
	}

	sprintf(buff, "append data%03d.txt\r", fileNumber);
	Serial1.print(buff);
	
	//Wait for OpenLog to indicate file is open and ready for writing
	while (1) {
		if (Serial1.available())
			if (Serial1.read() == '<') break;
	}
}

//This method kind of works, but has major issues and doesn't run like it should for some reason
bool DataLogger::GetData(char array[])
{
	CommandMode();
	Serial.println(9);
	//delay(5000);
	sprintf(buff, "read data%03d.txt", fileNumber);
	//Serial1.print("read data001.txt");
	Serial1.print(buff);
	Serial1.write(13);

	//The OpenLogger echos any text you send to it, so this makes sure you don't store the command you just sent
	while (1) {
		if (Serial1.available())
			if (Serial1.read() == '\r') break;
	}
	Serial.println(10);
	//delay(1000);

	//This loop will stop listening after 1 second of no characters received
	int spot = 0;
	for (int timeOut = 0; timeOut < 1000; timeOut++) {
		while (Serial1.available()) {
			array[spot++] = Serial1.read();
			//Serial.write(array); //Take the string from OpenLog and push it to the Arduino terminal
			timeOut = 0;
		}
		delay(1);
	}
	Serial.println(11);
	return true;
}

bool DataLogger::SendData(char deviceID, float array[], int size)
{
	Serial1.print(deviceID);
	for (int i = 0; i < size; i++)
	{
		Serial1.print(":");
		Serial1.print(array[i]);
	}
	Serial1.println();
	return true;
}
void DataLogger::CommandMode()
{
	//The device is supposed to go into command mode after you send CTRL+Z to it three times, but the only way I could get it to actually work is by sending it repeatedly until it finally worked.
	//However, I think this makes it print extra CTRL+Z's and that is probably what causes the errors for the get method
	//At higher baud rates it does seem to work with just three, but it only works the first time. So the same issues happen for the get method
	Serial1.write(26);
	Serial1.write(26);
	Serial1.write(26);
	while (1) {
		if (Serial1.available())
			if (Serial1.read() == '>') break;
	}
}
