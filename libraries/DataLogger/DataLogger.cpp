#include "DataLogger.h"

DataLogger::DataLogger()
{

}
void DataLogger::Init()
{
	Serial.begin(9600);
	Serial1.begin(9600);
	delay(1000);
	Serial.println(1);
	//delay(1000);
	//fileNumber = 1;
	fileNumber = random(999);
	Serial.println(2);
	//delay(1000);
	CommandMode();
	Serial.println(3);
	//delay(1000);
	//sprintf(filename, "log%03d.txt", fileNumber);
	sprintf(buff, "new data%03d.txt\r", fileNumber);
	Serial1.println(buff);
	Serial.println(4);
	//delay(1000);

	//Wait for OpenLog to return to waiting for a command
	while (1) {
		if (Serial1.available())
			if (Serial1.read() == '>') break;
	}
	Serial.println(5);
	delay(1000);
	sprintf(buff, "append data%03d.txt\r", fileNumber);
	Serial1.println(buff);
	Serial.println(6);
	//delay(1000);
	//Serial.print("append ");
	//Serial.println(filename);
	
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
	sprintf(buff, "read data%03d.txt\r", fileNumber);
	//Serial1.println();
	Serial1.println(buff);
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

//This assumes the device is already set to append the file, which is set up in the init method. It will stay in append mode until the getData method is called. If you want to be able to switch between reading and writing, I can fix this
bool DataLogger::SendData(char deviceID, float array[], int size)
{
	//CommandMode();
	
	
	//Serial.println("It made it through!!");
	Serial1.print(deviceID);
	for (int i = 0; i < size; i++)
	{
		Serial1.print(":");
		Serial1.print(array[i]);
	}
	Serial1.println();
	//CommandMode();
	return true;
}
void DataLogger::CommandMode()
{
	/*Serial1.write(26);
	delay(50);
	Serial1.write(26);
	delay(50);
	Serial1.write(26);*/
	
	//The device is supposed to go into command mode after you send CTRL+Z to it three times, but the only way I could get it to actually work is by sending it repeatedly until it finally worked
	while (1) {
		Serial1.write(26);
		//delay(2);
		if (Serial1.available())
			if (Serial1.read() == '>') break;
		//delay(100);
	}
	//delay(1000);
}
