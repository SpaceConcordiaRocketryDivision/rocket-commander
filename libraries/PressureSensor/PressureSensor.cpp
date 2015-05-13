#include "PressureSensor.h"

PressureSensor::PressureSensor() {}

bool PressureSensor::Init() {
	Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
	seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
	Serial.println(bmp.begin());
	return bmp.begin();

}

bool PressureSensor::GetData(float array[]) {
	bmp.getPressureAndTemperature(&array[1], &array[2]);
	array[0] = millis();
	array[3] = bmp.pressureToAltitude(seaLevelPressure, array[1], array[2]);
	return 1; 
}

bool PressureSensor::SendData(float pressureToSet) {
	seaLevelPressure = pressureToSet;
} 
