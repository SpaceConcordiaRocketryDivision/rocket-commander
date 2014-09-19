#include "PressureSensor.h" 


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

//Component objects
PressureSensor pressureSensor;

// Status of sensors: 0 = offline, 1 = online
boolean pressureSensorStatus = 0; 
boolean dofSensorStatus = 0;
boolean gpsDataStatus = 0;

// Array for sensors, sizes of each array defined in header for sensor
float dofData[9] = {0}; //9 channels (accel xyz, mag xyz, heading, 2 unused)
float gpsData[10] = {0}; // CHANGE to suit number of required data fields 
float bmpData[PRESSURE_ARRAY_SIZE] = {0};  // Pressure Temperature Altitude


void setup() { 
  Serial.begin(9600);
  
  pressureSensor.init();
}

void loop() {
  
  pressureSensorStatus = pressureSensor.getData(bmpData);
  outputDataArrays();
}


void outputDataArrays() {
  if (pressureSensorStatus)
  {
    Serial.print(bmpData[0]);
    Serial.print(" hPa : ");
    Serial.print(bmpData[1]);
    Serial.print("C : ");
    Serial.print(bmpData[2]);
    Serial.println(" m");
  } 
}
