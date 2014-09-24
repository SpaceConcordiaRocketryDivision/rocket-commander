#include "PressureSensor.h" 

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

//Flight Stages of the Rocket
#define LOCKED_GROUND_STAGE 0
#define UNLOCKED_GROUND_STAGE 1
#define PRE_PARA_FLIGHT_STAGE 2
#define FST_PARA_FLIGHT_STAGE 3
#define SCD_PARA_FLIGHT_STAGE 4


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

int rocketStage = LOCKED_GROUND_STAGE;

void setup() { 
  Serial.begin(9600);
  
  pressureSensor.init();
}

void loop() {
  
  pressureSensorStatus = pressureSensor.getData(bmpData);
  outputDataArrays();
  
  switch(rocketStage)
  {
     case LOCKED_GROUND_STAGE:
       break;
     case UNLOCKED_GROUND_STAGE:
       break;
     case PRE_PARA_FLIGHT_STAGE:
       break;
     case FST_PARA_FLIGHT_STAGE:
       break;
     case SCD_PARA_FLIGHT_STAGE:
       break;
     default:
       break;
  }
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
