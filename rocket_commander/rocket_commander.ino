#include <PressureSensor.h>
#include <TransceiverModule.h>
#include <Accelerometer.h>
#include <Gyroscope.h>

#include <Wire.h>

//Flight Stages of the Rocket
#define LOCKED_GROUND_STAGE 0
#define STAGE_ONE 1
#define STAGE_TWO 2
#define STAGE_THREE 3
#define STAGE_FOUR 4

//Component Id values
#define PRESSURE_SENSOR_ID 0x50
#define DOF_SENSOR_ID 0x51

//Component objects
PressureSensor pressureSensor;
TransceiverModule transceiverModule;
Accelerometer accelerometer;
Gyroscope gyroscope;

// Status of sensors: 0 = offline, 1 = online
boolean pressureSensorStatus = 0;
boolean dofSensorStatus = 0;
boolean accelerometerStatus = 0; //separate value for each sensor
boolean gyrometerStatus = 0;
boolean gpsDataStatus = 0;

// Array for sensors, sizes of each array defined in header for sensor
float accelData[3] = {
  0}; //x, y, z
float gyroData[3] = { 
  0}; //x, y, z
float gpsData[10] = {
  0}; // CHANGE to suit number of required data fields
float bmpData[PRESSURE_ARRAY_SIZE] = {
  0};  // Pressure Temperature Altitude
char* currentCommand[] = {
  '\0'};

int rocketStage = LOCKED_GROUND_STAGE;

void setup() {
  Serial.begin(9600);
  pressureSensor.Init();
  accelerometer.Init(); //TODO: set up resolution
  gyroscope.Init(); //TODO: set up resolution
}

void loop() {
  //TODO: switch sensor to output
  Serial.print("hi");
  pressureSensorStatus = pressureSensor.GetData(bmpData);
  accelerometerStatus = accelerometer.GetData(accelData);  
  gyrometerStatus = gyroscope.GetData(gyroData);
  transceiverModule.SendData(bmpData,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID);
  //OutputDataArrays();

  switch (rocketStage)
  {
  case LOCKED_GROUND_STAGE:
    break;
  case STAGE_ONE:
    StageOne();
    break;
  case STAGE_TWO:
    StageTwo();
    break;
  case STAGE_THREE:
    StageThree();
    break;
  case STAGE_FOUR:
    StageFour();
    break;
  default:
    break;
  }
}

void StageOne()
{
  if (1)
  {


  }
}

void StageTwo()
{
}

void StageThree()
{
}

void StageFour()
{
}

void OutputDataArrays() {
  if (pressureSensorStatus)
  {
    Serial.print(bmpData[0]);
    Serial.print(" ms : ");
    Serial.print(bmpData[1]);
    Serial.print(" hPa : ");
    Serial.print(bmpData[2]);
    Serial.print("C : ");
    Serial.print(bmpData[3]);
    Serial.println(" m");
  }

  if (accelerometerStatus)
  {
    Serial.print(accelData[0]);
    Serial.print(" m/s^2 x" );
    Serial.print(accelData[1]);
    Serial.print(" m/s^2 y ");
    Serial.print(accelData[2]);
    Serial.println(" m/s^2 z ");
  }
  if (gyrometerStatus)
  {
    Serial.print(accelData[0]);
    Serial.print(" rad/s x" );
    Serial.print(accelData[1]);
    Serial.print(" rad/s y ");
    Serial.print(accelData[2]);
    Serial.println(" rad/s z ");
  }
}

