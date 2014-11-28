#include <PressureSensor.h>
#include <TransceiverModule.h>
#include <Accelerometer.h>
#include <Gyroscope.h>
#include <GpsSensor.h>

#include <Wire.h>

//Flight Stages of the Rocket
#define LOCKED_GROUND_STAGE 0
#define STAGE_ONE 1
#define STAGE_TWO 2
#define STAGE_THREE 3
#define STAGE_FOUR 4

#define OUTPUT_EXCEL_ENABLED 0

//Component Id values
#define PRESSURE_SENSOR_ID 0x50
#define DOF_SENSOR_ID 0x51
#define GPS_SENSOR_ID 0x52

//Component objects
PressureSensor pressureSensor;
TransceiverModule transceiverModule;
Accelerometer accelerometer;
Gyroscope gyroscope;
GpsSensor gps;

// Status of sensors: 0 = offline, 1 = online
boolean pressureSensorStatus = 0;
boolean dofSensorStatus = 0;
boolean accelerometerStatus = 0; //separate value for each sensor
boolean gyrometerStatus = 0;
boolean gpsStatus = 0;

int samplingRate = 0;

int count = 1;
// Array for sensors, sizes of each array defined in header for sensor
float accelData[3] = {
  0}; //x, y, z
float gyroData[3] = { 
  0}; //x, y, z
float gpsData[8] = {
  0}; // CHANGE to suit number of required data fields
float bmpData[PRESSURE_ARRAY_SIZE] = {
  0};  // Pressure Temperature Altitude
char* currentCommand[] = {
  '\0'};

int rocketStage = LOCKED_GROUND_STAGE;
float x = .2;
float oldAlt = 0;
float filteredAlt;
void setup() {
  Serial.begin(9600);
 
  pressureSensor.Init();
  accelerometer.Init(); //TODO: set up resolution
  gyroscope.Init(); //TODO: set up resolution
  gps.Init();
  
  #ifdef OUTPUT_EXCEL_ENABLED
	Serial.println("CLEARDATA");
	Serial.println("LABEL,Time,Alt,AltFiltered,Milis");
  #endif
}

void loop() {
   
  #ifdef OUTPUT_EXCEL_ENABLED
  Serial.print("DATA,TIME,"); Serial.print(bmpData[3]); Serial.print(","); Serial.print(filteredAlt); Serial.print(","); Serial.println(bmpData[0]);
  #endif
  
  pressureSensorStatus = pressureSensor.GetData(bmpData);
  accelerometerStatus = accelerometer.GetData(accelData);  
  gyrometerStatus = gyroscope.GetData(gyroData);
  gpsStatus = gps.GetData(gpsData);

  transceiverModule.SendData(bmpData,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID);
  OutputDataArrays();

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
  if (gpsStatus)
  {
    //Serial.print(gpsData[0]);
    Serial.print(gpsData[1]);
    Serial.print("Latitude: ");
    Serial.println(gpsData[2]);
    Serial.print("Longitude: ");
    Serial.println(gpsData[3]);
    Serial.print("Fix quality: ");
    Serial.println(gpsData[4]); 
    Serial.print("Satellites: ");
    Serial.println(gpsData[5]);
    Serial.print("Altitude: ");
    Serial.println(gpsData[6]);
    Serial.print("Speed: ");
    Serial.println(gpsData[7]); // Unsure about units
  }
}

