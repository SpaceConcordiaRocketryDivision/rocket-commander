#include <PressureSensor.h>
#include <TransceiverModule.h>

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


//Component objects
PressureSensor pressureSensor;
TransceiverModule transceiverModule;

// Status of sensors: 0 = offline, 1 = online
boolean pressureSensorStatus = 0;
boolean dofSensorStatus = 0;
boolean gpsDataStatus = 0;

int samplingRate = 0;

int count = 1;
// Array for sensors, sizes of each array defined in header for sensor
float dofData[9] = {0}; //9 channels (accel xyz, mag xyz, heading, 2 unused)
float gpsData[10] = {0}; // CHANGE to suit number of required data fields
float bmpData[PRESSURE_ARRAY_SIZE] = {0};  // Pressure Temperature Altitude
char* currentCommand[] = {'\0'};

int rocketStage = LOCKED_GROUND_STAGE;
float x = .2;
float oldAlt = 0;
float filteredAlt;
void setup() {
  Serial.begin(9600);
  
  pressureSensor.Init();
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Alt,AltFiltered,Milis");
}

void loop() {
 
  pressureSensorStatus = pressureSensor.GetData(bmpData);
  
  #ifdef OUTPUT_EXCEL_ENABLED
  Serial.print("DATA,TIME,"); Serial.print(bmpData[3]); Serial.print(","); Serial.print(filteredAlt); Serial.print(","); Serial.println(bmpData[0]);
  #endif
  
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
}
