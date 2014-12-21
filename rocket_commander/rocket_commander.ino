#include <PressureSensor.h>
#include <TransceiverModule.h>
#include <Accelerometer.h>
#include <Gyroscope.h>
//#include <GpsSensor.h>

#include <Wire.h>

#define SIMULATION

//Flight Stages of the Rocket
#define LOCKED_GROUND_STAGE 0
#define STAGE_ONE 1
#define STAGE_TWO 2
#define STAGE_THREE 3
#define STAGE_FOUR 4

//#define OUTPUT_EXCEL_ENABLED 0

//Component Id values
#define PRESSURE_SENSOR_ID 0x50
#define DOF_SENSOR_ID 0x51
#define GPS_SENSOR_ID 0x52



//Kalman Filter Macro
#define MEASUREMENTSIGMA 0.44
#define MODELSIGMA 0.002
#define MEASUREMENTVARIANCE MEASUREMENTSIGMA*MEASUREMENTSIGMA
#define MODELVARIANCE MODELSIGMA*MODELSIGMA

//Component objects
PressureSensor pressureSensor;
//TransceiverModule transceiverModule;
//Accelerometer accelerometer;
//Gyroscope gyroscope;
//GpsSensor gps;

// Status of sensors: 0 = offline, 1 = online
boolean pressureSensorStatus = 0;
boolean dofSensorStatus = 0;
boolean accelerometerStatus = 0; //separate value for each sensor
boolean gyrometerStatus = 0;
boolean gpsStatus = 0;

//Kalman Filter Variables
float gain[3] = { 0.10f, 0.010666, 0.004522 };
float est[3] = { 0, 0, 0 };
float estp[3] = {0, 0, 0 };
float phi[3][3] = { 1, 0, 0,
                      0, 1, 0,
	              0, 0, 1.0 };
float phit[3][3] = { 1, 0, 0,
	               0, 1, 0,
	               0, 0, 1.0 };
float pest[3][3] = { 0.002, 0, 0,
0, 0.004, 0,
0, 0, 0.002 };
float pestp[3][3] = { 0, 0, 0,
0, 0, 0,
0, 0, 0 };
float term[3][3];
int samplingRate = 0;

float apogee_time = 0.0f;

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
  Serial.begin(115200);
  pressureSensor.Init();
  //accelerometer.Init(); //TODO: set up resolution
  //gyroscope.Init(); //TODO: set up resolution
 // gps.Init();
  
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
  //FilterPressure();
  if ( apogee_time == 0.0f && bmpData[1] < 0.0f && bmpData[3] > 10.0f)
    apogee_time = bmpData[0];
  
 // accelerometerStatus = accelerometer.GetData(accelData);  
  //gyrometerStatus = gyroscope.GetData(gyroData);
 // gpsStatus = gps.GetData(gpsData);
  //transceiverModule.SendData(bmpData,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID);
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
void CalculateKalmanGain()
{
  float dt = 7/1000;

  phi[0][1] = dt;
  phi[1][2] = dt;
  phi[0][2] = dt*dt/2.0;
  phit[1][0] = dt;
  phit[2][1] = dt;
  phit[2][0] = dt*dt/2.0;
  
  term[0][0] = phi[0][0] * pest[0][0] + phi[0][1] * pest[1][0] + phi[0][2] * pest[2][0];
  term[0][1] = phi[0][0] * pest[0][1] + phi[0][1] * pest[1][1] + phi[0][2] * pest[2][1];
  term[0][2] = phi[0][0] * pest[0][2] + phi[0][1] * pest[1][2] + phi[0][2] * pest[2][2];
  term[1][0] = phi[1][0] * pest[0][0] + phi[1][1] * pest[1][0] + phi[1][2] * pest[2][0];
  term[1][1] = phi[1][0] * pest[0][1] + phi[1][1] * pest[1][1] + phi[1][2] * pest[2][1];
  term[1][2] = phi[1][0] * pest[0][2] + phi[1][1] * pest[1][2] + phi[1][2] * pest[2][2];
  term[2][0] = phi[2][0] * pest[0][0] + phi[2][1] * pest[1][0] + phi[2][2] * pest[2][0];
  term[2][1] = phi[2][0] * pest[0][1] + phi[2][1] * pest[1][1] + phi[2][2] * pest[2][1];
  term[2][2] = phi[2][0] * pest[0][2] + phi[2][1] * pest[1][2] + phi[2][2] * pest[2][2];

  pestp[0][0] = term[0][0] * phit[0][0] + term[0][1] * phit[1][0] + term[0][2] * phit[2][0];
  pestp[0][1] = term[0][0] * phit[0][1] + term[0][1] * phit[1][1] + term[0][2] * phit[2][1];
  pestp[0][2] = term[0][0] * phit[0][2] + term[0][1] * phit[1][2] + term[0][2] * phit[2][2];
  pestp[1][0] = term[1][0] * phit[0][0] + term[1][1] * phit[1][0] + term[1][2] * phit[2][0];
  pestp[1][1] = term[1][0] * phit[0][1] + term[1][1] * phit[1][1] + term[1][2] * phit[2][1];
  pestp[1][2] = term[1][0] * phit[0][2] + term[1][1] * phit[1][2] + term[1][2] * phit[2][2];
  pestp[2][0] = term[2][0] * phit[0][0] + term[2][1] * phit[1][0] + term[2][2] * phit[2][0];
  pestp[2][1] = term[2][0] * phit[0][1] + term[2][1] * phit[1][1] + term[2][2] * phit[2][1];
  pestp[2][2] = term[2][0] * phit[0][2] + term[2][1] * phit[1][2] + term[2][2] * phit[2][2];
  pestp[0][0] = pestp[0][0] + MODELVARIANCE;

  
}
void FilterPressure()
{
  float dt = 5.0f/1000.0f;

  phi[0][1] = dt;
  phi[1][2] = dt;
  phi[0][2] = dt*dt/2.0;
  phit[1][0] = dt;
  phit[2][1] = dt;
  phit[2][0] = dt*dt/2.0;
  
  /* Propagate state */
  estp[0] = phi[0][0] * est[0] + phi[0][1] * est[1] + phi[0][2] * est[2];
  estp[1] = phi[1][0] * est[0] + phi[1][1] * est[1] + phi[1][2] * est[2];
  estp[2] = phi[2][0] * est[0] + phi[2][1] * est[1] + phi[2][2] * est[2];
  
  est[0] = estp[0] + gain[0] * (bmpData[3] - estp[0]);
  est[1] = estp[1] + gain[1] * (bmpData[3] - estp[0]);
  est[2] = estp[2] + gain[2] * (bmpData[3] - estp[0]);
  
  bmpData[1] = est[1];
  bmpData[2] = est[2];
  bmpData[3] = est[0];
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
    #ifndef SIMULATION 
      Serial.print(bmpData[0]);
      Serial.print(" ms : ");
      Serial.print(bmpData[1]);
      Serial.print(" hPa : ");
      Serial.print(bmpData[2]);
      Serial.print("C : ");
      Serial.print(bmpData[3]);
      Serial.println(" m");
    #else
      Serial.print(bmpData[0]);
      Serial.print(" ms : ");
      Serial.print(bmpData[1]);
      Serial.print(" m/s : ");
      Serial.print(bmpData[2]);
      Serial.print(" m/s^2 : ");
      Serial.print(bmpData[3]);
      Serial.println(" m");
    #endif
  }
  else if (apogee_time != -1.0f)
  {
    Serial.print("Filtered apogee is at ");
    Serial.println(apogee_time);
    apogee_time = -1.0f;
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

