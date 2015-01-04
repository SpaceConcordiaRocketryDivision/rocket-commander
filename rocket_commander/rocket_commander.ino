#include <PressureSensor.h>
#include <TransceiverModule.h>
#include <Accelerometer.h>
#include <Gyroscope.h>

#include <Wire.h>

// ifdef to enable or disable pieces of code for debugging
#define SIMULATION
#define TRANSCIEVER_ENABLED

//Flight Stages of the Rocket
#define LOCKED_GROUND_STAGE 0
#define STAGE_ONE 1
#define STAGE_TWO 2
#define STAGE_THREE 3
#define STAGE_FOUR 4
#define STAGE_FIVE 5

//#define OUTPUT_EXCEL_ENABLED 0

//Component Id values
#define PRESSURE_SENSOR_ID 0x50
#define DOF_SENSOR_ID 0x41
#define GYRO_SENSOR_ID 0x47
#define GPS_SENSOR_ID 0x52


//Kalman Filter Macro
#define MEASUREMENTSIGMA 0.44
#define MODELSIGMA 0.002
#define MEASUREMENTVARIANCE MEASUREMENTSIGMA*MEASUREMENTSIGMA
#define MODELVARIANCE MODELSIGMA*MODELSIGMA

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
boolean gpsStatus = 0;
boolean commandRecieved = 0;

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
float calculatedVelocity = 0.0f;
float calculatedAcceleration = 0.0f;
float filteredAltitude = 0.0f;
int samplingRate = 0;

float apogee_time = 0.0f;

int count = 1;
// Array for sensors, sizes of each array defined in header for sensor
float accelData[ACCELEROMETER_ARRAY_SIZE] = {
  0}; //x, y, z
float gyroData[GYROSCOPE_ARRAY_SIZE] = { 
  0}; //x, y, z
float gpsData[8] = {
  0}; // CHANGE to suit number of required data fields
float bmpData[PRESSURE_ARRAY_SIZE] = {
  0};  // Pressure Temperature Altitude
  

  
int bandCount = 0;

int rocketStage = LOCKED_GROUND_STAGE;
float x = .2; 
float oldAlt = 0;
float filteredAlt;
void setup() {
  Serial.begin(115200);
  
  pinMode(7,OUTPUT);
  
  pressureSensor.Init();
  accelerometer.Init(); 
  transceiverModule.Init('A');
 // gyroscope.Init(); //TODO: set up resolution
  
  #ifdef OUTPUT_EXCEL_ENABLED
	Serial.println("CLEARDATA");
	Serial.println("LABEL,Time,Alt,AltFiltered,Milis");
  #endif
}

void loop() { 
   char currentCommand[25] = {
  '\0'};
  
  #ifdef OUTPUT_EXCEL_ENABLED
  Serial.print("DATA,TIME,"); Serial.print(bmpData[3]); Serial.print(","); Serial.print(filteredAlt); Serial.print(","); Serial.println(bmpData[0]);
  #endif
  
  #ifdef SIMULATION
    SimulateValues();
  #else
    pressureSensorStatus = pressureSensor.GetData(bmpData);
    accelerometerStatus = accelerometer.GetData(accelData);
  #endif 
  
  FilterPressure();

  //gyrometerStatus = gyroscope.GetData(gyroData);
  #ifdef TRANSCIEVER_ENABLED
  delay(5);
   if (bandCount % 25 == 0 )
   {
     if (pressureSensorStatus)
       transceiverModule.SendData(bmpData,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocketStage));
     if ( bandCount % 50 == 0 )
     {
       if (accelerometerStatus)
         transceiverModule.SendData(accelData,ACCELEROMETER_ARRAY_SIZE,DOF_SENSOR_ID, (char)(((int)'0') + rocketStage));
       if (gyrometerStatus)
         transceiverModule.SendData(gyroData,GYROSCOPE_ARRAY_SIZE,GYRO_SENSOR_ID, (char)(((int)'0') + rocketStage));
     }
   }
   
   
   #endif
  bandCount++;
  
  commandRecieved = transceiverModule.GetData(currentCommand,25);
  if ( commandRecieved )
  {
    for ( int i = 0; i < 25; i++ )
      Serial.println((int)currentCommand[i]);
    commandRecieved = 0 ;
  }
  if ( currentCommand[0] == (char)START_BYTE )
  {
    if ( currentCommand[7] == 'U' && currentCommand[8] == 'U' && currentCommand[12] == 'S' && currentCommand[13] == '1' )
      rocketStage = STAGE_ONE;
    else if ( currentCommand[7] == 'L' && currentCommand[8] == 'L' && currentCommand[12] == 'S' && currentCommand[13] == '0' )
      rocketStage = LOCKED_GROUND_STAGE;
  }
 // OutputDataArrays();
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
  estp[0] = phi[0][0] * filteredAltitude + phi[0][1] * est[1] + phi[0][2] * est[2];
  estp[1] = phi[1][0] * filteredAltitude + phi[1][1] * est[1] + phi[1][2] * est[2];
  estp[2] = phi[2][0] * filteredAltitude + phi[2][1] * est[1] + phi[2][2] * est[2];
  
  est[0] = estp[0] + gain[0] * (bmpData[3] - estp[0]);
  est[1] = estp[1] + gain[1] * (bmpData[3] - estp[0]);
  est[2] = estp[2] + gain[2] * (bmpData[3] - estp[0]);
  
  calculatedVelocity = est[1];
  calculatedAcceleration = est[2];
  filteredAltitude = est[0];
}
void StageOne()
{
  if (filteredAltitude > 10.0f) // if altitude is greater than 10m
  {
    Serial.println("Thrust Stage");
    rocketStage = STAGE_TWO;
  }
}

void StageTwo()
{
  if ( calculatedAcceleration < 0 )
  {
    rocketStage = STAGE_THREE;
    Serial.println("Deceleration Stage");
  }
}

void StageThree()
{
  if ( calculatedVelocity <= 0 )
  {
    digitalWrite(7, HIGH);  
    rocketStage = STAGE_FOUR;
   // Serial.println("Descent drogue chute stage");
    apogee_time = bmpData[0];
    //Serial.print("Filtered apogee is at ");
    //Serial.println(apogee_time);
  }
}

void StageFour()
{
  if ( filteredAltitude <= 1000.0f )
  {
    Serial.println("Ground Stage");
    rocketStage = STAGE_FIVE;
  }
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
  else if (apogee_time != -1.0f)
  {
    Serial.print("Filtered apogee is at ");
    Serial.println(apogee_time);
    apogee_time = -1.0f;
  }
  if (accelerometerStatus)
  {
    Serial.print(bmpData[0]);
    Serial.print(" ms : ");
    Serial.print(accelData[1]);
    Serial.print(" m/s^2 x:" );
    Serial.print(accelData[2]);
    Serial.print(" m/s^2 y:");
    Serial.print(accelData[3]);
    Serial.println(" m/s^2 z:");
  }
  if (gyrometerStatus)
  {
    Serial.print(bmpData[0]);
    Serial.print(" ms : ");
    Serial.print(gyroData[1]);
    Serial.print(" rad/s x" );
    Serial.print(gyroData[2]);
    Serial.print(" rad/s y ");
    Serial.print(gyroData[3]);
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
void SimulateValues()
{
  static bool simulation_done = false;
  static bool apogee = false;
  static float apogee_time = 0.0f;
  static float start_millis = millis();
  static float actual_millis = 0.0f;
  static float dt = 0.0f;	
  static float t_accmax = .15f;
  static float a_peak = 160;
  static float hold_a_max = 1.0f;
  static float t_g = .5f;
  static float altitude = 0.01f;
  static float velocity = 0.01f;
  static float acceleration = -9.81f;
  static float acceleration_rate = (a_peak + 9.81f) / t_accmax;
  static float deacceleration_rate =  ( 9.81f + a_peak ) / -(t_g); 
  static float standard_deviation_alt = 1.0f;
  if (!simulation_done && rocketStage > 0)
  {
    //dt = millis() / 1000.0f - actual_millis;
      dt = 5.0f/1000.0f;
      //Serial.println(dt * 1000.0f);
      //Serial.println(start_millis);
      //actual_millis = (millis() - start_millis) / 1000.0f;
      actual_millis += dt;
      
      if (actual_millis < t_accmax)
        acceleration = acceleration + acceleration_rate * dt;
      else if (actual_millis < hold_a_max)
	acceleration = a_peak;
      else if (acceleration > -9.81f)
	acceleration = acceleration + deacceleration_rate * dt;
      else
	acceleration = -9.81;
      float x1, x2, w, y1, y2;
      if (altitude <= 0.0f)
      {
	simulation_done = true;
	Serial.print("Simulation Over at ");
	Serial.println(actual_millis * 1000.0f);
		
	Serial.print("Simulation Apogee at ");
	Serial.println(apogee_time * 1000.0f);
	apogee = true;
        return;
      }
      else
      {
        velocity = velocity + acceleration * dt;
	altitude = altitude + velocity * dt + .5f*dt*dt*acceleration;
	//Generate normal distribution number using box-muller transform
	if (  apogee_time == 0.0f && velocity <= 0.0f  && altitude > 10.0f)
	{
	  apogee_time = actual_millis;
	  Serial.println(apogee_time);
	}
	do
	{
	  x1 = 2.0 * ((float)random(1000) / 1000.0f) - 1.0;
	  x2 = 2.0 * ((float)random(1000) / 1000.0f) - 1.0;
	  w = x1 * x1 + x2 * x2;
	} while (w >= 1.0);
	w = sqrt((-2.0 * log(w)) / w);
	y1 = x1 * w;
	y2 = x2 * w;
      }
      bmpData[3] = altitude + y1 * standard_deviation_alt;
      accelData[3] = acceleration;
      bmpData[0] = actual_millis * 1000;
      accelData[0] = actual_millis * 1000;
  }
  pressureSensorStatus = 1;
  accelerometerStatus = 1;
}

