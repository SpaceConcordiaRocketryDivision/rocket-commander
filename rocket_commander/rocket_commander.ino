#include <PressureSensor.h>
#include <TransceiverModule.h>
#include <Accelerometer.h>
//#include <Gyroscope.h>

//#include <SoftwareSerial.h>
#include <Wire.h>

// ifdef to enable or disable pieces of code for debugging
//#define SIMULATION
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
#define CALC_VALUES_ID 0x43

//Ejection Charge Pins
#define DROGUE_CHUTE_PIN 7
#define MAIN_CHUTE_PIN 6

//Kalman Filter Macro
#define MEASUREMENTSIGMA 0.44
#define MODELSIGMA 0.002
#define MEASUREMENTVARIANCE MEASUREMENTSIGMA*MEASUREMENTSIGMA
#define MODELVARIANCE MODELSIGMA*MODELSIGMA


//Component objects
PressureSensor pressureSensor;
TransceiverModule transceiverModule;
Accelerometer accelerometer;
//Gyroscope gyroscope;

// Status of sensors: 0 = offline, 1 = online
boolean pressureSensorStatus = 0;
boolean dofSensorStatus = 0;
boolean accelerometerStatus = 0; //separate value for each sensor
boolean gyrometerStatus = 0;
boolean gpsStatus = 0;
boolean commandRecieved = 0;
boolean transmitCustomData = 1;

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
float pest[3][3] = { 1, 0, 0,
0, 1, 0,
0, 0, 1 };
float pestp[3][3] = { 0, 0, 0,
0, 0, 0,
0, 0, 0 };
float term[3][3];
float calculatedVelocity = 0.0f;
float calculatedAcceleration = 0.0f;
float filteredAltitude = 0.1f;
int samplingRate = 0;

float apogee_time = 0.0f;
float filt_apogee_time = 0.0f;

int count = 1;
// Array for sensors, sizes of each array defined in header for sensor
float accelData[ACCELEROMETER_ARRAY_SIZE] = {
  0}; //x, y, z
//float gyroData[GYROSCOPE_ARRAY_SIZE] = { 
  //0}; //x, y, z
float gpsData[8] = {
  0}; // CHANGE to suit number of required data fields
float bmpData[PRESSURE_ARRAY_SIZE] = {
  0};  // Pressure Temperature Altitude
  

bool simulationOn = 0;  
int bandCount = 0;
int oldBandCount = 0;
long transcieverCount = 0;

int rocketStage = 0;
float velocity = 0.01f;
void setup() {
 // mySerial.begin(115200);
  Serial.begin(115200);
  pinMode(DROGUE_CHUTE_PIN,OUTPUT);
  pinMode(MAIN_CHUTE_PIN, OUTPUT);
  
  pressureSensor.Init();
  accelerometer.Init(); 
  transceiverModule.Init('A');
 // gyroscope.Init(); //TODO: set up resolution
  
  #ifdef OUTPUT_EXCEL_ENABLED
	Serial.println("CLEARDATA");
	Serial.println("LABEL,Time,Alt,AltFiltered,Milis");
  #endif
  CalculateKalmanGain(80);
}

void loop() { 
   char currentCommand[25] = {
  '\0'};
  #ifdef OUTPUT_EXCEL_ENABLED
  Serial.print("DATA,TIME,"); Serial.print(bmpData[3]); Serial.print(","); Serial.print(filteredAlt); Serial.print(","); Serial.println(bmpData[0]);
  #endif
    
  if (simulationOn)
    SimulateValues();
  else
  {
    pressureSensorStatus = pressureSensor.GetData(bmpData);
    accelerometerStatus = accelerometer.GetData(accelData);
    //gyrometerStatus = gyroscope.GetData(gyroData);
    if ( bandCount == 10 )
      pressureSensor.SendData(bmpData[1]);
  } 
  if (bandCount == 100)
    rocketStage=1;
  //int timeTemp = millis();

  //Serial.println(millis()-timeTemp);
  FilterPressure();


  #ifdef TRANSCIEVER_ENABLED

   if (transcieverCount * 1000 < millis() ) // every
   {
     Serial.print("Iterations for last one seconds: "); Serial.println(bandCount - oldBandCount);
     
     transcieverCount++;
     if (pressureSensorStatus)
       transceiverModule.SendData(bmpData,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocketStage));
     if ( transcieverCount % 2 == 0 ) // Transmit every 2 seconds
     {
       if (accelerometerStatus)
           transceiverModule.SendData(accelData,ACCELEROMETER_ARRAY_SIZE,DOF_SENSOR_ID, (char)(((int)'0') + rocketStage));
      // if (gyrometerStatus)
        //   transceiverModule.SendData(gyroData,GYROSCOPE_ARRAY_SIZE,GYRO_SENSOR_ID, (char)(((int)'0') + rocketStage));
 
     }
     else
     {
       if (transmitCustomData)
       {
           float tempArray[] = {bmpData[0],filteredAltitude, calculatedVelocity,(float)(bandCount-oldBandCount)};
           transceiverModule.SendData(tempArray,4,CALC_VALUES_ID, (char)(((int)'0') + rocketStage));
       }  
     }
     oldBandCount = bandCount;
   }

   
   #endif
  bandCount++;
  
  if ( transceiverModule.GetData(currentCommand,25) && currentCommand[0] == (char)START_BYTE )
  {
    if ( currentCommand[7] == 'U' && currentCommand[8] == 'U' && currentCommand[12] == 'S' && currentCommand[13] == '1' ) // Put rocket-commander in stage one(unlocked)
      rocketStage = STAGE_ONE;
    else if ( currentCommand[7] == 'L' && currentCommand[8] == 'L' && currentCommand[12] == 'S' && currentCommand[13] == '0' ) // Put rocket-commander in stage zero (locked)
      rocketStage = LOCKED_GROUND_STAGE;
    else if ( currentCommand[7] == 'S' && currentCommand[8] == 'M' && currentCommand[12] == 'S' && currentCommand[13] == '1' ) // Put rocket-commander in simulation mode
      simulationOn = 1;
    else if ( currentCommand[7] == 'S' && currentCommand[8] == 'M' && currentCommand[12] == 'S' && currentCommand[13] == '0' ) // Turn simulation mode off for rocket-commander
      simulationOn = 0;
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
void CalculateKalmanGain(int count)
{
  for ( int i = 0; i <= count; i++ )
  {
    float dt = 20.0f/1000.0f;
  
    phi[0][1] = dt;
    phi[1][2] = dt;
    phi[0][2] = dt*dt/2.0;
    phit[1][0] = dt;
    phit[2][1] = dt;
    phit[2][0] = dt*dt/2.0;
   //Error Covariance Update P = A*P*A' + Q;
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
  
  // Equivalent to K = P*H'/(H*P*H' + R);
    gain[0] = (phi[0][0] * pestp[0][0] + phi[0][1] * pestp[1][0] + phi[0][2] * pestp[2][0]) / (pestp[0][0] + MEASUREMENTVARIANCE);
    gain[1] = (phi[1][0] * pestp[0][0] + phi[1][1] * pestp[1][0] + phi[1][2] * pestp[2][0]) / (pestp[0][0] + MEASUREMENTVARIANCE);
    gain[2] = (phi[2][0] * pestp[0][0] + phi[2][1] * pestp[1][0] + phi[2][2] * pestp[2][0]) / (pestp[0][0] + MEASUREMENTVARIANCE);
    
    //Error Covariance Correction P = (eye(3) - K *H)*P;
    pest[0][0] = pestp[0][0] * (1.0 - gain[0]);
    pest[0][1] = pestp[1][0] * (1.0 - gain[0]);
    pest[0][2] = pestp[2][0] * (1.0 - gain[0]);
    pest[1][0] = pestp[0][1] - gain[1] * pestp[0][0];
    pest[1][1] = pestp[1][1] - gain[1] * pestp[1][0];
    pest[1][2] = pestp[2][1] - gain[1] * pestp[2][0];
    pest[2][0] = pestp[0][2] - gain[2] * pestp[0][0];
    pest[2][1] = pestp[1][2] - gain[2] * pestp[1][0];
    pest[2][2] = pestp[2][2] - gain[2] * pestp[2][0];
    
    Serial.print( "Gain[0]: "); Serial.print(gain[0]); Serial.print( " Gain[1]: ");Serial.print(gain[1]); Serial.print( " Gain[2]: ");Serial.println(gain[2]);
  }
}
void FilterPressure()
{
  float dt = 20.0f/1000.0f;
  
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
    digitalWrite(DROGUE_CHUTE_PIN, HIGH);  
    rocketStage = STAGE_FOUR;
   // Serial.println("Descent drogue chute stage");
    filt_apogee_time = bmpData[0];
    //Serial.print("Filtered apogee is at ");
    //Serial.println(apogee_time);
  }
}

void StageFour()
{
  if ( filteredAltitude <= 1000.0f )
  {
    digitalWrite(MAIN_CHUTE_PIN, HIGH);
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
      Serial.print(" m: ");
      
      Serial.print(" Filt altitude: ");
      Serial.print(filteredAltitude);
      Serial.print(" m");
      Serial.print(" Calc Velocity: ");
      Serial.print(calculatedVelocity);
      Serial.print(" m/s");
      Serial.print(" Actual Velocity: ");
      Serial.print(velocity);
      Serial.println(" m/s");
  }
  else if (filt_apogee_time != -1.0f)
  {
    Serial.print("Filtered apogee is at ");
    Serial.println(filt_apogee_time);
    filt_apogee_time = -1.0f;
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
  /*if (gyrometerStatus)
  {
    Serial.print(bmpData[0]);
    Serial.print(" ms : ");
    Serial.print(gyroData[1]);
    Serial.print(" rad/s x" );
    Serial.print(gyroData[2]);
    Serial.print(" rad/s y ");
    Serial.print(gyroData[3]);
    Serial.println(" rad/s z ");
  }*/
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
  //static float velocity = 0.01f;
  static float acceleration = -9.81f;
  static float acceleration_rate = (a_peak + 9.81f) / t_accmax;
  static float deacceleration_rate =  ( 9.81f + a_peak ) / -(t_g); 
  static float parachute_deacceleration_rate = ( 9.81f) / 5.0f;
  static float standard_deviation_alt = 1.0f;
  
  if (!simulation_done && rocketStage > 0)
  {
      delay(5);
    //dt = millis() / 1000.0f - actual_millis;
      dt = 20.0f/1000.0f;
      //Serial.println(dt * 1000.0f);
      //Serial.println(start_millis);
      //actual_millis = (millis() - start_millis) / 1000.0f;
      actual_millis += dt;
      
      if (actual_millis < t_accmax)
        acceleration = acceleration + acceleration_rate * dt;
      else if (actual_millis < hold_a_max)
	acceleration = a_peak;
      else if (acceleration > -9.81f && velocity >= 0)
	acceleration = acceleration + deacceleration_rate * dt;
      else if (acceleration < 0.0f )
	acceleration = acceleration + parachute_deacceleration_rate * dt;
      else
        acceleration = 0.0f;
        
      float x1, x2, w, y1, y2;
      if (altitude <= 0.0f)
      {
	simulation_done = true;
	Serial.print("Simulation Over at ");
	Serial.println(actual_millis * 1000.0f);
		
	Serial.print("Simulation Apogee at ");
	Serial.println(apogee_time * 1000.0f);

        Serial.print("Filtered Simulation apogee at ");
        Serial.println(filt_apogee_time);
        
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
	  //Serial.println(apogee_time);
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

