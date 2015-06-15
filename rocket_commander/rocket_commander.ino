#include <PressureSensor.h>
#include <TransceiverModule.h>
#include <Accelerometer.h>


//#include <SoftwareSerial.h>
#include <Wire.h>

//Flight Stages of the Rocket
#define LOCKED_GROUND_STAGE 0 // Rocket is on launch pad and is locked
#define STAGE_ONE 1           // Rocket is on launch pad and is unlocked
#define STAGE_TWO 2           // Rocket motor is burning and rocket is accelerating
#define STAGE_THREE 3         // Rocket motor is burnt out and rocket is deaccelerating and approaching apogee
#define STAGE_FOUR 4          // Rocket has achieved apogee, deployed drogue chute and is now descending
#define STAGE_FIVE 5          // Rocket has achieved specified altitude, deployed main chute and still descending but now slower towards the ground

//Component Id values
#define PRESSURE_SENSOR_ID 0x50
#define DOF_SENSOR_ID 0x41
#define GYRO_SENSOR_ID 0x47
#define GPS_SENSOR_ID 0x52
#define CALC_VALUES_ID 0x43

//Ejection Charge Pins
#define DROGUE_CHUTE_TRANSISTOR 2 
#define DROGUE_CHUTE_PIN 3
#define MAIN_CHUTE_PIN 6
#define MAIN_CHUTE_TRANSISTOR 7 

//Kalman Filter Macro
#define MEASUREMENTSIGMA 0.44
#define MODELSIGMA 0.002
#define MEASUREMENTVARIANCE MEASUREMENTSIGMA*MEASUREMENTSIGMA
#define MODELVARIANCE MODELSIGMA*MODELSIGMA


//Component objects
PressureSensor pressure_sensor;
TransceiverModule transceiver_module;
Accelerometer accelerometer_sensor;


// Status of sensors: 0 = offline, 1 = online
boolean pressure_sensor_status = 0;
boolean accelerometer_status = 0; //separate value for each sensor

boolean transmit_customData = 1;

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
float calculated_velocity = 0.0f;
float calculated_acceleration = 0.0f;
float filtered_altitude = 0.1f;
float max_altitude = 0.0f;

float apogee_time = 0.0f;
float filt_apogee_time = 0.0f;

int count = 1;
int send_rate = 150;
// Array for sensors, sizes of each array defined in header for sensor
float accel_data[ACCELEROMETER_ARRAY_SIZE] = {
  0}; //x, y, z
float bmp_data[PRESSURE_ARRAY_SIZE] = {
  0};  // Pressure Temperature Altitude
  
int time_temp = 0;
bool simulation_on = 0;  
float new_millis_loop = 0;
float old_millis_loop = 0;
long transciever_count = 0;

int rocket_stage = 0;
float velocity = 0.01f;

void setup() {
  int initCounter = 0;
  Serial.begin(115200);
  
  pinMode(DROGUE_CHUTE_PIN,OUTPUT);
  pinMode(MAIN_CHUTE_PIN, OUTPUT);
  pinMode(DROGUE_CHUTE_TRANSISTOR, OUTPUT);
  pinMode(MAIN_CHUTE_TRANSISTOR, OUTPUT);
  
  pressure_sensor_status = pressure_sensor.Init();
  accelerometer_status = accelerometer_sensor.Init(); 
  transceiver_module.Init('A');

  while(initCounter < 200) // So some sample readings to let the sensors settle
  {
 
    initCounter++;
    pressure_sensor_status = pressure_sensor.GetData(bmp_data);
    accelerometer_sensor.GetData(accel_data);
  }
  pressure_sensor.SendData(bmp_data[1]);

  CalculateKalmanGain(40);
}

void loop() { 
  
  old_millis_loop = new_millis_loop;
  new_millis_loop = millis();
     
  if (simulation_on)
  {
    SimulateValues();
  }
  else
  {
    if ( pressure_sensor_status)
      pressure_sensor.GetData(bmp_data);
    if ( accelerometer_status)
      accelerometer_sensor.GetData(accel_data);
  } 
  
  FilterPressure();
  
  if ( max_altitude < filtered_altitude )
    max_altitude = filtered_altitude;
  
   if (transciever_count * send_rate  < millis() ) // every
   {
    // Serial.print("Iterations for last one seconds: "); Serial.println(band_count - old_band_count);
     
     transciever_count++;
     if (pressure_sensor_status)
       transceiver_module.SendData(bmp_data,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocket_stage));
     if ( transciever_count % 2 == 0 ) // Transmit every 2 seconds
     { 
       if (accelerometer_status)
           transceiver_module.SendData(accel_data,ACCELEROMETER_ARRAY_SIZE,DOF_SENSOR_ID, (char)(((int)'0') + rocket_stage));
      // if (gyrometerStatus)
        //   transceiver_module.SendData(gyroData,GYROSCOPE_ARRAY_SIZE,GYRO_SENSOR_ID, (char)(((int)'0') + rocket_stage));
 
     }
     else
     {                                                                                                                                                                                                                                                                                                                                                        
       if (transmit_customData)
       {
           float tempArray[] = {bmp_data[0],filtered_altitude, calculated_velocity,new_millis_loop - old_millis_loop,max_altitude};
           transceiver_module.SendData(tempArray,5,CALC_VALUES_ID, (char)(((int)'0') + rocket_stage));
       }  
     }
    
   } 
  CheckIfcommand_recieved();
 
  //OutputDataArrays(); 
 
   // Below handles the rocket stages, the functions will transition the stage of the rocket to the next stage when a condition is met
  switch (rocket_stage)
  {
  case LOCKED_GROUND_STAGE:
    StageZero();
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
    float dt = 15.0f/1000.0f;
  
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
    
    Serial.print( "Gain[0]: "); Serial.print(gain[0],10); Serial.print( " Gain[1]: ");Serial.print(gain[1], 10); Serial.print( " Gain[2]: ");Serial.println(gain[2], 10);
  }
}
void FilterPressure()
{
  float dt = 15.0f/1000.0f;
  
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
  
  est[0] = estp[0] + gain[0] * (bmp_data[3] - estp[0]);
  est[1] = estp[1] + gain[1] * (bmp_data[3] - estp[0]);
  est[2] = estp[2] + gain[2] * (bmp_data[3] - estp[0]);
  
  calculated_velocity = est[1];
  calculated_acceleration = est[2];
  filtered_altitude = est[0];
  
}
void CustomData()
{
   float tempArray[] = {bmp_data[0],filtered_altitude, calculated_velocity,new_millis_loop - old_millis_loop,max_altitude};
   transceiver_module.SendData(tempArray,5,CALC_VALUES_ID, (char)(((int)'0') + rocket_stage)); 
}
void StageZero()
{
  digitalWrite(DROGUE_CHUTE_TRANSISTOR, LOW);
  digitalWrite(MAIN_CHUTE_TRANSISTOR, LOW);
}
void StageOne()
{
  digitalWrite(DROGUE_CHUTE_TRANSISTOR, HIGH);
  digitalWrite(MAIN_CHUTE_TRANSISTOR, HIGH);
  if (filtered_altitude > 100.0f) // if altitude is greater than 50m
  {
    Serial.println("Stage Two" );
    transceiver_module.SendData(bmp_data,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocket_stage));
    CustomData();
    rocket_stage = STAGE_TWO;
  }
}

void StageTwo()
{
  if ( calculated_acceleration < 0 )
  {
    Serial.println("Stage Three" );
    transceiver_module.SendData(bmp_data,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocket_stage));
    CustomData();
    rocket_stage = STAGE_THREE;
  }
}

void StageThree()
{
  if ( calculated_velocity <= 0 )
  {
    Serial.println("Stage Four" );
    transceiver_module.SendData(bmp_data,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocket_stage));
    CustomData();
    digitalWrite(DROGUE_CHUTE_PIN, HIGH);  
    rocket_stage = STAGE_FOUR;
    filt_apogee_time = bmp_data[0];
  }
}

void StageFour()
{
  if ( filtered_altitude <= 1000.0f )
  {
    Serial.println("Stage Five" );
    transceiver_module.SendData(bmp_data,PRESSURE_ARRAY_SIZE,PRESSURE_SENSOR_ID, (char)(((int)'0') + rocket_stage));
    CustomData();
    digitalWrite(MAIN_CHUTE_PIN, HIGH);
    rocket_stage = STAGE_FIVE;
  }
}


void OutputDataArrays() {
  if (pressure_sensor_status && rocket_stage)
  {
      Serial.print(bmp_data[0]);
      Serial.print(" ms : ");
      Serial.print(bmp_data[1]);
      Serial.print(" hPa : ");
      Serial.print(bmp_data[2]);
      Serial.print("C : ");
      Serial.print(bmp_data[3]);
      Serial.print(" m: ");
      
      Serial.print(" Filt altitude: ");
      Serial.print(filtered_altitude);
      Serial.print(" m");
      Serial.print(" Calc Velocity: ");
      Serial.print(calculated_velocity);
      Serial.print(" m/s");
      Serial.print(" Actual Velocity: ");
      Serial.print(velocity);
      Serial.println(" m/s");
  }
  if (accelerometer_status && rocket_stage != STAGE_FIVE)
  {
    Serial.print(bmp_data[0]);
    Serial.print(" ms : ");
    Serial.print(accel_data[1]);
    Serial.print(" m/s^2 x:" );
    Serial.print(accel_data[2]);
    Serial.print(" m/s^2 y:");
    Serial.print(accel_data[3]);
    Serial.println(" m/s^2 z:");
  }
}
void SimulateValues()
{
  static bool simulation_done = false;
  static float apogee_time = 0.0f;
  static float start_millis = millis();
  static float actual_millis = 0.0f;
  static float dt = 0.0f;	
  static float t_accmax = .15f; // How long it takes the rocket to achieve max acceleration
  static float a_peak = 160; // Peak acceleration the rocket will achieve
  
  static float hold_a_max = 1.0f; // How long the rocket will hold max acceleration
  static float t_g = .5f;
  static float altitude = 0.01f;
  //static float velocity = 0.01f;
  static float acceleration = -9.81f; 
  static float acceleration_rate = (a_peak + 9.81f) / t_accmax; // The rate of acceleration when motor is giving thrust
  static float deacceleration_rate =  ( 9.81f + a_peak ) / -(t_g); // The rate of de-acceleration when motor is burnt out
  static float drogue_parachute_deacceleration_rate = ( 9.81f) / 5.0f; // The rate of de-acceleration when drogue chute is deployed
  static float standard_deviation_alt = 1.0f; // Expected sensor noise in meter
  static float max_sim_altitude = 0.1f;
  
  if (!simulation_done && rocket_stage > 0)
  {
      delay(20);
    //dt = millis() / 1000.0f - actual_millis;
      dt = 20.0f/1000.0f;
      //Serial.println(dt * 1000.0f);
      //Serial.println(start_millis);
      //actual_millis = (millis() - start_millis) / 1000.0f;
      actual_millis += dt;
      
      if (actual_millis < t_accmax) // Is the parachute motor burning
        acceleration = acceleration + acceleration_rate * dt;
      else if (actual_millis < hold_a_max) // Is the rocket at max acceleration
	acceleration = a_peak;
      else if (acceleration > -9.81f && velocity >= 0) // Is the rocket motor burnt out and the rocket is still ascending (i.e approaching apogee)
	acceleration = acceleration + deacceleration_rate * dt;
      else if (acceleration < 0.0f ) // Is it post apogee, if so apply drogue chute deacceleration rate
	acceleration = acceleration + drogue_parachute_deacceleration_rate * dt;
      else
        acceleration = 0.0f;
        
      float x1, x2, w, y1, y2;
      
      pressure_sensor_status = 1;
      accelerometer_status = 1;
      if (altitude <= 0.0f)
      {
	simulation_done = true;
	Serial.print("Simulation Over at ");
	Serial.println(actual_millis * 1000.0f);
		
	Serial.print("Simulation Apogee at ");
	Serial.println(apogee_time * 1000.0f);

        Serial.print("Filtered Simulation apogee at ");
        Serial.println(filt_apogee_time);
        
        Serial.print("Max filtered altitude is at ");
        Serial.println(max_altitude);
        
        Serial.print("Max simulated altitude is at ");
        Serial.println(max_sim_altitude);

        pressure_sensor_status = 0;
        accelerometer_status = 0;
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
      if ( max_sim_altitude < altitude )
        max_sim_altitude = altitude;
      bmp_data[3] = altitude + y1 * standard_deviation_alt;
      accel_data[3] = acceleration;
      bmp_data[0] = actual_millis * 1000 + millis();
      accel_data[0] = actual_millis * 1000;
      
  }
  else
  {
     pressure_sensor_status = 0;
     accelerometer_status = 0; 
  }

}
void CheckIfcommand_recieved()
{
  char currentCommand[25] = {'\0'};
  
   if ( transceiver_module.GetData(currentCommand,25) && currentCommand[0] == (char)START_BYTE )
  {
    Serial.println(currentCommand);
    if ( currentCommand[7] == 'U' && currentCommand[8] == 'U' && currentCommand[12] == 'S' && currentCommand[13] == '1' ) // Put rocket-commander in stage one(unlocked)
      rocket_stage = STAGE_ONE;
    else if ( currentCommand[7] == 'L' && currentCommand[8] == 'L' && currentCommand[12] == 'S' && currentCommand[13] == '0' ) // Put rocket-commander in stage zero (locked)
      rocket_stage = LOCKED_GROUND_STAGE;
    else if ( currentCommand[7] == 'S' && currentCommand[8] == 'M' && currentCommand[12] == 'S' && currentCommand[13] == '1' ) // Put rocket-commander in simulation mode
      simulation_on = 1;
    else if ( currentCommand[7] == 'S' && currentCommand[8] == 'M' && currentCommand[12] == 'S' && currentCommand[13] == '0' ) // Turn simulation mode off for rocket-commander
      simulation_on = 0;
    else if ( currentCommand[7] == 'D' && currentCommand[8] == 'A' && currentCommand[12] == 'D' && currentCommand[13] == 'A' ) // Fire drogue chute
      digitalWrite(DROGUE_CHUTE_PIN, HIGH); 
    else if ( currentCommand[7] == 'D' && currentCommand[8] == 'M' && currentCommand[12] == 'D' && currentCommand[13] == 'M' ) // Fire drogue chute
      digitalWrite(MAIN_CHUTE_PIN, HIGH); 
    else if ( currentCommand[7] == 'C' && currentCommand[8] == 'R' ) //  Get Rate
    {
      byte byte_array[2];
      byte_array[0] = (byte)currentCommand[12];
      byte_array[1] = (byte)currentCommand[13];
      
      send_rate = (byte_array[0] << 8) + byte_array[1];
      transciever_count = millis() / send_rate;
      Serial.println(send_rate);
      Serial.println(transciever_count);
    }
     
  }
}

