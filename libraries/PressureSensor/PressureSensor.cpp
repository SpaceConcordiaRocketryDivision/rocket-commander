#include "PressureSensor.h"

PressureSensor::PressureSensor()
{
}
void PressureSensor::Init()
{
  
  #ifndef SIMULATION
	  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
	  seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
	  if(!bmp.begin())
	  {
		while(1);
	  }
  #endif
}
bool PressureSensor::GetData(float array[])
{
  #ifndef SIMULATION
	
	  sensors_event_t event;
	  bmp.getEvent(&event);

	  if (event.pressure)
	  {
		float temperature = 25.0f;

		//bmp.getTemperature(&temperature);
		array[0] = millis();
		array[1] = event.pressure;
		array[2] = temperature;
		array[3] = bmp.pressureToAltitude(seaLevelPressure,
											event.pressure,
											temperature);
		return 1; 
	  }
  #else
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
	if (!simulation_done)
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
			return 0;
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
		array[3] = altitude + y1 * standard_deviation_alt;
		array[1] = velocity;
		array[2] = acceleration;
		array[0] = actual_millis * 1000;
		return 1;
	}
  #endif
  return 0;
}
bool PressureSensor::SendData(float pressureToSet)
{
  seaLevelPressure = pressureToSet;
} 
