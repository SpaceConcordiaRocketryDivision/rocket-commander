#include "Accelerometer.h"

Accelerometer::Accelerometer() {
	Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(00001);
}
bool Accelerometer::Init() {
	return accel.begin();

}

boolean Accelerometer::GetData(float array[]) {
	sensors_event_t event;
	accel.getEvent(&event);

  //TODO: event if
	array[0] = millis();
	array[1] = event.acceleration.x;
	array[2] = event.acceleration.y;
	array[3] = event.acceleration.z;

	return 1;
}
