#include <PressureSensor.h>
#include <Wire.h>
PressureSensor pressure_sensor;

float bmp_data[4] = {
  0};  // Pressure Temperature Altitude
void setup() {
  Serial.begin(115200);
  pressure_sensor.Init();
  int counter = 0;
  while(counter < 200)
  {
    counter++;
    pressure_sensor.GetData(bmp_data);
  }
  pressure_sensor.SendData(bmp_data[1]);
}

void loop() {
  pressure_sensor.GetData(bmp_data);
  Serial.print(bmp_data[0]);
  Serial.print(":");
  Serial.print(bmp_data[1]);
  Serial.print(":");
  Serial.print(bmp_data[2]);
  Serial.print(":");
  Serial.println(bmp_data[3]);
  
  
}
