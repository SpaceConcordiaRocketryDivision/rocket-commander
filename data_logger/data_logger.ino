#include <DataLogger.h>
#include <Wire.h>

DataLogger dataLogger;
void setup()
{
  dataLogger.Init();
}


void loop()
{
  //Serial.println("Test");
  float array[] = {1, 2, 3};
  float array2[] = {4, 5, 98, 42};
  dataLogger.SendData('a', array, 3);
  dataLogger.SendData('b', array2, 4);
  Serial.println(6);
  //delay(1000);
}
