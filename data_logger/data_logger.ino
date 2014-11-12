#include <DataLogger.h>

#include <Wire.h>
DataLogger dataLogger;
void setup()
{
  
  dataLogger.Init();
  float array[] = {1, 2, 3};
  float array2[] = {4, 5, 98, 42};
  char array3[1000];
  //dataLogger.SendData('a', array, 3);
  Serial.println(7);
  //delay(1000);
  //dataLogger.SendData('b', array2, 4);
  Serial.println(8);
  //delay(1000);
  dataLogger.GetData(array3);
  Serial.println(array3);
  Serial.println(12);
}


void loop()
{
  //Serial.println("Test");
  /*float array[] = {1, 2, 3};
  float array2[] = {4, 5, 98, 42};
  dataLogger.SendData('a', array, 3);
  dataLogger.SendData('b', array2, 4);
  delay(1000);*/
  
}
