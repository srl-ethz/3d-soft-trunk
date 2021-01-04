/*
 * Read 2-axis bend sensor and output to serial, to be read by SerialInterface.
 * Uses the SparkFun library
 * https://github.com/sparkfun/SparkFun_Displacement_Sensor_Arduino_Library
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
#define DATA_SIZE 26

ADS myFlexSensor; //Create instance of the ADS class
char data[DATA_SIZE];
float data_float[6];

void setup()
{
  Serial.begin(38400);
  
  // set footer bytes
  data[DATA_SIZE - 2] = '\r';
  data[DATA_SIZE - 1] = '\n';
  
  // set dummy data for unconnected sensors
  float dummy = 3.14;
  memcpy(&data[8], &dummy, sizeof(dummy));
  memcpy(&data[12], &dummy, sizeof(dummy));
  memcpy(&data[16], &dummy, sizeof(dummy));
  memcpy(&data[20], &dummy, sizeof(dummy));

  Wire.begin();
  if (myFlexSensor.begin() == false)
  {
    // No sensor detected. Check wiring. Freezing...
    delay(1);
  }
}

void loop()
{
  if (myFlexSensor.available())
  {
    data_float[0] = myFlexSensor.getX();
    data_float[1] = myFlexSensor.getY();
    memcpy(&data[0], &data_float[0], sizeof(data_float[0]));
    memcpy(&data[4], &data_float[1], sizeof(data_float[1]));
    
    Serial.write(data, DATA_SIZE);
  }
  delay(1);
}
