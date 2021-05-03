/*
 * Read 2-axis bend sensor and output to serial, to be read by SerialInterface.
 * Uses the SparkFun library (copy / git clone to Arduio/libraries directory)
 * https://github.com/sparkfun/SparkFun_Displacement_Sensor_Arduino_Library
 * Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h"

ADS myFlexSensor1; // most distal segment (tip)
ADS myFlexSensor2;
ADS myFlexSensor3; // most proximal segment (base)

float data_float[6];

void setup()
{
  Serial.begin(115200);

  for (int i = 0; i < 6; i++)
  {
    data_float[i] = 0;
  }

  Wire.begin();
  if (myFlexSensor1.begin(10) == false)
  {
    Serial.println("cannot connect to sensor 1");
  }
  if (myFlexSensor2.begin(20) == false)
  {
    Serial.println("cannot connect to sensor 2");
  }
  if (myFlexSensor3.begin(30) == false)
  {
    Serial.println("cannot connect to sensor 3");
  }
}

void loop()
{
  // first check if sensor is connected, and only then get the data from the sensor (utilizing short-circuit evaluation)
  // signal filtering is done within ADS::available().
  // here, fix the mapping of the sensor to the model.
  /** @todo print error or warning when sensor is not connected? */
  if (myFlexSensor1.isConnected() && myFlexSensor1.available())
  {
    data_float[0] = myFlexSensor1.getX();
    data_float[1] = myFlexSensor1.getY();
  }
  if (myFlexSensor2.isConnected() && myFlexSensor2.available())
  {
    data_float[2] = myFlexSensor2.getX();
    data_float[3] = myFlexSensor2.getY();
  }
  if (myFlexSensor3.isConnected() && myFlexSensor3.available())
  {
    data_float[4] = myFlexSensor3.getX();
    data_float[5] = myFlexSensor3.getY();
  }

  for (int i=0; i < 6; i++){
    Serial.print(data_float[i]);
    if (i < 5)
      Serial.print(",");
  }
  Serial.println();
 
  // without this delay the sensor readings become very noisy for some reason?
  delay(10);
}
