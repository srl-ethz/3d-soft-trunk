/*
 * Read 2-axis bend sensor and output to serial, to be read by SerialInterface.
 * Uses the SparkFun library (copy / git clone to Arduio/libraries directory)
 * https://github.com/sparkfun/SparkFun_Displacement_Sensor_Arduino_Library
*/

#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

ADS myFlexSensor; //Create instance of the ADS class
float data_float[6];
int num_data = sizeof(data_float) / sizeof(data_float[0]);

// set dummy data for unconnected sensors
float dummy = 0;

void signal_filter(float * sample)
{
  // copied from https://github.com/bendlabs/two_axis_ads/blob/master/examples/two_axis_ads_demo/two_axis_ads_demo.ino
  // LPF to filter out noise
  // does processing on first 2 items in input array
  static float filter_samples[2][6];
  for(uint8_t i=0; i<2; i++)
  {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)sample[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[i][0] = filter_samples[i][1]*(0.36952737735124147f) - 0.19581571265583314f*filter_samples[i][2] + \
      0.20657208382614792f*(filter_samples[i][3] + 2*filter_samples[i][4] + filter_samples[i][5]);

    sample[i] = filter_samples[i][0];
  }
}

void deadzone_filter(float * sample)
{
  // copied from https://github.com/bendlabs/two_axis_ads/blob/master/examples/two_axis_ads_demo/two_axis_ads_demo.ino
  // LPF to filter out noise
  static float prev_sample[2];
  float dead_zone = 0.5f;

  for(uint8_t i=0; i<2; i++)
  {
    if(fabs(sample[i]-prev_sample[i]) > dead_zone)
        prev_sample[i] = sample[i];
    else
        sample[i] = prev_sample[i];
  }
}

void setup()
{
  Serial.begin(38400);

  for (int i = 0; i < num_data; i++)
  {
    data_float[i] = dummy;
  }

  Wire.begin();
  if (myFlexSensor.begin() == false)
  {
    // No sensor detected. Check wiring. Freezing...
    delay(1);
  }
  // set sample rate to 100Hz, make sure it corresponds to the calulation in signal_filter() function.
  myFlexSensor.setSampleRate(ADS_100_HZ);
}

void loop()
{
  // only check data when new data is available
  if (myFlexSensor.available())
  {
    data_float[0] = myFlexSensor.getX();
    data_float[1] = myFlexSensor.getY();
    // process data to remove noise as much as possible
    signal_filter(data_float);
    deadzone_filter(data_float);

    Serial.print("a");
    for (int i=0; i < num_data; i++){
      Serial.print(data_float[i]);
      if (i != num_data-1)
        Serial.print(",");
    }
    Serial.println();
  }
  delay(1);
}
