#include <Arduino.h>
#include <Ewma.h>
#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor

Ewma filtered_data(0.01);   // Exponentially Weighted Moving Average
ADS myFlexSensor; //Create instance of the Angular Displacement Sensor (ADS) class
byte deviceType; //Keeps track of if this sensor is a one axis of two axis sensor
float data;

void setup()
{
  // **************************
  // for the bendlabs sensor
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("SparkFun Displacement Sensor Example");

  Wire.begin();
  deviceType = myFlexSensor.getDeviceType();
  
  if (myFlexSensor.begin() == false)
  {
    Serial.println(F("No sensor detected. Check wiring. Freezing..."));
    while (1)
      ;
  }
  
  calibrate();
  // END Bendlabs sensor
  // **************************
}

void loop()
{
  // get bandlabs sensor data
  if (myFlexSensor.available() == true)
  {
    data = myFlexSensor.getX();
    float filtered = filtered_data.filter(data);
    Serial.print(filtered);
    Serial.println();
  }


  delay(10);
}


// Bendlabs Calibration Function
void calibrate()
{
  Serial.println(F("Calibration routine"));

  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  Serial.println(F("Press a key when the sensor is flat and straight on a table"));
  while (Serial.available() == 0)
  {
    myFlexSensor.available();
    delay(10); //Wait for user to press character
  }

  myFlexSensor.calibrateZero(); //Call when sensor is straight on both axis

  if (deviceType == ADS_TWO_AXIS)
  {
    while (Serial.available() > 0)
      Serial.read(); //Flush all characters
    Serial.println(F("Good. Now press a key when the sensor is straight from base but 90 degrees up from table (along Y axis)."));
    while (Serial.available() == 0)
    {
      myFlexSensor.available();
      delay(10); //Wait for user to press character
    }

    myFlexSensor.calibrateY(); //Call when sensor is straight on Y axis and 90 degrees on X axis
  }

  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  Serial.println(F("Good. Now press a key when the sensor is flat on table but bent at 90 degrees (along X axis)."));
  while (Serial.available() == 0)
  {
    myFlexSensor.available();
    delay(10); //Wait for user to press character
  }

  myFlexSensor.calibrateX(); //Call when sensor is straight on Y axis and 90 degrees on X axis

  Serial.println(F("Calibration complete."));
}
