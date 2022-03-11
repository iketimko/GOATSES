/* 
QHM + Cody Wheeler + Ike Timko
Using igus dryve D7 Stepper Motor Control System, Using Bendlabs angle sensor data read and calibration
For BendLabs:
SCL to SCL
SDA to SDA
3.3V IN
Ground

For Stepper:
Ground
Pin 6 is EN+
Pin 7 is DIR+
Pin 8 is STEP+
GND connects to EN-, DIR-, and STEP-
On the D7 Controller, the motor will connect to A+/A-/B+/B-
Power source connects to V+/V-
Switches 4, 5, 6, 8 set ON

*/

#include <Arduino.h>
#include <Ewma.h>
#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
#include "ArduPID.h" // this is the adrupid function in the library manager

ArduPID Controller;
Ewma filtered_data(0.15);   // Exponentially Weighted Moving Average
ADS myFlexSensor; //Create instance of the Angular Displacement Sensor (ADS) class
byte deviceType; //Keeps track of if this sensor is a one axis of two axis sensor
float data;
const int StepsPerRev = 200;  // steps per revolution on the stepper motor
const float DistPerStep = 0.012; //distance moved by actuator for each step of the motor, in inches
int StepCount = 0; //initialize step counter

  //Hardcoded Gain Values for PID
  double Kp = 5;
  double Ki = 0;
  double Kd = 0;
  //Initial horizontal distance between user feet and actuator attachment point
  double x0 = 40; //[in]
  double setpoint;
  double Beta = 0;
  double DeltaB;
  float alpha_des;
  int h;
  
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

  // **************************
  // For Stepper Motor Control
  
  // set up pins as outputs
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  //set EN+ to positive
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(8, LOW);

  delay(5); //wait 5 ms

  // END Stepper Motor Control
  // **************************

  // **************************
  // For Stepper Motor PID

  Serial.println("Please enter the user tether height in inches:");
  while (Serial.available() == 0) {
  }
  h = Serial.parseInt();
  
  delay(2500);
  
  // initialize the PID function
  // Pause for test readiness to begin test
  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  Serial.println(F("Good. Now press a key when ready to begin the test."));
  while (Serial.available() == 0)
  {
    myFlexSensor.available();
    delay(10); //Wait for user to press character
  }
  double alpha = myFlexSensor.getX();
  setpoint = filtered_data.filter(alpha);
  //  float alpha_des = get_bendlabs_data();
  alpha_des = setpoint; // use zero for testing purposes
  Serial.println(setpoint);
  delay(5000);
  Controller.begin(&Beta,&DeltaB,&setpoint,Kp,Ki,Kd);
  Controller.start();

  // END Stepper Motor PID
  // **************************
}

void loop()
{
  // **************************
  // For Bendlabs Calibration and data reading
  float filtered;
  // get bandlabs sensor data
  if (myFlexSensor.available() == true)
  {
    data = myFlexSensor.getX();
    filtered = filtered_data.filter(data); // Gets input to PID Control law function
//    Serial.print(filtered);
//    Serial.print(", ");
//    Serial.println(data);
  }

  // END BendLabs Calibration and Data Reading
  // **************************

  // **************************
  // For Stepper Motor PID
   
  double dx = control_loop(alpha_des, filtered);
  Serial.println(dx);
  //Serial.println(dx);
  
  // END Stepper Motor PID
  // **************************

  // **************************
  // For Stepper Motor Command
  
  StepCount = Move(dx); //Function requires dx float (distance to move output by PID control
  
  // END Stepper Motor Command
  // **************************

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

// PID Control loop function
double control_loop(float alpha_des, float alpha_meas)
{
  //Defining the quantity beta
  Beta = tan(alpha_meas) - tan(alpha_des);//[NA]
  
  // compute the PID response
  //Controller.compute();
  DeltaB = Beta;
  //Converting change in Beta to change in X
  double DeltaX = (-h + sqrt(sq(h)/sq(DeltaB) + sq(x0)))/DeltaB; //[in];
  //Returning commanded shift in X
  return(DeltaX);
}

//this is the function we can call to move the motor
int Move(float dx)
{
  //calculate number of steps we need to move
  int stepsaway = (dx)/DistPerStep; //calculate how many steps we need to move to get to desired location
  int StepCount = 0;
  StepCount += stepsaway;

  if (stepsaway < 0)
  { 
    digitalWrite(7, HIGH); //set direction pin to high if we're going in the negative direction
    stepsaway = stepsaway * (-1); //make this a positive value
    delay(5); //needs a delay of 5 microseconds, this is more than that
  }
  //going to make an if for stepsaway being positive as well - but i can come up with a cleaner way to do this with less delay later on
  else 
  { 
    digitalWrite(7, LOW); //set direction pin to high if we're going in the negative direction
    delay(1); //needs a delay of 5 microseconds, this is more than that
  }

  //now we need to have a square wave at STEP+ pretty fast for the correct number of times for stepsaway
  for (int i = 1; i<=stepsaway; i++)
  {
    digitalWrite(8, HIGH);
    delay(1); //this could probably be less but we'll start here)
    digitalWrite(8, LOW); //this is where it'll actually move
    delay(1);
  }
  return StepCount;
}
