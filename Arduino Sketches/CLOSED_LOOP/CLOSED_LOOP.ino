/* 
QHM + Cody Wheeler + Ike Timko
Using igus dryve D7 Stepper Motor Control System, Using Bendlabs angle sensor data read and calibration
For BendLabs:
SCL to SCL
SDA to SDA
3.3V IN
Ground

For Lim Switch:
Left Lim Switch: Digital 0,1 (0 is open switch, 1 for signal when closed)
Right Lim Switch: Digital 3,4 (0 is open switch, 1 for signal when closed)
5V IN
Both Ground

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
// Flag variable to prevent excess looping
// flag = 0;
//Limit Switch pressed boolean
int OutOfBounds;
//Number of steps taken counter
int Nsteps = 0;
//Number of commanded steps to take by motor
int Steps2Take = 1;
//Variable to skip step 1 calibration or not
int skip = 0;
// Limit Switch Pin state variables, Lim switch left
const int Gate1 = 0;
const int Gate2 = 1;
// Limit Switch Pin state variables, Lim switch right
const int Gate3 = 2;
const int Gate4 = 3;

//Stop signal if limit switch is pressed
int stopperRight = 1;
int stopperLeft = 2;
  
  //Hardcoded Gain Values for PID
  double Kp = 1;
  double Ki = 0;
  double Kd = 0;
  //Initial horizontal distance between user feet and actuator attachment point
  double x0 = 40; //[in]
  double setpoint;
  double Beta;
  double DeltaB = 0;
  float alpha_des;
  int h;
  
void setup()
{
  Serial.begin(115200);
    while (!Serial)
    ;
  Serial.println("Beginning Test Setup/Calibration");
    
  // **************************
  // Limit switch stop check

  pinMode(Gate1,INPUT);
  pinMode(Gate2,INPUT);
  pinMode(Gate3,INPUT);
  pinMode(Gate4,INPUT);

  //END limit switch stop check
  // **************************
  
  // **************************
  // For the bendlabs sensor

  Wire.begin();
  deviceType = myFlexSensor.getDeviceType();
  
  if (myFlexSensor.begin() == false)
  {
    Serial.println(F("No BendLabs sensor detected. Check wiring. Freezing..."));
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
  // Actuator Centering

  Serial.println("Actuator Centering...");
  //Center();
  
  // END Actuator Centering
  // **************************

  // **************************
  // For Stepper Motor PID
  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  Serial.println("Please enter the user tether height in inches:");
  while (Serial.available() == 0) 
  {
    myFlexSensor.available();
    delay(10); //Wait for user to press character
  }
  h = Serial.parseInt();
  
  
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
  Beta = setpoint; // Initial BendLabs Sensor position
  
  Serial.print("The 0 Lean Tether Angle is: ");
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
  // IMU Data Collection

  // END IMU Data Collection
  // **************************
  
  // **************************
  // Limit switch stop check

  OutOfBounds = LimSwitch();
  if (OutOfBounds != 0)
  {
    Serial.println("Actuator Went Out of Bounds, Ending Test for Safety.");
    exit(0);
  }
  
  //END limit switch stop check
  // **************************
   
  // **************************
  // For Bendlabs Calibration and data reading
  
  float filtered;
  // get bandlabs sensor data
  if (myFlexSensor.available() == true)
  {
    data = myFlexSensor.getX();
    filtered = filtered_data.filter(data); // Gets input to PID Control law function
    Serial.print(filtered);
    Serial.print(", ");
    Serial.print(data);
  }

  // END BendLabs Calibration and Data Reading
  // **************************

  // **************************
  // For Stepper Motor PID
   
  double dx = control_loop(alpha_des, filtered);
  Serial.print(", ");
  Serial.println(dx);
  
  // END Stepper Motor PID
  // **************************

  // **************************
  // For Stepper Motor Command
  
  StepCount = Move(dx); //Function requires dx float (distance to move output by PID control
  
  // END Stepper Motor Command
  // **************************

  delay(5);
}

// Bendlabs Calibration Function
void calibrate()
{
  Serial.println(F("BendLabs Calibration routine"));

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

  Serial.println(F("BendLabsCalibration complete."));
}

// PID Control loop function
double control_loop(float alpha_des, float alpha_meas)
{

  //Defining the quantity beta
  Beta = tan(radians(alpha_meas)) - tan(radians(alpha_des));//[NA]
  double DeltaX;
  // compute the PID response
  Controller.compute();
  //Converting change in Beta to change in X
  DeltaX = (-h + sqrt(sq(h)/sq(DeltaB) + sq(x0)))/(DeltaB); //[in];
  if (abs(DeltaX)>=5)
  {
//    while (abs(DeltaX)>=5 || abs((DeltaB))==0)
//    {
//      // compute the PID response
      Controller.compute();
//      //Converting change in Beta to change in X
      DeltaX = (-h + sqrt(sq(h)/sq(DeltaB) + sq(x0)))/(DeltaB); //[in];
//    }
      DeltaX = 1;
  }
  Serial.print(", ");
  Serial.print(DeltaB);
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
    delay(1); //needs a delay of 5 microseconds, this is more than that
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
    delayMicroseconds(1); //this could probably be less but we'll start here)
    digitalWrite(8, LOW); //this is where it'll actually move
    delayMicroseconds(1);
  }
  return StepCount;
}

// Limit Switch control function
int LimSwitch()
{
  //Check the switch state
  //If either limit switch is pressed send the signal to stop all movement of the actuator
  if(digitalRead(Gate1) == HIGH){
    Serial.println("Left Bumper Hit");
    return stopperLeft;
    }
  else if(digitalRead(Gate2) == HIGH){
    Serial.println("Right Bumper Hit");
    return stopperRight;
    }
  else{
    return 0;
    }
  }

// Limit Switch Actuator Centering Function
void Center()
{
//Call the function to check if either limit switch on an actuator is pressed
  OutOfBounds = LimSwitch();
  //Move the actuator one step to one side
  if(OutOfBounds != 1){
    StepCount = Move(Steps2Take);
  }
  else{
    //Skip over step 1 since already at one side
    skip = 1;
  }
  
  //Actuator Calibration Step 1 (Move to One Side)
  while(OutOfBounds != 1 && skip == 0)
  {
    //Call the function to check if either limit switch on an actuator is pressed
    OutOfBounds = LimSwitch();
    //Check to see if the actuator should halt movement
    if(OutOfBounds == 1){
      //Reset OutOfBounds Variable
      OutOfBounds = 0;
      break;
      }
    else{
      //Move the actuator one step to one side
      StepCount = Move(Steps2Take);
      }
   }
    
  //Actuator Calibration Step 2 (Move to Other Side)
  while(OutOfBounds != 1)
  {
    //Move the actuator one step to the opposite side and track number of steps
      StepCount = Move(Steps2Take*-1);
    Nsteps = Nsteps + 1;
    
    //Call the function to check if either limit switch on an actuator is pressed
    OutOfBounds = LimSwitch();
    //Check to see if the actuator should halt movement
    if(OutOfBounds == 1){
      //Reset OutOfBounds Variable
      OutOfBounds = 0;

      //Actuator Calibration Step 3 (Center the Actuator)
      //Move the actuator to center by moving it half the total number of steps between the limit switches
      Steps2Take = Nsteps/2;
      StepCount = Move(Steps2Take);
      Steps2Take = 1;
      break;
      }
    }
}
