/*
  QHM + Cody Wheeler + Ike Timko + August Hauter
  Using igus dryve D7 Stepper Motor Control System, Using Bendlabs angle sensor data read and calibration

  For BendLabs:
  SCL to SCL
  SDA to SDA
  3.3V IN
  Ground

  For Lim Switch:
  Directionality for gates of Lim Switch: Digital 0,1 (0 is left switch, 1 for signal when right switch hit)
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
#include <Ewma.h> // BendLabs filtering library
#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_Displacement_Sensor
//#include "ArduPID.h" // this is the adrupid function in the library manager

//ArduPID Controller;
Ewma filtered_data(0.005);   // Exponentially Weighted Moving Average
ADS myFlexSensor;           //Create instance of the Angular Displacement Sensor (ADS) class
byte deviceType;            //Keeps track of if this sensor is a one axis of two axis sensor
float data;
const int StepsPerRev = 200;  // steps per revolution on the stepper motor
const float DistPerStep = 0.012; //distance moved by actuator for each step of the motor, in inches
int StepCount = 0;          //initialize step counter
int OutOfBounds = 0;            //Limit Switch pressed boolean
int Nsteps = 0;             //Number of steps taken counter
int Steps2Take = 1;         //Number of commanded steps to take by motor
const int GateL = 0;        // Left Limit Switch pin
const int GateR = 1;        // Right Limit Switch pin
int stopperRight = 1;       // Stop signal for pressed right limit switch
int stopperLeft = 2;        // Stop signal for pressed left limit switch
boolean isCentered = 0;     // Centered calibration flag

// CONTROL LAW STUFF
// Hardcoded Gain Values for PID
//double Kp = .1;
//double Ki = 0;
//double Kd = 0;
//double Beta = 0;
// Initialize dx and dB as 0
//double DeltaB = 0;
float dx = 0;
float alpha_des;
// Initial horizontal distance between user feet and actuator attachment point, and heigth of attachment point
float h;
float x0; //[in]
float x;

void setup() //*********************************************************************************************************************
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Beginning Test Setup/Calibration");

  // **************************
  // Limit switch stop check

  pinMode(GateL, INPUT);
  pinMode(GateR, INPUT);

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
  delay(1000);

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
  Center();
  //isCentered == 1 implies that we've centered the actuator
  isCentered = 1;

  // END Actuator Centering
  // **************************

  // **************************
  // For Stepper Motor PID
  // Get user height
  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  Serial.println("Please enter the height of the tether attachment point on the user (from the actuator attachment) (in):");
  while (Serial.available() == 0)
  {
    myFlexSensor.available();
    delay(10); //Wait for user to press character
  }
  h = Serial.parseInt();

  while (Serial.available() > 0)
    Serial.read(); //Flush all characters
  Serial.println("Please enter the distance between the attachement point on the user and actuator (in):");
  while (Serial.available() == 0)
  {
    myFlexSensor.available();
    delay(10); //Wait for user to press character
  }
  x0 = Serial.parseInt();
  x = x0;

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

  // loop to remove drift
  float temp;
    Serial.println("Looping ... ");
   for (int i = 0; i < 1500; i++)
    {
      temp = myFlexSensor.getX();
      delay(10);
    }  


  //100 point average of bendlabs angle
  float S_sum;
  float alpha;
  for (int i = 0; i < 500; i++)
  {
    alpha = myFlexSensor.getX();
    S_sum = S_sum + filtered_data.filter(alpha);  
    delay(10);
  }

  alpha_des = S_sum / 500; // Centered initial angle measurement between attachment point and user

  Serial.print("The NO Lean Tether Angle is: ");
  Serial.println(alpha_des);
  delay(1000);
  // Initialize controller
  //Controller.begin(&Beta,&DeltaB,&alpha_des,Kp,Ki,Kd);
  //Controller.start();

  // END Stepper Motor PID
  // **************************
}

void loop() 
{
  //Extra delay for best angle querying
  // **************************
  delay(10);
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
    Serial.println(data);
  }

  // END BendLabs Calibration and Data Reading
  // **************************

  // **************************
  // For Stepper Motor PID
  float prev_dx = dx;
  x = prev_dx + x;
  dx = 1 * control_loop(alpha_des, filtered, x);
//  Serial.print(", ");
//  Serial.println(dx);

  // END Stepper Motor PID
  // **************************

  // **************************
  // For Stepper Motor Command
  StepCount = Move(dx); //Function requires dx float (distance to move output by PID control

  // END Stepper Motor Command
  // **************************
}

// Bendlabs Calibration Function *************************************************************************************************
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

//  if (deviceType == ADS_TWO_AXIS)
//  {
//    while (Serial.available() > 0)
//      Serial.read(); //Flush all characters
//    Serial.println(F("Good. Now press a key when the sensor is straight from base but 90 degrees up from table (along Y axis)."));
//    while (Serial.available() == 0)
//    {
//      myFlexSensor.available();
//      delay(10); //Wait for user to press character
//    }
//
//    myFlexSensor.calibrateY(); //Call when sensor is straight on Y axis and 90 degrees on X axis
//  }
 

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
float control_loop(float alpha_des, float alpha_meas, float x)
{
  float DeltaX;

  // compute the PID response
  //Controller.compute();
  if (alpha_des < alpha_meas && abs(alpha_des-alpha_meas)>0.1)
  {
    DeltaX = -DistPerStep;
  }
  else
  {
    DeltaX = DistPerStep;
  }
  //Converting change in Beta to change in X
//  DeltaX = radians(h * sin(radians(90 - alpha_meas - degrees(asin(((x) / h) * (sin(radians(alpha_meas))))) - radians(alpha_des)))); //[in];
//
//  // Filter incorrect commands
//  if (abs(DeltaX) >= 20 * DistPerStep)
//  {
//    if (DeltaX > 0) {
//      DeltaX = 20 * DistPerStep;
//    }
//    else {
//      DeltaX = -20 * DistPerStep;
//    }
//  }

//  Serial.print(", ");
//  Serial.print(DeltaX);
  //Returning commanded shift in X
  return (-DeltaX);
}

//this is the function we can call to move the motor 
int Move(float dx)
{
  //calculate number of steps we need to move
  int stepsaway = (1 * dx) / DistPerStep; //calculate how many steps we need to move to get to desired location

  int StepCount = 0;
  StepCount += stepsaway;

  if (stepsaway < 0)
  {
    digitalWrite(7, HIGH); //set direction pin to high if we're going in the negative direction
    stepsaway = stepsaway * (-1); //make this a positive value
    delayMicroseconds(10); //needs a delay of 10 microseconds, this is more than that
  }
  //going to make an if for stepsaway being positive as well - but i can come up with a cleaner way to do this with less delay later on
  else
  {
    digitalWrite(7, LOW); //set direction pin to high if we're going in the negative direction
    delayMicroseconds(10); //needs a delay of 10 microseconds, this is more than that
  }

  //now we need to have a square wave at STEP+ pretty fast for the correct number of times for stepsaway
  for (int i = 1; i <= stepsaway; i++)
  {
    OutOfBounds = LimSwitch();
    if (OutOfBounds != 0 && isCentered == 1)
    {
      Serial.println("Actuator Went Out of Bounds, Ending Test for Safety.");
      exit(0);
    }
    //Square wave
    digitalWrite(8, HIGH);
    delay(1); //this could probably be less but we'll start here)
    digitalWrite(8, LOW); //this is where it'll actually move
    delay(1);
  }
  return StepCount;
}

// Limit Switch control function 
int LimSwitch()
{
  //Check the switch state
  //If either limit switch is pressed send the signal to stop all movement of the actuator
  if (digitalRead(GateL) == HIGH) {
    Serial.println("Left Bumper Hit");
    return stopperLeft;
  }
  else if (digitalRead(GateR) == HIGH) {
    Serial.println("Right Bumper Hit");
    return stopperRight;
  }
  //If no bumber is hit return logical zero
  else {
    return 0;
  }
}

// Limit Switch Actuator Centering Function 
void Center()
{
  //Actuator Calibration Step 1 (Move to One Side)
  while (OutOfBounds != stopperLeft)
  {
    //Call the function to check if either limit switch on an actuator is pressed
    OutOfBounds = LimSwitch();
    //Check to see if the actuator should halt movement
    if (OutOfBounds == stopperLeft) {
      //Reset OutOfBounds Variable
      OutOfBounds = 0;
      break;
    }
    else {
      //Move the actuator one step to one side
      StepCount = Move(DistPerStep);
    }
  }

  //Actuator Calibration Step 2 (Move to Other Side)
  while (OutOfBounds != stopperRight)
  {
    //Move the actuator one step to the opposite side and track number of steps
    StepCount = Move(DistPerStep * -1);
    Nsteps = Nsteps + 1;

    //Call the function to check if either limit switch on an actuator is pressed
    OutOfBounds = LimSwitch();
    //Check to see if the actuator should halt movement
    if (OutOfBounds == stopperRight) {
      //Reset OutOfBounds Variable
      OutOfBounds = 0;

      //Actuator Calibration Step 3 (Center the Actuator)
      //Move the actuator to center by moving it half the total number of steps between the limit switches
      Steps2Take = Nsteps / 2;
      StepCount = Move(Steps2Take * DistPerStep);
      Steps2Take = 1;
      break;
    }
  }
}
