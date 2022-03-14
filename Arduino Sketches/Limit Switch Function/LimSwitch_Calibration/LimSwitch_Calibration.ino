//Header//
//Author: August Hauter
//Team: GOATSES (2021-2022)
//Last Update: 3/08/22

//Library of stepper controls
#include <Stepper.h> 
//Steps per revolution
const int stepsPerRev = 200;
//Initialize the stepper motor libraries on pins 8 & 9
Stepper myStepper(stepsPerRev, 8, 9);

void setup() {
  // Setting the Baud Rate
  Serial.begin(9600);
  // Flag variable to prevent excess looping
  // flag = 0;
  //Limit Switch pressed boolean
  int OutOfBounds = 0;
  //Number of steps taken counter
  int Nsteps = 0;
  //Number of commanded steps to take by motor
  int Steps2Take = 1;
  //Variable to skip step 1 calibration or not
  int skip = 0;
}

void loop() {
  //Call the function to check if either limit switch on an actuator is pressed
  OutOfBounds = LimSwitch();
  //Move the actuator one step to one side
  if(OutOfBounds != 1){
    myStepper.step(Steps2Take);
  }
  else{
    //Skip over step 1 since already at one side
    skip = 1;
  }
  
  //Actuator Calibration Step 1 (Move to One Side)
  while(OutOfBounds != 1 && skip == 0){
    //Call the function to check if either limit switch on an actuator is pressed
    OutOfBounds = LimSwitch();
    //Check to see if the actuator should halt movement
    if(OutOfBounds == 1){
      //Reset OutOfBounds Variable
      OutOfBounds = 0;
      break
      }
    else{
      //Move the actuator one step to one side
      myStepper.step(Steps2Take);
      }
    }
    
  //Actuator Calibration Step 2 (Move to Other Side)
  while(OutOfBounds != 1){
    //Move the actuator one step to the opposite side and track number of steps
    myStepper.step(-Steps2Take);
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
      myStepper.step(Steps2Take);
      Steps2Take = 1;
      break
      }
    }  
}

int STOP = LimSwitch(){
  // Pin state variables
  const int Gate1 = xxx;
  const int Gate2 = xxx;
  pinMode(Gate1,INPUT);
  pinMode(Gate2,INPUT);
  //Stop signal if limit switch is pressed
  int stopperRight = 1;
  int stopperLeft = 2;
  //Check the switch state
  //If either limit switch is pressed send the signal to stop all movement of the actuator
  if(digitalRead(Gate1) == HIGH){
    delay(20);
    return stopperRight;
    }
  else if(digitalRead(Gate2) == HIGH){
    delay(20);
    return stopperLeft;
    }
  else{
    delay(20);
    return 0;
    }
  }
