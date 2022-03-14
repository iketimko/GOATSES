//Header//
//Author: August Hauter
//Team: GOATSES (2021-2022)
//Last Update: 3/8/22

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

void centerActuator(){
  //Variable to skip step 1 of centering if already touching one limit switch
  int skip = 0;
  //Direction of motion in step 1
  int Dir = 1;
  
  //Call the function to check if either limit switch on an actuator is pressed
  OutOfBounds = LimSwitch();
  //Move the actuator one step to one side
  if(OutOfBounds == 0){
    ActuatorDistance(distPerStep);
  }
  else if(OutOfBounds == 2){
    //Skip over step 1 since already at one side
    //Also reverse direction of step 2
    skip = 1;
    Dir = -1;
  }
  else{
    //Skip over step 1 since already at one side
    skip = 1;
    }
  
  //Actuator Calibration Step 1 (Move to One Side)
  while(OutOfBounds == 0 && skip == 0){
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
      ActuatorDistance(Dir*distPerStep);
      }
    }
    
  //Actuator Calibration Step 2 (Move to Other Side)
  while(OutOfBounds != 1){
    //Move the actuator one step to the opposite side and track number of steps
    ActuatorDistance(-Dir*distPerStep);
    Nsteps = Nsteps + 1;
    
    //Call the function to check if either limit switch on an actuator is pressed
    OutOfBounds = LimSwitch();
    //Check to see if the actuator should halt movement
    if(OutOfBounds == 1){
      //Reset OutOfBounds Variable
      OutOfBounds = 0;

      //Actuator Calibration Step 3 (Center the Actuator)
      //Move the actuator to center by moving it half the total number of steps between the limit switches
      float Steps2Take = Nsteps/2;
      ActuatorDistance(Steps2Take*Dir*distPerStep);
      //Reset of how many steps it will take to center the actuator to prevent errors
      //Steps2Take = 1;
      break
      }
    }  
  }
