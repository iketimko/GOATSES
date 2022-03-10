//QHM 1/27/22
//this is for the 6-wire stepper motor and DS2003 control system
// not what we're using for the actual GOATSES but we don't have the driver yet so we'll use this to validate the integration of the rest of our software.


void setup() {
  
#include <Stepper.h> //use library of stepper controls
const int stepsPerRevolution = 200;  // steps per revolution
const float distPerStep = 0.1; //distance moved by actuator for each step - will be used to calculate steps necessary
// ^^ABOVE NUMBER IS A DUMMY, I do not know how far the actuator will move for each step! (yet)
Stepper myStepper(stepsPerRevolution, 6, 7, 8, 9); // initialize the Stepper library on pins 8 through 11: - this is pins used in examples
int stepCount = 0;         // keep track of steps the motor has taken
float Position = 0;
  // initialize the serial port:
  Serial.begin(9600);
  //test actuator distance command function
  float testdist = distPerStep*5;
  ActuatorDistance(testdist);
  //wait half a second
  delay(500);
  ActuatorDistance(-testdist); //it should move 5 steps forward, and then 5 steps back, for testing. 
}

void loop() {
  // 
  float testdist = distPerStep*5;
  float testdist2 = distPerStep*-5;
  ActuatorDistance(testdist);
  float Position = stepCount*distPerStep;
  Serial.print(Position);
delay(500);
  ActuatorDistance(testdist2);
  delay(200);
  ActuatorDistance(testdist*5);
  delay(500);
  ActuatorDistance(testdist2*5);
  delay(200);
}

//input to function is dx
void ActuatorDistance(float dx){
  
int stepsaway = (dx)/distPerStep; //calculate how many steps we need to move to get to desired location

//command actuator to move that many steps

//if negative
if (stepsaway<0){ //path if negative dirn
  stepsaway = stepsaway*(-1);
  for (int i = 1; i<stepsaway; i++){
  myStepper.step(-1); //take 1 step forward
  delay(10);
} //for, negative
}

//else, follow loop for positive
else{ //path if positive dirn
for (int i = 1; i<stepsaway; i++){
  myStepper.step(1); //take 1 step forward
  delay(10);
}
}
stepCount = stepCount + stepsaway;
delay(5); //we need at least 5 microseconds between switching directions - this if 5 milliseconds, so PLENTY

}
