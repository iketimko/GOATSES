//QHM 1/27/22

#include <Stepper.h> //use library of stepper controls
const int stepsPerRevolution = 200;  // steps per revolution
const float distPerStep = 0.1; //distance moved by actuator for each step - will be used to calculate steps necessary
// ^^ABOVE NUMBER IS A DUMMY, I do not know how far the actuator will move for each step! (yet)
Stepper myStepper(stepsPerRevolution, 8, 9); // initialize the Stepper library on pins 8 through 11: - this is pins used in examples
int stepCount = 0;         // keep track of steps the motor has taken
float Position = 0;

void setup() {
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
  ActuatorDistance(3*distPerStep);
  float Position = stepCount*distPerStep;
  Serial.print(Position);
  delay(500);
}

//input to function is dx
void ActuatorDistance(float dx){
  
int stepsaway = (dx)/distPerStep; //calculate how many steps we need to move to get to desired location

//command actuator to move that many steps
myStepper.step(stepsaway); //I'm PRETTY sure this will work when it's negative
stepCount = stepCount + stepsaway; // modify stepcount to keep track of where we are now

delay(50); //we do need at least 5 microseconds between switching directions,so this is way more than we need

}
