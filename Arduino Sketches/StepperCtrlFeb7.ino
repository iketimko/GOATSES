
/* 
QHM
Feb 7, 2022
Using igus dryve D7 Stepper Motor Control System
Pin 6 is EN+
Pin 7 is DIR+
Pin 8 is STEP+
GND connects to EN-, DIR-, and STEP-
On the D7 Controller, the motor will connect to A+/A-/B+/B-
Power source connects to V+/V-

*/

const int StepsPerRev = 200;  // steps per revolution on the stepper motor
const float DistPerStep = 0.1; //distance moved by actuator for each step of the motor
// ^^DUMMY NUMBER FOR NOW - will have to figure this out for real when we have the actuator

int StepCount = 0; // use this to keep track of how many steps we've taken
float Position = 0; // use this to keep track of position

void setup() {
  // initialize the serial port:
  Serial.begin(9600);
  // set up pins as outputs
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  //set EN+ to positive
  digitalWrite(6, HIGH);
  delay(5); //wait 5 ms

}

// set up some test movement values to input to Move
  float dx = DistPerStep;
  float dx2 = DistPerStep*2;
  float dx3 = DistPerStep*3;
  float dx4 = DistPerStep*(-1);
  
void loop() {
  //do some different movements
  Move(dx); //one step positive
  delay(1000);
  Move(dx2); //two steps positive
  delay(1000);
  Move(dx3); //three steps positive
  delay(1000);
  Move(dx4); //one step negative
  delay(1000);
}

//this is the function we can call to move the motor
void Move(float dx){
//calculate number of steps we need to move
int stepsaway = (dx)/DistPerStep; //calculate how many steps we need to move to get to desired location


if (stepsaway < 0){ 
  digitalWrite(7, HIGH); //set direction pin to high if we're going in the negative direction
  stepsaway = stepsaway * (-1); //make this a positive value
  delay(1); //needs a delay of 5 microseconds, this is more than that
}
//going to make an if for stepsaway being positive as well - but i can come up with a cleaner way to do this with less delay later on
else { 
  digitalWrite(7, LOW); //set direction pin to high if we're going in the negative direction
  delay(1); //needs a delay of 5 microseconds, this is more than that
}

//now we need to have a square wave at STEP+ pretty fast for the correct number of times for stepsaway
for (int i = 1; i<=stepsaway; i++){
  digitalWrite(8, HIGH);
  delay(5); //this could probably be less but we'll start here)
  digitalWrite(8, LOW); //this is where it'll actually move
  delay(5);
}

}
