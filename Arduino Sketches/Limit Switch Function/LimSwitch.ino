void setup() {
  // Setting the Baud Rate
  Serial.begin(9600);
  // Flag variable to prevent excess looping
  // flag = 0;
  
}

void loop() {
  //Call the function to check if either limit switch on an actuator is pressed
  OutOfBounds = LimSwitch();
  //Check to see if the actuator should halt movement
  if(OutOfBounds == 1){
    break
    }
  
}

int STOP = LimSwitch(){
  // Pin state variables
  const int Gate1 = xxx;
  const int Gate2 = xxx;
  pinMode(Gate1,INPUT);
  pinMode(Gate2,INPUT);
  //Stop signal if limit switch is pressed
  int stopper = 1;
  //Check the switch state
  //If either limit switch is pressed send the signal to stop all movement of the actuator
  if((digitalRead(Gate1) == HIGH) || (digitalRead(Gate2) == HIGH)){
    delay(20);
    return stopper;
    }
  }
