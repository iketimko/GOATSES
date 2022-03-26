#include "ArduPID.h" // this is the adrupid function in the library manager

ArduPID Controller;

//Hardcoded Gain Values
double Kp = .9;
double Ki = .6;
double Kd = .5;
//Initial horizontal distance between user feet and actuator attachment point
double x0 = 40; //[in]
double setpoint = 512;
double Beta = 0;
double DeltaB;
float alpha_des;
int h;


// generate a time variable for the bendlabs sensor simulation
double t = 0;
  
void setup() {
  Serial.begin(9600);
  delay(5000);
  Serial.println("Please enter the user tether height in inches:");
  while (Serial.available() == 0) {
  }
  h = Serial.parseInt();
  //  float alpha_des = get_bendlabs_data();
  float pi = 3.14159; //Pi
  alpha_des = pi/4; // use 45deg for testing purposes
  // initialize the PID function
  Controller.begin(&Beta,&DeltaB,&setpoint,Kp,Ki,Kd);
}

void loop() {
  float alpha_meas = get_bendlabs_data();
  Serial.println(alpha_meas);
  double dx = control_loop(alpha_des, alpha_meas);
  Serial.println(dx);
  delay(100);
  t += .1;
}

double control_loop(float alpha_des, float alpha_meas){
  //Defining the quantity beta
  Beta = tan(alpha_meas) - tan(alpha_des);//[NA]
  
  // compute the PID response
  Controller.compute();
  
  //Converting change in Beta to change in X
  double DeltaX = (-h + sqrt(sq(h)/sq(DeltaB) + sq(x0)))/DeltaB; //[in]
  Serial.println(DeltaX);
  //Returning commanded shift in X
  return(DeltaX);
}


float get_bendlabs_data() {
  // define step time period
  int steplen = 10;
  
  // return 1
  if (t<steplen){
    return 1;
  }
  // return 0
  else if (t<(2*steplen)){
    return 0;
  }
  // reset
  else {
    t = 0;
    return 1;
  }
}
