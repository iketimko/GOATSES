
#include <PID_v1.h>

void setup() {
  // put your setup code here, to run once:
  //Hardcoded Gain Values
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
  //Initial horizontal distance between user feet and actuator attachment point
  double x0 = 40; //[in]
  
  //Necessary Input Variables
  const int alpha_1Pin = xxx;
  const int alpha_2Pin = xxx;
  const int hPin = xxx;
  pinMode(alpha_1Pin,INPUT);
  pinMode(alpha_2Pin,INPUT);
  pinMode(hPin,INPUT);
  
  //Necessary Output Variables
  const int dXpin = xxx;
  pinMode(dXpin, OUTPUT);
  
}

void loop() {
  //Reading in the two attachment angles
  double alpha_1 = digitalRead(alpha_1Pin);//[pF]
  double alpha_2 = digitalRead(alpha_2Pin);//[pF]

  //Reading in the user's height
  double h = digitalRead(hPin);//[in]
  
  //Converting raw electrical signal values into angle measures
  alpha_1 = alpha_1/(0.274*10^-12);//[deg]
  alpha_2 = alpha_2/(0.274*10^-12);//[deg]

  //Calling the PID controller function
  double DeltaX = Cntrl(alpha_1,alpha_2,Kp,Ki,Kd,x0,h);
  
  //Outputting Commanded DeltaX shift value
  digitalWrite(dXpin,DeltaX)
}

double Cntrl(alpha_1,alpha_2,Kp,Ki,Kd,x0,h){ 
  //Defining the quantity beta
  double Beta = tan(alpha_2) - tan(alpha_2);//[NA]
  
  //Running the PID controller on the unequal angles
  PID(&Beta,&DeltaB,&0,Kp,Ki,Kd,DIRECT);
  
  //Converting change in Beta to change in X
  DeltaX = (-h + sqrt((h^2)/(deltaB^2) + x0^2))/deltaB; //[in]

  //Returning commanded shift in X
  return(DeltaX);
}
