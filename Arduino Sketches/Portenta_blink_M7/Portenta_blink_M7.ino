/*
	Blink green LED using Portenta M7 Core
*/

const int ON = LOW; // Voltage level is inverted
const int OFF = HIGH;

void setup() {
  //bootM4(); // Boot M4 core
  pinMode(LEDG, OUTPUT); // Set green LED as output
  digitalWrite(LEDG, OFF); // Turn green LED off
}

void loop() {
 digitalWrite(LEDG, ON); // Turn green LED on
 delay(1000); // Wait for 1 second
 digitalWrite(LEDG, OFF); // Turn green LED off
 delay(1000);
}
