/*
	Blink red LED using Portenta M4 Core
*/

const int ON = LOW; // Voltage level is inverted
const int OFF = HIGH;

void setup() {
  pinMode(LEDR, OUTPUT); // Set red LED as output
  digitalWrite(LEDR, OFF); // Turn red LED off
  digitalWrite(LEDG, OFF); // Turn green LED off
  digitalWrite(LEDB, OFF); // Turn blue LED off

}

void loop() {
//  digitalWrite(LEDR, ON); // Turn red LED on
//  delay(500); // Wait for 1/2 second
//  digitalWrite(LEDR, OFF); // Turn red LED off
//  delay(500);
}
