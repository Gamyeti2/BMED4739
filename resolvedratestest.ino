/*  Lab 2: Motor Control
    BMED 4739/6739: Medical Robotics
    Ryan Emadi, Kyle Huang, Michael Pavelchek, Dominique Rever
 */
 
// Define number of pulses per revolution
const int pulse_rev = 1600;
 
// Define stepper motor control pins
const int stepPinHighBase = 13; // STEP pin (+)
const int stepPinLowBase = 12; // STEP pin (-)
const int dirPinHighBase = 11;  // DIR pin (+)
const int dirPinLowBase = 10; // DIR pin (-)
const int stepPinHighArm1 = 9; // STEP pin (+)
const int stepPinLowArm1 = 8; // STEP pin (-)
const int dirPinHighArm1 = 7;  // DIR pin (+)
const int dirPinLowArm1 = 6; // DIR pin (-)
const int stepPinHighArm2 = 5; // STEP pin (+)
const int stepPinLowArm2 = 4; // STEP pin (-)
const int dirPinHighArm2 = 3;  // DIR pin (+)
const int dirPinLowArm2 = 2; // DIR pin (-)
 
// Include the Arduino Stepper.h library:
#include "Stepper.h"
 
// Define number of steps per rotation:
const int stepsPerRevolution = 2048;
 
// Wiring:
// Pin 8 to IN1 on the ULN2003 driver
// Pin 9 to IN2 on the ULN2003 driver
// Pin 10 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver
 
// Create stepper object called 'myStepper', note the pin order:
Stepper myStepper = Stepper(stepsPerRevolution, 45, 46, 47, 48);
 
String receivedData = "";
float list[5]; // Adjust size as needed
int index = 0;
 
void setup() {
  // Initialize the pins as outputs
  pinMode(stepPinHighBase, OUTPUT);
  pinMode(dirPinHighBase, OUTPUT);
  pinMode(stepPinLowBase, OUTPUT);
  pinMode(dirPinLowBase, OUTPUT);
 
  // Set initial motor direction
  digitalWrite(dirPinHighBase, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowBase, HIGH); // Inverse of dirPinHigh
  // Initialize the pins as outputs
  pinMode(stepPinHighArm1, OUTPUT);
  pinMode(dirPinHighArm1, OUTPUT);
  pinMode(stepPinLowArm1, OUTPUT);
  pinMode(dirPinLowArm1, OUTPUT);
 
  // Set initial motor direction
  digitalWrite(dirPinHighArm1, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowArm1, HIGH); // Inverse of dirPinHigh
  // Initialize the pins as outputs
  pinMode(stepPinHighArm2, OUTPUT);
  pinMode(dirPinHighArm2, OUTPUT);
  pinMode(stepPinLowArm2, OUTPUT);
  pinMode(dirPinLowArm2, OUTPUT);
 
  // Set initial motor direction
  digitalWrite(dirPinHighArm2, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowArm2, HIGH); // Inverse of dirPinHigh
 
 // Set the speed to 5 rpm:
  myStepper.setSpeed(5);
 
  // Begin Serial communication at a baud rate of 9600:
  Serial.begin(9600);
 
}
 
void loop() {

  // Collect from matlab
    Serial.flush();
    if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('\n'); // Read data until newline
        Serial.println("Data Received: " + receivedData);
        receivedData = receivedData + ",";
        // Parse the received data
        index = 0; // Reset index
        int startIdx = 0;
        for (int i = 0; i < receivedData.length(); i++) {
            if (receivedData[i] == ',' || i == receivedData.length() - 1) {
                String value = receivedData.substring(startIdx, i);
                while (value.length() > 0 && isWhitespace(value[0])) {
                  value.remove(0, 1); // Remove the first character
                }
                Serial.println(value);
                Serial.println("break");
                list[index] = value.toFloat();
                startIdx = i + 1;
                index++;
            }
        }
        for (int i = 0; i < sizeof(list)/sizeof(list[0]); i++) {
          Serial.println(list[i]);
          list[i] = list[i] / 6.28;
        }
        for (int i = 0; i < sizeof(list)/sizeof(list[0]); i++) {
          Serial.print("This is the size ");
          Serial.println((sizeof(list)/sizeof(list[0])));
          Serial.println(list[i]);
        }
    //Serial.flush();
if (list[1] >= 0) {
  digitalWrite(dirPinHighBase, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowBase, LOW); // Inverse of dirPinHigh
  // Move the motor 200 steps in one direction
  for (int i = 0; i < pulse_rev*list[1]; i++) {
    digitalWrite(stepPinHighBase, HIGH);
    digitalWrite(stepPinLowBase, LOW);
    delayMicroseconds(5000); // Adjust speed by changing the delay
    digitalWrite(stepPinHighBase, LOW);
    digitalWrite(stepPinLowBase, HIGH);
    delayMicroseconds(0);
  }
} else {
  digitalWrite(dirPinHighBase, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowBase, HIGH); // Inverse of dirPinHigh
  // Move the motor 200 steps in one direction
  for (int i = 0; i < pulse_rev*-1*list[1]; i++) {
    digitalWrite(stepPinHighBase, HIGH);
    digitalWrite(stepPinLowBase, LOW);
    delayMicroseconds(5000); // Adjust speed by changing the delay
    digitalWrite(stepPinHighBase, LOW);
    digitalWrite(stepPinLowBase, HIGH);
    delayMicroseconds(0);
}
}
  delay(3000); // Wait 3 seconds
 
 
 if (list[3] >= 0) {
   digitalWrite(dirPinHighArm2, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowArm2, LOW); // Inverse of dirPinHigh
  for (int i = 0; i < pulse_rev*list[3]; i++) {
    digitalWrite(stepPinHighArm2, HIGH);
    digitalWrite(stepPinLowArm2, LOW);
    delayMicroseconds(5000); // Adjust speed by changing the delay
    digitalWrite(stepPinHighArm2, LOW);
    digitalWrite(stepPinLowArm2, HIGH);
    delayMicroseconds(0);
  }
 } else {
digitalWrite(dirPinHighArm2, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowArm2, HIGH); // Inverse of dirPinHigh
  for (int i = 0; i < pulse_rev*-1*list[3]; i++) {
    digitalWrite(stepPinHighArm2, HIGH);
    digitalWrite(stepPinLowArm2, LOW);
    delayMicroseconds(5000); // Adjust speed by changing the delay
    digitalWrite(stepPinHighArm2, LOW);
    digitalWrite(stepPinLowArm2, HIGH);
    delayMicroseconds(0);
 }
 }
  delay(3000); // Wait 3 seconds
if (list[2] >= 0) {
 digitalWrite(dirPinHighArm1, HIGH); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowArm1, LOW); // Inverse of dirPinHigh
  for (int i = 0; i < pulse_rev*list[2]; i++) {
    digitalWrite(stepPinHighArm1, HIGH);
    digitalWrite(stepPinLowArm1, LOW);
    delayMicroseconds(5000); // Adjust speed by changing the delay
    digitalWrite(stepPinHighArm1, LOW);
    digitalWrite(stepPinLowArm1, HIGH);
    delayMicroseconds(0);
  }
} else {
digitalWrite(dirPinHighArm1, LOW); // HIGH for clockwise, LOW for counterclockwise
  digitalWrite(dirPinLowArm1, HIGH); // Inverse of dirPinHigh
  for (int i = 0; i < pulse_rev*-1*list[2]; i++) {
    digitalWrite(stepPinHighArm1, HIGH);
    digitalWrite(stepPinLowArm1, LOW);
    delayMicroseconds(5000); // Adjust speed by changing the delay
    digitalWrite(stepPinHighArm1, LOW);
    digitalWrite(stepPinLowArm1, HIGH);
    delayMicroseconds(0);
}
}
  delay(3000); // Wait 3 seconds
 

 if (list[4] >= 0) {
   // Step one revolution in one direction:
  Serial.println("clockwise");
  myStepper.step(stepsPerRevolution*list[4]);
  delay(500);
 } else {
  // Step one revolution in the other direction:
  Serial.println("counterclockwise");
  myStepper.step(stepsPerRevolution*list[4]);
  delay(500);
 }
 //Serial.flush();
}  

}