#include "Arduino.h"

/*Define the Pinout*/
#define trigPin 39
#define echoPin 38

/*define sound speed in m*/
#define SOUND_SPEED 340

/*Define the varibles required*/
long duration;
float distance;

void setup() {
  Serial.begin(115200); // Starts the serial communication
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  Serial.println("Ultrasonic Sensor is set");
  delay(10);
}

void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance (in m)
  distance = (duration * SOUND_SPEED/100)/2;
  
  // Prints the distance in the Serial Monitor
  Serial.print("Distance (cm): ");
  Serial.println(distance/100);

  delay(1000); //Change the delay if you want
}
