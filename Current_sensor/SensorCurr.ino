#include <Wire.h>

int sensor_PIN = 0;
float sensibility = 0.066; //sensibility in V/A
float current_value, irms, power;

void setup() {
  Serial.begin(9600);
}

void loop() {
  get_data();
  printData();
  delay(500);
}

void get_data(){
  current_value = get_current(); 
  irms = current_value*0.707;
  power = irms*220.0; // P=IV watts, check value
}

float get_current(){
  float sensorRead = analogRead(sensor_PIN) * (5.0 / 1023.0);
  float current = (sensorRead-2.5)/sensibility;
  return current;
}

void printData(){
  Serial.print("Current (A): ");
  Serial.println(current_value, 4);
  Serial.print("Irms (A): ");
  Serial.println(irms, 4);
  Serial.print("Power (W): ");
  Serial.println(power, 4);
}