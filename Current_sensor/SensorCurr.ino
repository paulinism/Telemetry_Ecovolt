#include <Wire.h>

int sensorC_PIN = 0;
int sensorV_PIN = 0;
float sensibility = 0.066; //sensibility in V/A
float current_value, irms, power;

void setup() {
  Serial.begin(115200);
}

void loop() {
  get_data();
  printData();
  delay(500);
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void get_data(){
  sensorCRead = analogRead(sensorC_PIN) * (5.0 / 1023.0);
  current = (sensorRead-2.5)/sensibility;
  //irms = current*0.707;
  sensorVRead = analogRead(sensorV_PIN);
  voltage = value = fmap(sensorValue, 0, 1023, 0.0, 10.0);
  power = current*voltage; // P=IV watts
}

void printData(){
  Serial.print("Current (A): ");
  Serial.println(current_value, 4);
  Serial.print("Irms (A): ");
  Serial.println(irms, 4);
  Serial.print("Power (W): ");
  Serial.println(power, 4);
}
