#include <Wire.h>

volatile int interruptCounter;

int sensor_PIN = 0;
float sensibility = 0.066; //sensibility in V/A
float current_value, irms, power;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile void has_expired = false;

void setup() {
  Serial.begin(115200);
  setup_timer();
}

void loop() {
  if(has_expired)
  {
    get_data();
    printData();
    has_expired = false; 
  }
  //delay(500);
}

void setup_timer(){
  timer = timerBegin(0, 80, true); // Timer 0, divisor de reloj 80
  timerAttachInterrupt(timer, &timerInterrupcion, true); // Interruption function
  timerAlarmWrite(timer, 500000, true); // Interruption every 1/2 second
  timerAlarmEnable(timer); // Enable alarm, start timer
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

void IRAM_ATTR timerInterrupcion() {
  has_expired = true;
}