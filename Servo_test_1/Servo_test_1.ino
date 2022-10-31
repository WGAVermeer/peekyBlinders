#include <Servo.h>
Servo name_servo;

void setup() {
  const int temperature = A0; //A0 is the analog pin  
  
  name_servo.attach(10);


  Serial.begin(9600);


}

void motor_move(int dir, float perc) {

  name_servo.write(90*(1.0 + dir * perc));
}

bool stall_detect() {
  float bound = 0.7; // Amps
  
}

void connect_to_internet() {

}

float get_weather_temp() {

}

void get_prefered_temp() {

}

float get_internal_temp() {

}


void loop() {
 
  // name_servo.write(180);
  // delay(2000);
  // name_servo.write(90);
  // delay(2000);
  // name_servo.write(0);
  // delay(2000);

  int Temp = analogRead(A0)*4.4;
  float celcius = (Temp - 500) / 10;
  //celcius = celcius - 0.5;
  //celcius = celcius * 100;

  Serial.println(celcius);
  Serial.println(Temp);   
  delay(1000); 
}