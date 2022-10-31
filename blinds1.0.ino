#include <Adafruit_INA219.h>
#include <Servo.h>


int Temp_pin = A0;
int servo_pin = 10;
float temp_adjust = 4.5;
long closing_time = -1;
long current_pos;
int global_dir = 1; //
Adafruit_INA219 ina219;
Servo Servo;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(1);
  }
  Serial.println("Hello!");
  Servo.attach(servo_pin); //Attach the servo to the arduino digital pin

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  ina219.setCalibration_32V_1A();
  closing_time = initialize_blinds(); 

  delay(2000);
}

void loop() {
  decider();
  delay(300000); //wait 5 minutes
}

long initialize_blinds() {

  Servo.write(90 + 90 * global_dir);  // make sure its completely closed
  delay(300); // Time to let the servo spin up
  while (!motor_stall()) {}  //Keep detecting until stalled

  Servo.write(90);  //set the servo to stop
  current_pos = 100; //store the current curtain position
  delay(2000);

  long Time = millis(); //store starting time
  Servo.write(90 - 90 * global_dir);  //either 0/180
  delay(300);

  while (!motor_stall()) {}  //keep detecting until stalled

  long ElapsedTime = millis() - Time; //Calculate total time taken to open blinds
  Servo.write(90);  //set the servo to stop
  current_pos = 0; //store the current curtain position

  return ElapsedTime;
}

bool motor_stall() {
  float cutoff = 300;  //mA spike cutoff

  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  // Serial.print("Bus Voltage:   ");  Serial.print(busvoltage);   Serial.println(" V");
  // Serial.print("Shunt Voltage: ");  Serial.print(shuntvoltage); Serial.println(" mV");
  // Serial.print("Load Voltage:  ");  Serial.print(loadvoltage);  Serial.println(" V");
  // Serial.print("Current:       ");  Serial.print(current_mA);   Serial.println(" mA");
  // Serial.print("Power:         ");  Serial.print(power_mW);     Serial.println(" mW");
  // Serial.println("");

  return (current_mA > cutoff);
}

float local_temp() {
  int num_readings = 5;

  float temp_sums = 0;
  for (int i = 0; i < num_readings; i++) {
    float celcius = (analogRead(A0) * temp_adjust - 500) / 10;
    temp_sums = celcius + temp_sums;
  }
  return temp_sums / num_readings;
}

void motor_control(float pos) {
  int dir = 0;
  long time_to_take = 0;

  if (pos > current_pos) {
    time_to_take = ((closing_time / 100) * (pos - current_pos));
    dir = 1 * global_dir;
  } else {
    time_to_take = ((closing_time / 100) * (current_pos - pos));
    dir = -1 * global_dir;
  }

  long start_time = millis();
  long time_taken = 0;

  Servo.write(90 + 90 * dir);
  delay(300); // Time to let the servo spin up
  do {
    time_taken = millis() - start_time;
  } while ((time_taken < time_to_take) && !motor_stall());

  Servo.write(90);
  // Serial.print("Moved from "); Serial.print(current_pos); Serial.print(" to position "); Serial.println(pos);
  current_pos = pos;
}

void decider() {

  //static values since Wi-Fi connection does not work.
  float time = 14; // time in hours    
  float temp_preferred = 25;
  bool bright = true;  
  
  float tempI = local_temp(); //temp in celcius
//   Serial.print("Local temperture is "); Serial.println(tempI);


//   Serial.println("Please give current time");
//   float time = get_input();
//   Serial.print(time); Serial.println(" given as time.");

//   Serial.println("Please give brightness");
//   bool bright = get_input();   //get brightness
//   Serial.print(bright); Serial.println(" given as brightness.");

//   Serial.println("Please give preferred temp");
//   float temp_preferred = get_input(); // TBA
//   Serial.print(temp_preferred); Serial.println(" given as preferred temperture.");

  if(time > 20 || time < 9) { //It is ~ night time
    motor_control(100);
    //return;
  } else {
    if (tempI > temp_preferred) { //inside temperature is hotter than the preferred temperature
      if (bright) {
        motor_control(100);
      } else {
        motor_control(0);
      }
    } else { // inside temperture is COLDER* than the preferred temperture
      if(bright) {
         motor_control(0);
      } else {
        //Do nothing
      }
    }
  }
}

int get_input() {
  while(Serial.available() == 0) {
  }
  int input = Serial.parseInt();
  // delay(1000);
  Serial.parseInt();
  return input; 
}
