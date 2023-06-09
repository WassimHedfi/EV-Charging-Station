#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Adafruit_INA219.h>
#include "CircularBuffer.h"
const size_t num_previous_positions = 10;
CircularBuffer<float, num_previous_positions> previous_positions;

//Constants for current sensor INA29
Adafruit_INA219 ina219;
float voltage_V = 0, shuntVoltage_mV, busVoltage_V;
float current_mA = 0;
float power_mW = 0;
float energy_Wh = 0;
unsigned long time_s = 0;
unsigned long previous_time = 0;
unsigned long previous_timeR = 0;
unsigned long previous_timeS = 0;
unsigned long timer;
unsigned long duration_us = 0;

float analogVolt = 0.00;
float voltage = 0.00;
float Level = 70.00;

float timeStepDown = 0.01;
float timeStepUp = 0.001;

int forwardStep(100);

float min_SoC = 60.00;
float max_SoC = 70.00;



unsigned long lastUpdateTime = 0;
float stepSize = 0.2;
int timeInterval = 500;

int k = 1;



//____________________________________________________________________________INSTANCES
const int instances = 100;
float batt_state[instances];

unsigned long previousMillis = 0;
const long interval = 1000;
unsigned long previousMillisC = 0;
const long intervalC = 1000;
unsigned long previousMillisS = 0;
const long intervalS = 1000;

int head_start = 0;


int Black =  1;
int White =  0;




int sensorr =      9    ;   // for non PID circulation
int sensorl =        2   ;    // for non PID circulation
int sensorL ;
int sensorR;


#define MAX_MOTOR_SPEED       255
#define NORMAL_MOTOR_SPEED    80
#define SLOW_MOTOR_SPEED      80
#define turn_speed           120
#define forward_speed        100





// Constants for LCD
#define I2C_ADDR    0x27
LiquidCrystal_I2C lcd(0x27, 6, 2);
#define intput_pont_diviseur A2

int charging_time = 0;



// Constants for Servo
#define ServoAttach    5
Servo monServo;



// Constants for IR sensors
#define sensor1          2
#define sensor2          3
#define sensor3          4
#define sensor4          7
#define sensor5          9
int irReadings[5];





// Constants for PID and Motors
float pTerm, iTerm, dTerm;
int error;
int previousError;
float  kp = 0.7; //
float ki = 0.0001;
float kd = 0.003; //
float output;
int integral,  derivative;


int motor1Forward = 13;
int motor1Backward = 12;
int motor1pwmPin = 11;
int motor2Forward = 8;
int motor2Backward = 10;
int motor2pwmPin = 6;

int ENABLE_1_PIN =  11;
int MOTOR_1_INPUT_1  = 12  ;
int MOTOR_1_INPUT_2 = 13 ;
int MOTOR_2_INPUT_1 = 8 ;
int MOTOR_2_INPUT_2  = 10 ;
int ENABLE_2_PIN     = 6 ;

int motor1newSpeed;
int motor2newSpeed;
int motor2Speed = 160;
int motor1Speed = 140;

int OutputFactor = 40;

int connection;
int break_cond = 0;

// Constants for ultrasonic
#define echoPin A2
#define trigPin A1
float distance;
float previous_position ;
int best_position = 0;

long duration;


//_____________________________________________________________________________________________________________________
//________________________________________________           __________________________________________________________
//________________________________________________   SETUP   __________________________________________________________
//________________________________________________           __________________________________________________________
//_____________________________________________________________________________________________________________________
void setup() {
  Wire.begin();
  Wire.setClock(300000);
  ultrasonic();
  monServo.attach(ServoAttach);
  monServo.write(0);

  lcd.init();
  lcd.backlight();
  lcd.print(" EV-CS PFA-2023");
  delay(1000);
  lcd.clear();
  ultrasonic();
  lcd.print("   Check Batt");
  delay(1500);
  lcd.setCursor(0, 1);
  lcd.print("     ");
  lcd.print(Level);
  lcd.print("%");
  delay(3000);
  ultrasonic();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Check Tentacle");
  monServo.write(0);
  monServo.write(40);
  lcd.setCursor(0, 1);
  lcd.print("  45 deg Angle");
  delay(1000);
  ultrasonic();
  monServo.write(0);
  monServo.write(70);
  lcd.setCursor(0, 1);
  lcd.print("  90 deg Angle ");
  delay(1000);
  ultrasonic();
  monServo.write(40);
  lcd.setCursor(0, 1);
  lcd.print("  45 deg Angle");
  delay(500);
  ultrasonic();
  monServo.write(0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" EV-CS PFA-2023");
  lcd.setCursor(0, 1);
  lcd.print("Initialistation!");
  ultrasonic();


  //Declare all IR sensors  as inputs
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  //Declare Motors pins as outputs
  pinMode(motor1Forward, OUTPUT);
  pinMode(motor1Backward,  OUTPUT);
  pinMode(motor1pwmPin, OUTPUT);
  pinMode(motor2Forward, OUTPUT);
  pinMode(motor2Backward, OUTPUT);
  pinMode(motor2pwmPin, OUTPUT);


  //Declare I/O Ultrasonic pins
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);


  // Setup of INA219
  uint32_t currentFrequency;
  ina219.begin();

  delay(1000);
  ultrasonic();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" EV-CS PFA-2023");
  lcd.setCursor(0, 1);
  lcd.print("  =Ready To Go");
  delay(2000);
  lcd.clear();
  lcd.print(" EV-CS PFA-2023");
  lcd.setCursor(0, 1);
  lcd.print("    ===== Go    ");
  delay(1500);
  ultrasonic();
  lcd.clear();

}
//______________________________________________________________________________________________________________________
//________________________________________________            __________________________________________________________
//________________________________________________   CHUNKS   __________________________________________________________
//________________________________________________            __________________________________________________________
//______________________________________________________________________________________________________________________




//____________________________________________      INA29      _______________________________________________
//____________________________________________  Current sensor  _______________________________________________
//____________________________________________     Get Data     _______________________________________________

void getData() {
  time_s = millis() / (1000); // convert time to sec
  busVoltage_V = ina219.getBusVoltage_V();
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  voltage_V = busVoltage_V + (shuntVoltage_mV / 1000);
  current_mA = ina219.getCurrent_mA();
  //power_mW = ina219.getPower_mW();
  power_mW = current_mA * voltage_V;
  energy_Wh = (power_mW * time_s) / 3600; //energy in watt hour
}



//____________________________________________              _______________________________________________
//____________________________________________  ULTRASONIC  _______________________________________________
//____________________________________________              _______________________________________________
void ultrasonic() {

  // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

}


//____________________________________________              _______________________________________________
//____________________________________________  IR Sensors  _______________________________________________
//____________________________________________              _______________________________________________
void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  irReadings[0]  = digitalRead(sensor1); // Extreme Left
  irReadings[1]  = digitalRead(sensor2);
  irReadings[2]  = digitalRead(sensor3);
  irReadings[3]  = digitalRead(sensor4);
  irReadings[4]  = digitalRead(sensor5); //Extreme Right
}



//____________________________________________              _______________________________________________
//____________________________________________    Motors    _______________________________________________
//____________________________________________              _______________________________________________


void run_forward_1() {
  analogWrite(motor1pwmPin,  motor1newSpeed);
  digitalWrite(motor1Forward, HIGH);
  digitalWrite(motor1Backward,  LOW);
}
void run_forward_2() {
  analogWrite(motor2pwmPin,  motor2newSpeed);
  digitalWrite(motor2Forward, LOW);
  digitalWrite(motor2Backward,  HIGH);
}



//____________________________________________     PID     _______________________________________________
//____________________________________________    ERROR    _______________________________________________
//____________________________________________ CALCULATION _______________________________________________


void calculateError() {
  if ((irReadings[0] == Black) && (irReadings[1] ==  Black) && (irReadings[2] == Black) && (irReadings[3] == Black) && (irReadings[4] == White)) {
    error = 4;
  } else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2]  == Black) && (irReadings[3] == White) && (irReadings[4] == White)) {
    error = 3;
  }  else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2] == Black) &&  (irReadings[3] == White) && (irReadings[4] == Black)) {
    error = 2;
  } else if  ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2] == White) && (irReadings[3]  == White) && (irReadings[4] == Black)) {
    error = 1;
  } else if ((irReadings[0]  == Black) && (irReadings[1] == Black) && (irReadings[2] == White) && (irReadings[3] == Black) &&  (irReadings[4] == Black)) {
    error = 0;
  } else if ((irReadings[0] == Black) &&  (irReadings[1] == White) && (irReadings[2] == White) && (irReadings[3] == Black) && (irReadings[4]  == Black)) {
    error = -1;
  } else if ((irReadings[0] == Black) && (irReadings[1]  == White) && (irReadings[2] == Black) && (irReadings[3] == Black) && (irReadings[4] == Black)) {
    error = -2;
  } else if ((irReadings[0] == White) && (irReadings[1] == White) &&  (irReadings[2] == Black) && (irReadings[3] == Black) && (irReadings[4] == Black)) {
    error  = -3;
  } else if ((irReadings[0] == White) && (irReadings[1] == Black) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black)) {
    error = -4;
  }
  else if ((irReadings[0] == White) && (irReadings[1] == White) && (irReadings[2]  == White) && (irReadings[3] == White) && (irReadings[4] == White)) {
    error = 0;
  }
  // else if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2]  == Black) && (irReadings[3] == Black) && (irReadings[4] == Black)) {
  // error = 0;}


  else  if ((irReadings[0] == Black) && (irReadings[1] == Black) && (irReadings[2] == Black) && (irReadings[3]  == Black) && (irReadings[4] == Black)) {
    if (previousError < 0)  {
      error = -100;
    } else if (previousError < 0) {
      error = 100;
    }
    else error = 0;
  }

}

//____________________________________________     PID     _______________________________________________
//____________________________________________    TERMS    _______________________________________________
//____________________________________________ CALCULATION _______________________________________________

void pidCalculations()  {

  pTerm = kp * error;
  integral += error;
  iTerm = ki * integral;
  derivative = error - previousError;
  dTerm = kd * derivative;
  output  = pTerm + iTerm + dTerm;
  previousError = error;
}


//____________________________________________    PID     _______________________________________________
//____________________________________________   MOTORS   _______________________________________________
//____________________________________________   OUTPUT   _______________________________________________
void changeMotorSpeed()  {

  motor2newSpeed = motor2Speed  + OutputFactor * output;
  motor1newSpeed = motor1Speed - OutputFactor * output;

  if (motor2newSpeed > 200 )motor2newSpeed = 200;
  if (motor1newSpeed > 200) motor1newSpeed = 200;
  if (motor2newSpeed < 0) motor2newSpeed = 0;
  if (motor1newSpeed < 0) motor1newSpeed = 0;

  //Set new speed, and run motors in forward  direction
  run_forward_1();
  run_forward_2();

}


//____________________________________________     PID    _______________________________________________
//____________________________________________    MAIN    _______________________________________________
//____________________________________________   =BLOCK=  _______________________________________________

void Circulation() {
  //Put all of our functions here
  readIRSensors();
  calculateError();
  pidCalculations();
  changeMotorSpeed();
}

//____________________________________________                  _______________________________________________
//____________________________________________    STOP Motors   _______________________________________________
//____________________________________________                  _______________________________________________

void stopBot() {
  digitalWrite(motor2Forward, LOW);
  digitalWrite(motor2Backward,  LOW);
  digitalWrite(motor1Forward, LOW);
  digitalWrite(motor1Backward,  LOW);

}


//________________________________________________                      __________________________________________________________
//________________________________________________  No_PID_Circulation  __________________________________________________________
//________________________________________________                      __________________________________________________________

void forward(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, LOW);
  digitalWrite(MOTOR_1_INPUT_2, HIGH);
  digitalWrite(MOTOR_2_INPUT_1, LOW);
  digitalWrite(MOTOR_2_INPUT_2, HIGH);
}



void right(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed + 20); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, LOW);
  digitalWrite(MOTOR_1_INPUT_2, HIGH);
  digitalWrite(MOTOR_2_INPUT_1, HIGH);
  digitalWrite(MOTOR_2_INPUT_2, LOW);
}

void left(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed + 20); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, HIGH);
  digitalWrite(MOTOR_1_INPUT_2, LOW);
  digitalWrite(MOTOR_2_INPUT_1, LOW);
  digitalWrite(MOTOR_2_INPUT_2, HIGH);
}
void stopBot(uint8_t speed) {
  analogWrite(ENABLE_1_PIN, speed); //Left Motor Speed
  analogWrite(ENABLE_2_PIN, speed); //Right Motor Speed
  digitalWrite(MOTOR_1_INPUT_1, LOW);
  digitalWrite(MOTOR_1_INPUT_2, LOW);
  digitalWrite(MOTOR_2_INPUT_1, LOW);
  digitalWrite(MOTOR_2_INPUT_2, LOW);
}



void circulation() {


  sensorL = digitalRead(sensorl);
  sensorR = digitalRead(sensorr);
  if ((sensorL == White) && (sensorR == White))
  {
    forward(forward_speed);


  }

  //RIGHT
  else if (( sensorL == White) && (sensorR == Black) )
  {
    right(turn_speed);


  }

  // LEFT
  else if ((sensorL == Black) && (sensorR == White))
  {
    left(turn_speed);
    delay(50);


  }

  else
  {
    forward(forward_speed);



  }
}





//____________________________________________              _______________________________________________
//____________________________________________  Batt Level  _______________________________________________
//____________________________________________              _______________________________________________

float mean(float values[], int n) {
  float sum = 0;
  for (int j = 0; j < n; j++) {
    sum += values[j];
  }
  return sum / n;
}
/*
  void getBattLevel()
  {

  for (int k = 0; k < instances; k++) {
    batt_state[k] = analogRead(A0);
  }
  analogVolt = mean (batt_state, instances);
  voltage = 1 + analogVolt * (5.00 / 1023.00) * 2 ;
  Level = (voltage - 5.00) * (100.00 - 0.00) / (9.80 - 5.00) + 0.00;

  }



  void getBattLevelDown(float timeStepDown) {
  Level = Level - timeStepDown;
  }

  void getBattLevelUp(float timeStepUp) {
  Level = Level + timeStepUp;
  }
*/
void getBattLevelDown(float& Level) {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime > timeInterval) {
    lastUpdateTime = currentTime;
    Level -= stepSize;
  }
}

void getBattLevelUp(float& Level) {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime > timeInterval) {
    lastUpdateTime = currentTime;
    Level += stepSize;
  }
}

//____________________________________________       _______________________________________________
//____________________________________________  LCD  _______________________________________________
//____________________________________________       _______________________________________________

void LCD_Navig_Mode() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { // Check if it's time to update the display
    previousMillis = currentMillis;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Navigate Mode ");
    lcd.print("ON");

    lcd.setCursor(0, 1 );

    lcd.print("  SoC: ");
    lcd.print(Level);
    lcd.print("%");
  }
}
void LCD_Charge_Mode() {
  unsigned long currentMillisC = millis();
  if (currentMillisC - previousMillisC >= intervalC) {
    previousMillisC = currentMillisC;
    int level = Level;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Charging||");
    lcd.print(level);
    lcd.print("%");

    lcd.setCursor(0, 1 );

    lcd.print("Current in");
    lcd.print(current_mA);
    lcd.print("mA");
  }
}

void LCD_Search_Mode() {
  unsigned long currentMillisS = millis();
  if (currentMillisS - previousMillisS >= intervalS) {
    previousMillisS = currentMillisS;
    int level = Level;
    if (k == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP");
      k++;
    }
    if (k == 2) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP.");
      k++;
    }
    else if (k == 3) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP..");
      k++;
    }
    else if (k == 4) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP...");
      k++;
    }
    else if (k == 5) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP....");
      k++;
    }
    else if (k == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP");
      k++;
    }
    else if (k == 6) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP.....");
      k++;
    }
    if (k == 7) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down|");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP......");
      k = 1;
    }
  }

}

//____________________________________________                      _______________________________________________
//____________________________________________  Serial test ina219  _______________________________________________
//____________________________________________                      _______________________________________________
void printData() {
  Serial.print("Bus Voltage:   "); Serial.print(busVoltage_V); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage_mV); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(voltage_V); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.print("Energy:        "); Serial.print(energy_Wh); Serial.println(" mWh");
  Serial.println("----------------------------------------------------------------------------");
}

//__________________________________________________________________________________________________________________
//________________________________________________        __________________________________________________________
//________________________________________________  LOOP  __________________________________________________________
//________________________________________________        __________________________________________________________
//__________________________________________________________________________________________________________________

void loop() {

  // put your main code here, to run repeatedly:
  /*
    //Test for a straight forward movement and adjust each motor speed accordingly
    motor1newSpeed = motor1Speed;
    motor2newSpeed = motor2Speed;
    run_forward_1();
    run_forward_2();
  */

  ultrasonic();
  previous_positions.push(distance);

  int i = 0;
  best_position = 0;
  if (Level > min_SoC ) {
    getBattLevelDown(Level);
    LCD_Navig_Mode();
    monServo.write(0);
    Circulation();
  }


  else if (Level < min_SoC) {
    ultrasonic();
    previous_positions.push(distance);
    int sum = 0;
    for (int l = 0; l < num_previous_positions; l++) {
      sum += previous_positions[i];
    }
    ultrasonic();
    best_position = (distance < 15.00 && sum > 400.00 ) ? 1 : 0;

  }


  if ((Level < min_SoC) && (best_position == 0)) {
    getBattLevelDown(Level);
    LCD_Search_Mode();
    monServo.write(0);
    Circulation();

  }


  else if ((Level < min_SoC) && (best_position == 1)) {
    current_mA = ina219.getCurrent_mA();
    while ((Level < max_SoC) && (best_position == 1)) {
      if (i == 0) {
        Circulation();
        delay(forwardStep);
        i = 1;
      }
      stopBot();
      LCD_Charge_Mode();
      monServo.write(70);
      getBattLevelUp(Level);
    }
    best_position = 0;

  }


  /* getData();
    printData();
    delay(700);

    Serial.println(voltage);
    Serial.println(Level);

    delay(500);*/

}
