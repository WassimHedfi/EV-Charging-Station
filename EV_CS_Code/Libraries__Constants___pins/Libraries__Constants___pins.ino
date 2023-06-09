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
