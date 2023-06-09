
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
