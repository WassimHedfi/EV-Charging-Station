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
/*
 * To circulate without pid regulation use this function
 * The circulation however will no be precise and hard to anticipate, hence the problems that may arise with the placement of the station, for a precise mechanical contact
 * However this issue should be adressred in the future work.
 */
