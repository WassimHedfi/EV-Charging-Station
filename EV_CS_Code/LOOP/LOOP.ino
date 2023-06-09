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
