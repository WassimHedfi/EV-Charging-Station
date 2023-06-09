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
