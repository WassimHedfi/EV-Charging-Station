//____________________________________________                  _______________________________________________
//____________________________________________    STOP Motors   _______________________________________________
//____________________________________________                  _______________________________________________

void stopBot() {
  digitalWrite(motor2Forward, LOW);
  digitalWrite(motor2Backward,  LOW);
  digitalWrite(motor1Forward, LOW);
  digitalWrite(motor1Backward,  LOW);

}
