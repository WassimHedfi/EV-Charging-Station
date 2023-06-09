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
