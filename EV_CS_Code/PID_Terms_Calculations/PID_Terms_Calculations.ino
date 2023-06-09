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
