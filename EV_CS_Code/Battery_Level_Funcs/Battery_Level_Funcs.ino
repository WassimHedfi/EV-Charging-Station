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
