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
