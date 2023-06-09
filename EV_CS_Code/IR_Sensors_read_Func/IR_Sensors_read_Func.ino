//____________________________________________              _______________________________________________
//____________________________________________  IR Sensors  _______________________________________________
//____________________________________________              _______________________________________________
void readIRSensors() {
  //Read the IR sensors and put the readings in irReadings array
  irReadings[0]  = digitalRead(sensor1); // Extreme Left
  irReadings[1]  = digitalRead(sensor2);
  irReadings[2]  = digitalRead(sensor3);
  irReadings[3]  = digitalRead(sensor4);
  irReadings[4]  = digitalRead(sensor5); //Extreme Right
}
