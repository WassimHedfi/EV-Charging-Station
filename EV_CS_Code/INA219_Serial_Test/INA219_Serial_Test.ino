//____________________________________________                      _______________________________________________
//____________________________________________  Serial test ina219  _______________________________________________
//____________________________________________                      _______________________________________________
void printData() {
  Serial.print("Bus Voltage:   "); Serial.print(busVoltage_V); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage_mV); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(voltage_V); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.print("Energy:        "); Serial.print(energy_Wh); Serial.println(" mWh");
  Serial.println("----------------------------------------------------------------------------");
}
