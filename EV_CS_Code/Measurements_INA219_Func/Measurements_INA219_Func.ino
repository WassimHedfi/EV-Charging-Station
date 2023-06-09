//____________________________________________      INA29      _______________________________________________
//____________________________________________  Current sensor  _______________________________________________
//____________________________________________     Get Data     _______________________________________________

void getData() {
  time_s = millis() / (1000); // convert time to sec
  busVoltage_V = ina219.getBusVoltage_V();
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  voltage_V = busVoltage_V + (shuntVoltage_mV / 1000);
  current_mA = ina219.getCurrent_mA();
  //power_mW = ina219.getPower_mW();
  power_mW = current_mA * voltage_V;
  energy_Wh = (power_mW * time_s) / 3600; //energy in watt hour
}
