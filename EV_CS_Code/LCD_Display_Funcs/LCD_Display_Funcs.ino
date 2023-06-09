//____________________________________________       _______________________________________________
//____________________________________________  LCD  _______________________________________________
//____________________________________________       _______________________________________________

void LCD_Navig_Mode() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) { // Check if it's time to update the display
    previousMillis = currentMillis;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Navigate Mode ");
    lcd.print("ON");

    lcd.setCursor(0, 1 );

    lcd.print("  SoC: ");
    lcd.print(Level);
    lcd.print("%");
  }
}
void LCD_Charge_Mode() {
  unsigned long currentMillisC = millis();
  if (currentMillisC - previousMillisC >= intervalC) {
    previousMillisC = currentMillisC;
    int level = Level;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Charging||");
    lcd.print(level);
    lcd.print("%");

    lcd.setCursor(0, 1 );

    lcd.print("Current in");
    lcd.print(current_mA);
    lcd.print("mA");
  }
}

void LCD_Search_Mode() {
  unsigned long currentMillisS = millis();
  if (currentMillisS - previousMillisS >= intervalS) {
    previousMillisS = currentMillisS;
    int level = Level;
    if (k == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP");
      k++;
    }
    if (k == 2) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP.");
      k++;
    }
    else if (k == 3) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP..");
      k++;
    }
    else if (k == 4) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP...");
      k++;
    }
    else if (k == 5) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP....");
      k++;
    }
    else if (k == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP");
      k++;
    }
    else if (k == 6) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down| ");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP.....");
      k++;
    }
    if (k == 7) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Battery Down|");
      lcd.print(level);
      lcd.print("%");

      lcd.setCursor(0, 1 );

      lcd.print("Finding CP......");
      k = 1;
    }
  }

}
