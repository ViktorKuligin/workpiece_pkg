
#include <LiquidCrystal.h>

const int rs = 32, en = 33, d4 = 34, d5 = 35, d6 = 3, d7 = 2;
LiquidCrystal lcd(32, 33, 34, 35, 36, 37);

void setup() {
  lcd.begin(16, 2);
  lcd.print("hello, world!");
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
}
