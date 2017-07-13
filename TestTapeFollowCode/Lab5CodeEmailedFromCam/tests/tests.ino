

#include <phys253.h>
#include <LiquidCrystal.h>

void setup() {
    #include <phys253setup.txt>
    Serial.begin(9600);
    //pinMode(INPIN, INPUT);
}


void loop() {
  
  LCD.clear();
  LCD.print('Hello');
  
}
