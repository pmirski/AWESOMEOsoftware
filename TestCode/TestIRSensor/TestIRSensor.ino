#include <phys253.h>          
#include <LiquidCrystal.h>

void setup() {
  #include <phys253setup.txt>
  pinMode(A0,INPUT);
}

void loop() {
  delay(30);
  LCD.clear();
  LCD.home();

  if(digitalRead(A0) == HIGH){
    LCD.print("Driving");
  }else{
    LCD.print("Stopped");
  }

}
