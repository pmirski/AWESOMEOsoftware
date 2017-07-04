double Menu_Destination_Ratio = 3.0/ 1023.0; //Ratio to convert knob(6) input in menu to get correct destination
double Menu_DCMotor_Ratio = 255.0 / 1023.0;    //Ratio to convert knob(6) input in menu to get correct DC motor speeds


void loop()
{

  while (!stopbutton()) {
  setClawBlockVerticalPosition(int Destination_Claw_Block_Position);
  setTrolleyHorizontalPosition(int Destination_Trolley_Position);
  delay(100)
  }

  if (stopbutton())
    {
    motor.stop_all();

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter Claw New Position");
      LCD.setCursor(0, 1);
      knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
      LCD.print(round(Menu_Destination_Ratio*knob6value));
      delay(100);
    }
    
    Destination_Claw_Block_Position = round(Menu_Ratio*knob6value); //prob hafta change this to int...YEA!
    delay(500);

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter ClawBlock Motor Speed");
      LCD.setCursor(0, 1);
      knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
      LCD.print(round(Menu_DCMotor_Ratio*knob6value));
      delay(100);
    }
    
    Motor_Speed_Claw_Block = round(Menu_Ratio*knob6value); //prob hafta change this to int...YEA!
    delay(500);

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter Trolley New Position");
      LCD.setCursor(0, 1);
      knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
      LCD.print(round(Menu_Destination_Ratio*knob6value));
      delay(100);
    }
    
    Destination_Trolley_Position = round(Menu_Ratio*knob6value); //prob hafta change this to int...YEA!
    delay(500);

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter Trolley Motor Speed");
      LCD.setCursor(0, 1);
      knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
      LCD.print(round(Menu_DCMotor_Ratio*knob6value));
      delay(100);
    }
    
    Motor_Speed_Trolley = round(Menu_Ratio*knob6value); //prob hafta change this to int...YEA!
    delay(500);
    
    }

}

