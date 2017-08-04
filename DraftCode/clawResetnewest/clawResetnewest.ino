
void clawReset() {
  
  //Function resets Claw and boom. Trolley will be reset to furthest position from tower, Vertical translater/ClawBLK will be reset to bottom position
  //Function will time out after 2 seconds

  LCD.clear();  
  LCD.home();
  LCD.print("Claw Reset");

  unsigned long beginTime = millis(); //Record function start time
  unsigned long timeOut = 1000; //Timeout time
  
  RCServo0.write(Postn_Crane_AngleBot);   // crane
  RCServo1.write(Mot_Claw_Angle_Close);    // agent grabber

  RCServo2.detach();
  pinMode(Pin_SwitchMotTrol2Lift, OUTPUT);       // Relay now works (needs to be output)
  digitalWrite(Pin_SwitchMotTrol2Lift, HIGH);    // Set up relay so that trolley is controlled, not platform

  boolean VerticalFlag =false; //signifies whether vertical limit switch has been hit ie.claw has reached highest vertical position
  boolean HorizontalFlag =false; //signifies whether horizontal limit switch has been hit ie.claw has reached horizontal position furthest from boom

  //Do this while time in function is less than 2 seconds
  while(((millis() - beginTime) > timeOut) || (VerticalFlag==0 && HorizontalFlag==0)){
    LCD.clear();  
    LCD.home();
    LCD.print(millis() - beginTime);
    delay(30);
    
    if (VerticalFlag==false){
      if(digitalRead(Pin_Switch_ClBlk) == 1){ 
        //If the horizontal switch is not pressed, drive trolley outwards
        motor.speed(Pin_Mot_ClBlk, Mot_ClBlk_Speed); 
      }
      else{
        //If the switch is pressed stop powering motor and change flag to show switch has been pressed
        motor.speed(Pin_Mot_ClBlk, 0);
        VerticalFlag = true; 
      }
    }
    
    if (HorizontalFlag==false){
      if(digitalRead(Pin_Switch_Trol)==1) { 
        //If the horizontal switch has not been pressed, drive trolley outwards
        motor.speed(Pin_Mot_Trol,  Mot_Trol_Dirctn_ToEnd*Mot_ClBlk_Speed);
      }
      else{ 
        //Otherwise stop the motor and set the horizontal flag to true to show switch has been pressed
        motor.speed(Pin_Mot_Trol,  0); 
        HorizontalFlag = true;  
      }
    }
  }


  //Change both registers to show new positions
  Postn_ClBlk_Register = 0; 
  Postn_Trol_Register = 0; 
  
  setClawBlockVerticalPosition(3); //Move claw to bottom position
  
  return;
  
}


      
