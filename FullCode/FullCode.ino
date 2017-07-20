//=======================================================================ARDUINO: INCLUDES=================================================================================    

#include <phys253.h>          
#include <LiquidCrystal.h>

//=======================================================================ARDUINO SETUP VARIABLES===================================================================================

int inPin = 0;

//=======================================================================VARIABLES: GENERAL================================================================================

int NumOfAgentsSaved = 0; // Number of agents in possession
int Black_Tape = 1;       // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0;       // See Black_Tape comment
int Mot_Speed_Stop = 0;   // Use this to stop motors


//=======================================================================VARIABLES: ONLY FOR FUNCTION driveWheels()==========================================

int Drive_Correction = 0;               // Initialization value, keep at 0. 
int Drive_Error = 0;                    // Initialization value, keep at 0.
int Drive_DerivError = 0;               // Initialization value, keep at 0.
int Drive_LastError = 0;                // Initialization value, keep at 0.

int Snsr_Drive_FrontInL = 0;            // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontInL = 8;
boolean Postn_Drive_Snsr_leftLast;      // Used to record last sensor value.
int Snsr_Drive_FrontInR = 0;            // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontInR = 9;
boolean Postn_Drive_Snsr_rightLast;     // Used to record last sensor value.
int Snsr_Drive_FrontOutL = 0;           // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontOutL = 10; 
int Snsr_Drive_FrontOutR = 0;           // Initialization value completely arbitrary
int Pin_Snsr_Drive_FrontOutR = 11;

int Snsr_Chassis_FrontCrnrR = 0;        // Initialization value completely arbitrary 
int Pin_Snsr_Chassis_FrontCrnrR = 42;
int ANALOGTHRESHOLD = 400;              // Used to utilize side-of-chassis sensors as if digital input re: sensing tape

int Mot_Wheels_Speed = 125;              // Wheels speed
int Pin_Mot_Wheels_L = 0;
int Pin_Mot_Wheels_R = 1;
int KD = 20;                            //Drive_Proporttional     16 or 20 was good value during testing
int KP = 16;                            //Drive_Derivative      16 or 20 was good value during testing

int count = 0;                          //USED ONLY FOR TESTING driveWheels() to display menu



//=======================================================================VARIABLES: FOR ROBOT STATE & VARIABLES SPECIFIC TO A STATE (indented)=========================================================================

int State_Register = 0; 

int State_WaitAtStart = 0;                   
int State_Go2IRGate = 1;     
    int Snsr_Laser = 99999;                //  TO DO:  Change when know if using. Random numbers right now!
    int Pin_Snsr_Laser = 99999;            //  TO DO:  Change when know if using. Random numbers right now!
    int Time_Go2IRGate = 2000;
int State_Wait4IRBeacon = 2;  
    int Snsr_IR_Gate; 
    int Pin_Snsr_IR_Gate;                   // TO DO:  Change when know if using. Random numbers right now! 
    int TenThousHz = 10000;
    int OneThousHz = 1000;              
int State_ApprchRamp = 3;  
    int Time_WhenTimerWasLastRead = 0;
    int Time_ApprchRamp = 5000;             // TO DO: determine this!!! Right now: 5 seconds. In milliseconds.             
int State_GoUpRamp = 4;   
    int Time_GoUpRamp = 5000;               // Three seconds. TO DO: determine this!!!
    int Mot_Wheels_Speed_Ramp = 100;
    int Mot_Wheels_Speed_Reset = Mot_Wheels_Speed;  //Used to reset Mot_Wheels_Speed when value changes                  
int State_AprchCircle = 5;
      int MotSlowDownFactor = 2.5;              
int State_EntrgCircle = 6;
/*  Commented out because right now we're starting at Line 2 and continuing on around circle.
int State_AprchLine1 = 7;                   // TO DO: determine if can put just one sensor in, far enough back
    int Pin_Snsr_TowerBottom_Line1 = 99999; // TO DO:  Change when know if using. Random numbers right now!
    int Snsr_TowerBottom_Line1 = 99999;     // TO DO:  Change when know if using. Random numbers right now!
*/
int State_AprchNextLine = 7;
    int Pin_Snsr_TowerBottom_Lines2to6_L = 99999; // TO DO:  Change when know if using. Random numbers right now!   
    int Snsr_TowerBottom_Lines2to6_L = 99999;     // TO DO:  Change when know if using. Random numbers right now!
    int Pin_Snsr_TowerBottom_Lines2to6_R = 99999; // TO DO:  Change when know if using. Random numbers right now!
    int Snsr_TowerBottom_Lines2to6_R = 99999;     // TO DO:  Change when know if using. Random numbers right now!
int State_Retrvl = 8;
    int RetrievalType;
    int WetRetrieval = 1;
    int DryRetrieval = 0;   
int State_Go2ZipLine = 9;
    int Pin_Snsr_ZiplineArrival = 9999999;        // TO DO:  Change when know if using. Random numbers right now!
    int Snsr_ZiplineArrival = 9999999;            // TO DO:  Change when know if using. Random numbers right now!
    int Postn_Robot_UnderZL = 1;                  //Arbitrary until know what it needs to be re: sensor reading once zipline trips sensor

int RepeatState = 0;                        //Don't = 0, Do = 1


//=======================================================================VARIABLES: ONLY FOR FUNCTION setTrolleyHorizontalPosition==========================================

int Postn_Trol_PrvsSnsr = 0;      // You must choose initialization value
int Snsr_Trol_Postn = 0;          // Initialization value completely arbitrary 
int Mot_Trol_Dirctn_Bkwrd = -1;
int Mot_Trol_Dirctn_Fwrd = 1; 
int Pin_Snsr_Trol = 0;            // Pin on TINAH. As per TINAH log
int Pin_Mot_Trol = 2;             // Pin on TINAH. As per TINAH log
int Mot_Trol_Speed = 65;         // Value is Mot_Trol_Speed/255 of maximum. User must choose initialization value.
int TrolBwdSpeedDivider = 1.3;    // Denominator of motor speed when Trol goes backward
int Mot_Trol_Dirctn = 1;          // Initialization value completely arbitrary 
int Postn_Trol_Destntn = 3;       // Values meanings correspond to those of Postn_Trol_Register. Initialization value completely arbitrary 
int Postn_Trol_Register = 2;      // TO DO: CHANGE THIS TO 0 AFTER TESTING. Values 0 to 4 correspond to trolley position (see Postn_Trol variables). First position: Postn_Trol_AtBoom = 0
int Postn_Trol_AtBoom = 0;
int Postn_Trol_TubRimMax = 1;
int Postn_Trol_OverBasket = 2;
int Postn_Trol_OverDryAgent = 3;
int Postn_Trol_MaxExtnsn = 4;     // TO DO: You only have 4 white tape pieces on jib now.


//=======================================================================VARIABLES: ONLY FOR FUNCTION setClawBlockVerticalPosition==========================================

int Postn_ClBlk_PrvsSnsr = White_Tape;    // Initialization value important (re: whether Snsr_ClBlk_Postn initial reading results in transition assumption.) 
int Snsr_ClBlk_Postn = 0;                 // Initialization value completely arbitrary
int Mot_ClBlk_Dirctn_Down = -1;
int Mot_ClBlk_Dirctn_Up = 1;
int Pin_Snsr_ClBlk = 1;                   // Pin on TINAH. As per TINAH log 
int Pin_Mot_ClBlk = 3;                    // Pin on TINAH. As per TINAH log
int Mot_ClBlk_Speed = 90;                 // Value is Mot_ClBlk_Speed/255 of maximum.  User must choose initialization value.
int ClbBlkDwdSpeedDivider = 1.3;            // Denominator of motor speed when ClBlk goes downward
int Mot_ClBlk_Dirctn = 1;                 // Initialization value completely arbitrary 
int Postn_ClBlk_Destntn = 2;              // Values correspond to those of Postn_ClBlk_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_ClBlk_Register = 0;             // Values 0 to 4 correspond to claw block position (see Postn_ClBlk variables). First position: Postn_ClBlk_AtJib = 0
int Postn_ClBlk_AtJib = 0;
int Postn_ClBlk_AtDryAgentHigh = 1;
int Postn_ClBlk_AtDryAgentMedium = 2;
int Postn_ClBlk_AtDryAgentLow = 3;
int Postn_ClBlk_AtWater = 4;


//=======================================================================VARIABLES: ONLY FOR FUNCTION setClawPosition. Using RCServo1.write(angle) as per TINAH log==========

//Important concerns: delay length, initial claw position
int Postn_Claw_Open = 0;
int Postn_Claw_HaveAgent = 1;
int Postn_Claw_Close = 2;
int Snsr_Claw_Postn = 0;                      // Analog pot. Initialization value completely arbitrary.  Values 0 to 2 correspond to the 3 position variables.
int Postn_Claw_Destntn = 0;                   // Values 0 to 2 correspond to the 3 position variables. Initialization value completely arbitrary 
int Postn_Claw_Register = 0;                  // Values 0 to 2 correspond to the 3 position variables. First position: Postn_Claw_Open = 0
double Snsr_Claw_CnvrsnRatio = (2.0/1023.0);  // ASSUMPTION: have an agent means half open, This is the conversion used to get 0,1 or 2 values for Snsr_Claw_Postn that results in 
int Mot_Claw_Angle_Close = 0;                 // ASCERTAIN THIS VALUE IS CORRECT
int Mot_Claw_Angle_Open = 90;                 // ASCERTAIN THIS VALUE IS CORRECT
int Pin_Snsr_Claw = 43;                       // Pin on TINAH. As per TINAH log ANALOG
int Mot_Claw_Dirctn = 0;                      // Initialization value completely arbitrary  


//=======================================================================VARIABLES: ONLY FOR FUNCTION setCranePosition. Using RCServo0.write(angle) as per TINAH log/=========

//Important concerns: delay length, initial crane position, claw block motor speed,
int Mot_Crane_Dirctn = 0;                // ARBITRARY, Direction to go in; 
int Postn_Crane_Destntn = 0;             // Values correspond to those of Postn_Crane_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Crane_Register = 0;            // Values 0 to 2 correspond to the 3 position variables (0,70,90). User must choose initialization value. (First position: ClawOpen)
int Postn_Crane_AngleBot = 90;            // ASCERTAIN THIS VALUE IS CORRECT
//int Postn_Crane_AngleTubLine1st = 45;      // ASCERTAIN THIS VALUE IS CORRECT
int Postn_Crane_AngleTubLineStd = 70;   // ASCERTAIN THIS VALUE IS CORRECT





//=======================================================================ARDUINO: SETUP===================================================================================

void setup()
{
  #include <phys253setup.txt>
  Serial.begin(9600);  
  pinMode(inPin, INPUT);
  RCServo0.write(90);
}


//=======================================================================ARDUINO: VOID LOOP===============================================================================

void loop()
{
  
  //Push start to go!
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStartToGo!");
    delay(50);
  }
  delay(5000);

  State_Register = State_Go2IRGate;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Go2IRGate");
  
    //==========================================State_Go2IRGate
  // Robot tape tracks for Time_ApprchRamp until at beginning of ramp.
  Time_WhenTimerWasLastRead = millis();  

  while(State_Register == State_Go2IRGate && !(millis() > (Time_WhenTimerWasLastRead + Time_Go2IRGate)) ){  
    driveWheels();
  }

  State_Register = State_ApprchRamp;

  LCD.clear();
  LCD.home();   
  LCD.print("State_ApprchRamp");

/*                                                                      NO HARDWARE YET!!!
    //========================================State_Wait4IRBeacon 
  while(Snsr_IR_Gate != TenThousHz){
    }

  State_Register = State_ApprchRamp;
*/

    //========================================State_ApprchRamp
  // Robot tape tracks for Time_ApprchRamp until at beginning of ramp.
  Time_WhenTimerWasLastRead = millis();

  while(State_Register == State_ApprchRamp && !(millis() > (Time_WhenTimerWasLastRead + Time_ApprchRamp)) ){  
    driveWheels();
  }

  State_Register == State_GoUpRamp;

  LCD.clear();
  LCD.home();   
  LCD.print("State_GoUpRamp");
  
    //========================================State_GoUpRamp
  // Robot tape tracks for Timing_MilliscndsApprchRamp milliseconds until ostensibly at ramp
  Time_WhenTimerWasLastRead = millis();

  if(State_Register == State_GoUpRamp){
    Mot_Wheels_Speed_Reset == Mot_Wheels_Speed;
    Mot_Wheels_Speed = Mot_Wheels_Speed_Ramp;
  }
    
  while( !( millis() > (Time_WhenTimerWasLastRead + Time_GoUpRamp) ) ){ 
    driveWheels();      
  }

  Mot_Wheels_Speed = Mot_Wheels_Speed_Reset;

  State_Register = State_AprchCircle;

  LCD.clear();
  LCD.home();   
  LCD.print("State_AprchCircle");
  
    //=========================================State_AprchCircle
  //Follow tape until Snsr_Drive_FrontOutR senses black tape. When does, pivot on right wheel (driving on left) until 
  //Snsr_Drive_FrontInL & Snsr_Drive_FrontInR see black again (i.e. are on the circle again). Then return to main void loop (i.e. stop driving).
  //Pause is to ensure doesn't immediately scan for black tape, because will see immediately. 

  Mot_Wheels_Speed = Mot_Wheels_Speed/MotSlowDownFactor;

  
  while(State_Register == State_AprchCircle){ 
    driveWheels();
    LCD.setCursor(0,1);
    LCD.print(Mot_Wheels_Speed);
    Snsr_Chassis_FrontCrnrR = analogRead(Pin_Snsr_Chassis_FrontCrnrR);

    if(Snsr_Chassis_FrontCrnrR > ANALOGTHRESHOLD){
      motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
      motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
      delay(1000);
      Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR);
      
      LCD.clear();
      LCD.home();
      LCD.print("Found R-Corner");
                                                                                //IT'S NOT WORKING BELOW THIS LINE, BABY
      while(Snsr_Drive_FrontInR == White_Tape){
        LCD.setCursor(0,1);
        LCD.print("Look 4 Tape");
        
        Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR);
        motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
        motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
      }
      LCD.clear();
      LCD.home();
      LCD.print("On line again");
      State_Register = State_AprchNextLine;   
                                                            //State_Register = State_AprchLine1;    // Commented out because we're NOT USING AprchLine1 at the moment.
    }
      
  }                                                         // READ THIS COMMENT: The indented comments near & above and below this line will be used if we decide to go to Line 1 first.

  ////Motor halt and time delay just for test
        motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
        motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  LCD.clear();
  LCD.home();
  LCD.setCursor(0,1);
  delay(5000);
  LCD.print("DONE.State_AprchNextLine");
  delay(10000);
                                                            //==========================================State_AprchLine1
                                                            //State_Register = State_AprchLine1 executed in previous code block. Here, robot follows tape until sensor at tower base senses first line.
                                                            //  while(State_Register == State_AprchLine1 && digitalRead(Pin_Snsr_TowerBottom_Line1) == White_Tape){
                                                            //    driveWheels();
                                                            //  }

  //==========================================State_AprchNextLine
  //Robot follows tape until sensors at tower base sense next line.
  //State_Register = State_AprchNextLine;

//   START OF JUST TESTING : stop at next line
//  while(digitalRead(Pin_Snsr_Drive_FrontOutL) == White_Tape && digitalRead(Pin_Snsr_Drive_FrontOutR) == White_Tape){
//    driveWheels();
//  }
  
  //END OF TESTING : below commented out because not used
  //while(State_Register == State_AprchNextLine && digitalRead(Pin_Snsr_TowerBottom_Lines2to6_L) == White_Tape && digitalRead(Pin_Snsr_TowerBottom_Lines2to6_R) == White_Tape){
  //  driveWheels();
  //}

    //==========================================State_Retrvl
  //Function called to retrieve agent at circle line, when perpendicular. Starts from turning crane to tub. Ends once claw is opened and drops agent
  doAgentRtrvl(DryRetrieval, Postn_ClBlk_AtDryAgentHigh);


    //===========================================State_Go2ZipLine
//Robot drives forward until zipline_arrival sensor is triggered
//State_Register == State_Go2ZipLine;

//    while(digitalRead(Pin_Snsr_ZiplineArrival) != Postn_Robot_UnderZL ){
//      motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
//      motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed);
//    }


//Delay just for test
  LCD.clear();
  LCD.home();
  LCD.setCursor(0,1);
  LCD.print("AtEndVoidLoop.10sDly");
  delay(10000);
                                            
}





//=======================================================================COMPOSITE FUNCTIONS (call more than one function) START HERE================================================

//=======================================================================COMPOSITE FUNCTION: doAgentRtrvl() =========================================================

// Incoming argument: RetrievalType, AgentHeight. RetrievalType must be specified as either DryRetrieval or WetRetrieval.
// If DryRetrieval, AgentHeight must be specified. For WetRetrieval, AgentHeight does not need to be specified.
// Function meant to be called when tower-base aligned with  is at one of lines . Function calls other functions.
void doAgentRtrvl(int FXN_RetrievalType, int FXN_ClBlk_Destntn) {
  int FXN_Trol_Destntn;
  int FXN_Crane_Destntn;


  if(FXN_RetrievalType == DryRetrieval){
  FXN_Trol_Destntn = Postn_Trol_OverDryAgent;
  }
  else {
  FXN_Trol_Destntn = Postn_Trol_MaxExtnsn;
  }

  LCD.clear();
  LCD.home();
  LCD.print("TrolDest: ");
  LCD.print(FXN_Trol_Destntn); 
  LCD.setCursor(0, 1);
  LCD.print("TrolReg: ");
  LCD.print(Postn_Trol_Register);
  delay(2000);
  
  setTrolleyHorizontalPosition(FXN_Trol_Destntn);

  FXN_Crane_Destntn = Postn_Crane_AngleTubLineStd;

  LCD.clear();
  LCD.home();
  LCD.print("CrnDest: ");
  LCD.print(FXN_Crane_Destntn);
  delay(2000);
  
  setCranePosition(FXN_Crane_Destntn);


  if(RetrievalType == WetRetrieval){
    FXN_ClBlk_Destntn = Postn_ClBlk_AtWater;
  }

  LCD.clear();
  LCD.home();
  LCD.print("CBDest: ");
  LCD.print(FXN_ClBlk_Destntn);
  LCD.setCursor(0,1);
  LCD.print("CBReg: ");
  LCD.print(Postn_ClBlk_Register);
  delay(2000);
  
  setClawBlockVerticalPosition(FXN_ClBlk_Destntn);

  if(RetrievalType == WetRetrieval){
    FXN_Trol_Destntn = Postn_Trol_TubRimMax;
    setTrolleyHorizontalPosition(FXN_Trol_Destntn);      
  }

/*
  Postn_Claw_Destntn = Postn_Claw_Close;
  LCD.setCursor(0,1);
  LCD.print("MvgClw2:");
  LCD.print(Postn_Claw_Destntn);
  setClawPosition(Postn_Claw_Destntn);
*/

  FXN_ClBlk_Destntn = Postn_ClBlk_AtJib;

  LCD.clear();
  LCD.home();
  LCD.print("CBDest: ");
  LCD.print(FXN_ClBlk_Destntn);
  LCD.setCursor(0,1);
  LCD.print("CBReg: ");
  LCD.print(Postn_ClBlk_Register);
  delay(2000);
  
  setClawBlockVerticalPosition(FXN_ClBlk_Destntn);


  FXN_Crane_Destntn = Postn_Crane_AngleBot;

  LCD.clear();
  LCD.home();
  LCD.print("PreFxnCrnDstn: ");
  LCD.print(FXN_Crane_Destntn);
  delay(2000);
  
  setCranePosition(FXN_Crane_Destntn);


  FXN_Trol_Destntn = Postn_Trol_OverBasket;

  LCD.clear();
  LCD.home();
  LCD.print("TrolDest:");
  LCD.print(FXN_Trol_Destntn);  
  LCD.print("TrolReg: ");
  LCD.print(Postn_Trol_Register);
  delay(2000);
   
  setTrolleyHorizontalPosition(FXN_Trol_Destntn);

/*
  Postn_Claw_Destntn = Postn_Claw_Open;
  LCD.setCursor(0,1);
  LCD.print("MvgClw2:");
  LCD.print(Postn_Claw_Destntn);
  setClawPosition(Postn_Claw_Destntn);
*/
  return;
}




//=======================================================================NON-COMPOSITE FUNCTIONS START HERE============================================================================



//=======================================================================FUNCTION: driveWheels()=====================================================================
/* 
  Function drives wheels while tape-tracking.
*/
void driveWheels() {

    //=======================================================================P.I.D. CODE BLOCK
  //Continuous sensor readings
  Snsr_Drive_FrontInL = digitalRead(Pin_Snsr_Drive_FrontInL); 
  Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR); 
  
  // Calculate proportional error correction component
  if(Snsr_Drive_FrontInL == Black_Tape && Snsr_Drive_FrontInR == Black_Tape){
    Drive_Error = 0;
  }
  if(Snsr_Drive_FrontInL == White_Tape && Snsr_Drive_FrontInR == Black_Tape){
    Drive_Error = -1;    //turn right
    Postn_Drive_Snsr_leftLast = White_Tape;
    Postn_Drive_Snsr_rightLast = Black_Tape;
  }
  if(Snsr_Drive_FrontInL == Black_Tape && Snsr_Drive_FrontInR == White_Tape ){
    Drive_Error = 1;    //turn left
    Postn_Drive_Snsr_leftLast = Black_Tape;
    Postn_Drive_Snsr_rightLast = White_Tape;
  }
  if(Snsr_Drive_FrontInL == White_Tape && Snsr_Drive_FrontInR == White_Tape){
    if(Postn_Drive_Snsr_rightLast == Black_Tape)
      Drive_Error = -5;   //turn hard right
    if(Postn_Drive_Snsr_leftLast == Black_Tape)
      Drive_Error = 5;    //turn hard left
  }

  //Calculate derivative error correction component
  Drive_DerivError = Drive_Error - Drive_LastError;
  
  //Sum correction components into correction variable
  Drive_Correction = Drive_Error*KP + Drive_DerivError*KD; //Original "int Drive_Correction = error*kp + Drive_DerivError*kd" , but extracted Extracted "+ intError*ki" and earler declaration " intError += error;"

  //Add correction to wheel motor speeds
  motor.speed(Pin_Mot_Wheels_L, (Mot_Wheels_Speed - Drive_Correction)*-1);
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed + Drive_Correction);
  
  //To note last error to use in next iteration
  Drive_LastError = Drive_Error;


 /*   //=======================================================================START: (SCREEN + KD,KP,SPEED CONTROLS); CODE FOR TESTING ONLY; CAN DELETE THIS CODE BLOCK
  //Screen Display
  count += 1;
  if(count%300 == 0){
    LCD.clear();
    LCD.home();
    LCD.setCursor(0, 0);
    LCD.print("KD: ");
    LCD.print(KP);
    LCD.print("  KP: ");
    LCD.print(KD);
    LCD.setCursor(0, 1);
    LCD.print("Speed: ");
    LCD.print(Mot_Wheels_Speed);
  }
  
  //Control KD, KP, & Mot_Wheels_Speed COMMENTED OUT, UNLESS USER WANTS TO CONTROL
//   KD = knob(6);                             //Drive_Prprtnl       16 or 20 was good value
//   KP = knob(7);                             //Drive_Dervtv      16 or 20 was good value
  */
  while (startbutton()){
     Mot_Wheels_Speed = Mot_Wheels_Speed + 10;
     delay(100);
   }
  
  while (stopbutton()){
    Mot_Wheels_Speed = Mot_Wheels_Speed - 10;
    delay(100);
  }
    //=======================================================================END: CODE FOR TESTING ONLY; CAN DELETE THIS CODE BLOCK

}


//=======================================================================FUNCTION: setCranePosition() =====================================================================

/* 
  Incoming argument: Postn_Crane_Destntn . Must have min value 0s and max value 180s.
  Sets position of crane.
*/
void setCranePosition(int FXN1_Crane_Destntn) {

  //Return if already at destination 
  if(FXN1_Crane_Destntn == Postn_Crane_Register){
    return;
  }

  //Set crane servo direction according to destination. 
  RCServo0.write(FXN1_Crane_Destntn); //sends desired angle change to servo

    //=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
  //Displays sensor's voltage and position register
  LCD.clear();  
  LCD.home();
  LCD.print("InFxnCrnDstn: ");
  LCD.print(FXN1_Crane_Destntn);
    //=======================================================================DELETE ABOVE; ONLY USED IN TESTING!!! DELETE AFTER
  
  delay(3000); //Delay to wait to ensure had time to turn
  Postn_Crane_Register = FXN1_Crane_Destntn; //No sensor; just assumes completed turn

  return;
}



//=======================================================================FUNCTION: setClawPosition() =====================================================================

/* 
  Incoming argument: Postn_Claw_Destntn . Must have min value 0 and max value 1 (open or closed).
  Opens or closes claw, based on input argument. Registers whether final state is open, closed, or has agent.
  If has agent, increments variable NumOfAgentsSaved.
*/
void setClawPosition(int FXN_Claw_Destntn) {

  //Set claw servo direction according to destination. Return if already at destination 
  if(FXN_Claw_Destntn = Postn_Claw_Register){
    return;
  }

  //Drive motor until destination, checking if arrived
  RCServo1.write(FXN_Claw_Destntn);//sends desired angle change to servo
  delay(500); //Delay to wait to ensure had time to close/open
  Snsr_Claw_Postn = round( analogRead(Pin_Snsr_Claw) * Snsr_Claw_CnvrsnRatio ); //This rounding results in value of 0,1,2, where Postn_Claw_Open = 0, Postn_Claw_HaveAgent = 1, Postn_Claw_Close = 2
  Postn_Claw_Register = Snsr_Claw_Postn;

  //Register if agent detected in grip
  if(Postn_Claw_Register == Postn_Claw_HaveAgent){
    NumOfAgentsSaved++;
  }

    //=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
    delay(1000); //This delay lets you see display command just before this funtion was called
  LCD.clear();  
  LCD.home();
  LCD.print("NumAgntsSvd: ");
  LCD.print(NumOfAgentsSaved);
  LCD.setCursor(0,1);
  LCD.print("ClawPostn: ");
  LCD.print(Postn_Claw_Register);
  delay(1000);
    //=======================================================================DISPLAY; DELETE ABOVE AFTER TESTING

  delay(1000); //This delay lets you see menu after destination state reached AND,
               //Keep this delay in (via counter in switch case) to not take off before agent falls in thing

  return;
}



//=======================================================================FUNCTION: setHorizontalTrolleyPosition() =====================================================================

/* 
  Incoming argument: Destination_ Trolley_Position . Must have min value 0 and max value 3.
  Destination_Trolley_Position compared with Postn_Trol_Register to see if higher or lower.
  Destination_Trolley_Position variable must have first value accurate (e.g. always start at TubRimMax) 
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers horizontal trolley 
  position and stops if same as destination.
*/
void setTrolleyHorizontalPosition(int FXN_Trol_Destntn) {

  //Set trolley motor direction according to destination. Return if already at destination 
  if(FXN_Trol_Destntn > Postn_Trol_Register)
    Mot_Trol_Dirctn = Mot_Trol_Dirctn_Fwrd;
  else if(FXN_Trol_Destntn < Postn_Trol_Register)
    Mot_Trol_Dirctn = Mot_Trol_Dirctn_Bkwrd;
  else if(FXN_Trol_Destntn == Postn_Trol_Register)
    return;

  //Drive motor until destination, checking if sense tape marker
  while(Postn_Trol_Register != FXN_Trol_Destntn) {
      if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_Bkwrd){
        motor.speed(Pin_Mot_Trol, (Mot_Trol_Speed*Mot_Trol_Dirctn)/TrolBwdSpeedDivider);
      }
      else {
        motor.speed(Pin_Mot_Trol, Mot_Trol_Speed*Mot_Trol_Dirctn);
      }
    Postn_Trol_PrvsSnsr = Snsr_Trol_Postn;
    Snsr_Trol_Postn = digitalRead(Pin_Snsr_Trol);

    //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
    if(Postn_Trol_PrvsSnsr != Snsr_Trol_Postn & Postn_Trol_PrvsSnsr == Black_Tape){
      if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_Fwrd)
        Postn_Trol_Register++;
      else
        Postn_Trol_Register--;
    }

    //=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SnsrRdg: ");
    LCD.print(digitalRead(Pin_Snsr_Trol));
    LCD.setCursor(0,1);
    LCD.print("TrolPostn: ");
    LCD.print(Postn_Trol_Register);
    //=======================================================================DELETE ABOVE; ONLY USED IN TESTING!!! DELETE AFTER

  }

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_Trol_Register == FXN_Trol_Destntn){
  motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
  }

  delay(1000);  //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER

  //return after while loop
  return;
}


//=======================================================================FUNCTION: setClawBlockVerticalPosition() =====================================================================

/* 
  Incoming argument: Destination_ Claw_Block_Position . Must have min value 0 and max value 2.
  Destination_ Claw_Block_Position compared with Postn_ClBlk_Register to see if higher or lower.
  Destination_ Claw_Block_Position must have first value accurate (e.g. always start at jib)  
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers claw block 
  position and stops if same as destination.
*/
void setClawBlockVerticalPosition(int FXN_ClBlk_Destntn) {

  //Set claw block motor direction according to destination. Return if already at destination 
  if(FXN_ClBlk_Destntn > Postn_ClBlk_Register)
    Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Down;
  else if(FXN_ClBlk_Destntn < Postn_ClBlk_Register)
    Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Up;
  else if(FXN_ClBlk_Destntn == Postn_ClBlk_Register)
    return;

  //Drive motor until destination, checking if sense tape marker
  while(Postn_ClBlk_Register != FXN_ClBlk_Destntn) {
    if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Down){
      motor.speed(Pin_Mot_ClBlk, (Mot_ClBlk_Speed*Mot_ClBlk_Dirctn)/ClbBlkDwdSpeedDivider);
    }
    else {
      motor.speed(Pin_Mot_ClBlk, Mot_ClBlk_Speed*Mot_ClBlk_Dirctn);
    }
    Postn_ClBlk_PrvsSnsr = Snsr_ClBlk_Postn;
    Snsr_ClBlk_Postn = digitalRead(Pin_Snsr_ClBlk);

    //If see tape marker transition (from black to white patch (which is position marker)), increment claw block position register according to motor direction
    if(Postn_ClBlk_PrvsSnsr != Snsr_ClBlk_Postn & Postn_ClBlk_PrvsSnsr == Black_Tape){
      if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Up)
        Postn_ClBlk_Register--;
      else
        Postn_ClBlk_Register++;
    }

    //=======================================================================DISPLAY; ONLY USED IN TESTING!!! DELETE AFTER
    //Displays sensor's voltage and position register
      LCD.clear();  
      LCD.home();
      LCD.print("SensorVoltage: ");
      LCD.print(Snsr_ClBlk_Postn);
      LCD.setCursor(0,1);
      LCD.print("ClBlkPostn: ");
      LCD.print(Postn_ClBlk_Register);
    //=======================================================================DELETE ABOVE; ONLY USED IN TESTING!!! DELETE AFTER

    delay(20);  //THIS DELAY LIMITS SPEED
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_ClBlk_Register == FXN_ClBlk_Destntn){
  motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);
  }

  delay(3000);  //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER

  //return after while loop
  return;
}
