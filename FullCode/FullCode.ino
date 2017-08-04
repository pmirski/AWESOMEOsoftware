//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: INITIAL ARDUINO CODE*************************************************************************
//*****************************************************************************************************************************************************************************

//=======================================================================ARDUINO: INCLUDES=================================================================================    

#include <phys253.h>          
#include <LiquidCrystal.h>


//=======================================================================ARDUINO SETUP VARIABLES===================================================================================

int inPin = 0;


//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: GLOBAL VARIABLES*************************************************************************
//*****************************************************************************************************************************************************************************


//=======================================================================VARIABLES: GENERAL================================================================================

int Surface_Register = 0;                       //
  int Surface_GoLeftSurf = 0;                   // These three variables are used in user selection of which surface robot is on, and changing relevant variables based on user choice.
  int Surface_GoRightSurf = 1;                  //

int Mot_Wheels_Speed = 110;                     // Wheels speed   //~14.5V: 110
int Mot_Wheels_Speed_Reset = Mot_Wheels_Speed;  // Used to reset Mot_Wheels_Speed when value changes                  
int Mot_Speed_Stop = 0;                         // Use this to stop motors

int Mot_Wheels_Speed_Turn_LW;                   // A variable that may be selected when user chooses surface
int Mot_Wheels_Speed_Turn_RW;                   // A variable that may be selected when user chooses surface

int NumOfAgentsSaved = 0;                       // Number of agents in possession
int Black_Tape = 1;                             // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0;                             // See Black_Tape comment

int Pin_Mot_Wheels_L = 0;
int Pin_Mot_Wheels_R = 1;

unsigned long Time_BegngOfHeat;                           // Keeps track of time since beginning of heat
unsigned long Time_BeginWetRtrvls = 60000;                // After 1 minute, only wet retrievals are possible! (vs. dry retrievals)

int Snsr_Chassis_FrontCrnrR = 0;                // Initialization value completely arbitrary 
int Pin_Snsr_Chassis_FrontCrnrR = 42;
int Snsr_Chassis_FrontCrnrL = 0;                // Initialization value completely arbitrary 
int Pin_Snsr_Chassis_FrontCrnrL = 41;
unsigned long Time_TurnB4Sense = 1000;


//=======================================================================VARIABLES: FOR ROBOT STATE & VARIABLES SPECIFIC TO A STATE (indented)===============================

int State_Register = 0; 

int State_WaitAtStart = 0;                   

int State_Go2IRGate = 1;     
    unsigned long Time_Go2IRGate = 3900;              // 16.6V:3900 is PEFECT

int State_Wait4IRBeacon = 2;  
//    int Snsr_IR;                              // NOT USED AT THE MOMENT (night before time trial)
//    int Pin_Snsr_IR = 9999;                   // NOT USED AT THE MOMENT (night before time trial)
    boolean GoSignal;                           // Indicates signal from IR PCB. 0 = 1kHz, 1 = 10kHz

int State_ApprchRamp = 3;  
    unsigned long Time_WhenTimerWasLastRead = 0;
    unsigned long Time_ApprchRamp = 6000;            //

int State_GoUpRamp = 4;   
    unsigned long Time_GoUpRamp = 2000;              //
    int Mot_Wheels_Speed_Ramp = Mot_Wheels_Speed*(200.0/110.0);                 //~14.5V: 200
    unsigned long Time_SensorReadDelay = 500;
    unsigned long Time_DelayStart;

int State_AprchCircle = 5;
    double Mot_Wheels_Speed_UpperPltfrmSlowDownFactor = 1;       // The value here for the L-platform by default. It changes in selectSurface() for R-platform to diff value. SEE FUNCTION  
    int Pin_Snsr_Chassis_Corner_ApprchCircle;                    // A variable that may be selected when user chooses surface
    int Pin_Snsr_Chassis_Front_ApprchCircle;                     // A variable that may be selected when user chooses surface
    unsigned long Time_TapeFlw2EnsureOnLine = 1200;              //16.3V:1500, 16.5V:1200 (1500 makes it almost go over)
    int ANALOGTHRESHOLD;
    int ANALOGTHRESHOLD_FarRightSnsr = 630;                      // Symmetrical (re: other surface)
    int ANALOGTHRESHOLD_FarLeftSnsr = 780;                       // Symmetrical (re: other surface)


int State_EntrgCircle = 6;
    double Mot_Wheels_Speed_UpperPltfrmADDITIONALSlowDownFactor = 1; // The value here is for the L-platform by default. It changes in selectSurface() for R-platform to diff value. SEE FUNCTION        

int State_AprchNextLine = 7;
    unsigned long Time_AfterReadLineToStop = 0;           // TO DO:  Change when know if using. COMPLETE GUESS RIGHT NOW!
    unsigned long Time_AfterReadLineToCallApprchLineFxn = 0;

int State_Retrvl = 8;
    int RetrievalType;
    int WetRetrieval = 1;
    int DryRetrieval = 0;

int State_TurnToRaise = 9;
  unsigned long Time_Turn2Raise = 1500; 
  unsigned long Time_DriveFwd2Raise = 600; 

int State_RaisePltfrm = 10;
    int Pin_Mot_Pltfrm = 2;                       // Pin on TINAH. As per TINAH log (same as Mot_Trol)
    int Mot_Pltfrm_Speed_Up = 70;                // TO TEST: 
    int Mot_Pltfrm_Speed_Down = -70;             // TO TEST: 
    int Pin_Snsr_Pltfrm = 7;                      // Pin on TINAH. As per TINAH log (changed w Kurt July 25)
    int Postn_Pltfrm_Register;                    // TO DO: Values correspond to next two vars: Postn_Pltfrm_AtBtm & Postn_Pltfrm_AtZL  
    int Postn_Pltfrm_AtBtm = 1;                   // DONE: value ascertained
    int Postn_Pltfrm_AtZL = 0;                    // DONE: value ascertained

    int Pin_SwitchMotTrol2Lift = 34;                  // Switches relay for motor. Pin on TINAH. As per TINAH log  

int State_ApprchZL = 11;
    int Pin_Snsr_ZLArrvlSwtch = 6;              // Pin on TINAH. As per TINAH log
    int Postn_Robot_UnderZL = 0;                  // TO DO: Arbitrary until know what it needs to be re: sensor reading once zipline trips sensor
    int Postn_Robot_Register = 1;                 // This (0) initialization value matters
    int Mot_Speed_ApprchZL = 85;

int State_LowerPltfrm = 12;
  unsigned long Time_LowerPltfrm = 2000;          // TO TEST: determine this!!! 


//=======================================================================VARIABLES: FOR FUNCTION driveWheels()==========================================

int KD = 20;                            // Drive_Proporttional. UNTIL THU B4 COMPETITION WEEK, FOR L-SURFACE: 20
int KP = 16;                            // Drive_Derivative     UNTIL THU B4 COMPETITION WEEK: FOR L-SURFACE:16

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

int leftTurnBias  = -15;
int rightTurnBias =  15;
int NORMAL = 0;
int LEFT_CIRCLE_HASH = 1;
int RIGHT_CIRCLE_HASH = 2;

int count = 0;                          // USED ONLY FOR TESTING driveWheels() to display menu


//=======================================================================VARIABLES: FOR FUNCTION setTrolleyHorizontalPosition==========================================

int Postn_Trol_PrvsSnsr = 0;      // You must choose initialization value
int Snsr_Trol_Postn = 0;          // Initialization value completely arbitrary 
int Mot_Trol_Dirctn_ToEnd = -1;
int Mot_Trol_Dirctn_ToTwr = 1;
int Pin_Snsr_Trol = 0;            // Pin on TINAH. As per TINAH log
int Pin_Switch_Trol = 2;
  int Pin_Switch_Trol_Pressed = 0;       //0 is hi for limit switches
  int Pin_Switch_Trol_Unpressed = 1;
int Pin_Mot_Trol = 2;             // Pin on TINAH. As per TINAH log (same as Mot_Pltfrm)
int Mot_Trol_Speed = 90;          // changed from 80 during testing.  TO TEST: Value is Mot_Trol_Speed/255 of maximum. User must choose initialization value.
int Mot_Trol_SpeedReset = Mot_Trol_Speed;
int Trol_Speed_ToEndMultplier = 1;    // TO TEST: Denominator of motor speed when Trol goes backward
int Mot_Trol_Dirctn = 1;          // Initialization value completely arbitrary 
int Postn_Trol_Destntn = 3;       // Values meanings correspond to those of Postn_Trol_Register. Initialization value completely arbitrary 
int Postn_Trol_OverBasket = 2;
int Postn_Trol_OverDryAgent = 1;  // //GOT RID OF int Postn_Trol_TubRimMax = 1; AND CHANGED SUCH THAT FIRST TROL MOVE (in retrieval) IF WET RETRIEVAL ISN'T TO THIS 
int Postn_Trol_MaxExtnsn = 0;     // TO TEST: You only have 4 white tape pieces on jib now. Determine how many you want.
volatile int Postn_Trol_Register = Postn_Trol_MaxExtnsn;  // MAKE THIS MAX EXTENSION after testing. Values 0 to 4 correspond to trolley position (see Postn_Trol variables). First position: Postn_Trol_AtBoom = 0


//=======================================================================VARIABLES: FOR FUNCTION setClawBlockVerticalPosition==========================================

int Postn_ClBlk_PrvsSnsr = White_Tape;    // Initialization value important (re: whether Snsr_ClBlk_Postn initial reading results in transition assumption.) 
int Snsr_ClBlk_Postn = 0;                 // Initialization value completely arbitrary
int Mot_ClBlk_Dirctn_Down = -1;

int Pin_Switch_ClBlk = 3;
  int Pin_Switch_ClBlk_Pressed = 0;       //0 is hi for limit switches
  int Pin_Switch_ClBlk_Unpressed = 1;

int Mot_ClBlk_Dirctn_Up = 1;
int Pin_Snsr_ClBlk = 1;                   // Pin on TINAH. As per TINAH log
int Pin_Mot_ClBlk = 3;                    // Pin on TINAH. As per TINAH log
int Mot_ClBlk_Speed = 195;                 // TO TEST: Best value. Value is Mot_ClBlk_Speed/255 of maximum.  User must choose initialization value.
int Mot_ClBlk_SpeedReset = Mot_ClBlk_Speed;
int ClbBlkDwdSpeedDivider = 2.5;          // During testing, changed from 2.0 to 2.5. TO TEST: Best value. Denominator of motor speed when ClBlk goes downward
int Mot_ClBlk_Dirctn = 1;                 // Initialization value completely arbitrary 
int Postn_ClBlk_Destntn = 2;              // Values correspond to those of Postn_ClBlk_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_ClBlk_AtJib = 0;
int Postn_ClBlk_AtDryAgentHigh = 1;
int Postn_ClBlk_AtDryAgentMedium = 2;
int Postn_ClBlk_AtDryAgentLow = 3;
                                                                // Volatile for interrupt. NOTA BENE: We start with claw at max bottom to get under first gate.
volatile int Postn_ClBlk_Register = Postn_ClBlk_AtDryAgentLow;  // Values 0 to 4 correspond to claw block position (see Postn_ClBlk variables). First position: Postn_ClBlk_AtJib = 0


//=======================================================================VARIABLES: FOR FUNCTION setClawPosition. Using RCServo1.write(angle) as per TINAH log==========

/* 
 * Important concerns: delay length, initial claw position. 
 * CURRENTLY, SENSOR NOT USED, SO SOME CODE MAY BE IRRELEVANT.
 */

int Postn_Claw_Open = 0;
//int Postn_Claw_HaveAgent = 1;     //Commented out because we're not using right now
int Postn_Claw_Close = 1;           //Commented out because we're not using right now
int Snsr_Claw_Postn = 0;                      // Analog pot. Initialization value completely arbitrary.  Values 0 to 2 correspond to the 3 position variables.
int Postn_Claw_Destntn = 0;                   // Values 0 to 1 correspond to the 2 position variables. Initialization value completely arbitrary 
int Postn_Claw_Register = Postn_Claw_Open;                  // Values 0 to 1 correspond to the 2 position variables. First position: Postn_Claw_Open = 0
double Snsr_Claw_CnvrsnRatio = (2.0/1023.0);  // ASSUMPTION: have an agent means half open, This is the conversion used to get 0,1 or 2 values for Snsr_Claw_Postn that results in 
int Mot_Claw_Angle_Close = 12;                 // Done: this value is correct
int Mot_Claw_Angle_Open = 90;                // Done: this value is correct
int Mot_Claw_Angle_MaxOpen = 178;            
int Pin_Snsr_Claw = 43;                       // Pin on TINAH. As per TINAH log ANALOG
int Mot_Claw_Dirctn = 0;                      // Initialization value completely arbitrary  


//=======================================================================VARIABLES: FOR FUNCTION setCranePosition. Using RCServo0.write(angle) as per TINAH log/=========

/*
 * Important concerns: delay length, initial crane position, claw block motor speed,
 */

int Mot_Crane_Dirctn = 0;               // ARBITRARY, Direction to go in; 
int Postn_Crane_Destntn = 0;            // Values correspond to those of Postn_Crane_Register ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Crane_AngleBot = 90;
int Postn_Crane_AngleTubLineStd;   // Arbitrary initialization value   
    int Postn_Crane_AngleTubLineStd_RSurf = 150;   //TO DO: ASCERTAIN THIS VALUE IS CORRECT
    int Postn_Crane_AngleTubLineStd_LSurf = 30;  //TO DO: ASCERTAIN THIS VALUE IS CORRECT
int Postn_Crane_Register = Postn_Crane_AngleBot;  // Values 0 to 2 correspond to the 3 position variables (0,70,90). User must choose initialization value. (First position: ClawOpen)
int Postn_Crane_FarTurn;
    int Postn_Crane_FarRight = 178; 
    int Postn_Crane_FarLeft = 28;

//not using next four lines?
int Postn_Crane_IRMakeWay = 80;        // Arbitrary initialization value 
    int Postn_Crane_IRMakeWay_R = 80;  //TO DO: ASCERTAIN THIS VALUE IS CORRECT
    int Postn_Crane_IRMakeWay_L = 100; //TO DO: ASCERTAIN THIS VALUE IS 
unsigned long Time_CraneMvmtDelay = 1200;         // Delay to allow crane to adjust before initiating next movement





//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: ARDUINO SETUP*************************************************************************
//*****************************************************************************************************************************************************************************





void setup()
{
  #include <phys253setup.txt>
  Serial.begin(9600);  
  pinMode(inPin, INPUT);

  RCServo2.detach();
  pinMode(Pin_SwitchMotTrol2Lift, OUTPUT);       // Relay now works (needs to be output)
  digitalWrite(Pin_SwitchMotTrol2Lift, HIGH);    // Set up relay so that trolley is controlled, not platform

  RCServo0.write(Postn_Crane_AngleBot);   // crane
  RCServo1.write(Mot_Claw_Angle_Close);    // agent grabber

}





//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: ARDUINO VOID LOOP*************************************************************************
//*****************************************************************************************************************************************************************************




void loop()
{

      //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
//   Push start to continue.
//   User chooses motor speed
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStartToCont!");

  KP = (double) ( (50 / 1023.0)*knob(7));
  KD = (double) ( (50 / 1023.0)*knob(6));
  LCD.setCursor(0, 1);
  LCD.print("KP:");                                      
  LCD.print(KP);
  LCD.print(" KD:");                                      
  LCD.print(KD);


  delay(50);
  }

  delay(1000);  //Need delay so can see next message

    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK


  // Push start to go!
  // User chooses surace
  while (!startbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStart2Go!");
    
    LCD.setCursor(0, 1);
    LCD.print("SelctSurf: ");
    if(knob(7) < 500) {
      LCD.print("L");
      Surface_Register = Surface_GoLeftSurf;
    }
    else if(knob(7) >= 500) {
      LCD.print("R");
      Surface_Register = Surface_GoRightSurf;
    }    
  }                           


  selectSurface(Surface_Register);

                /*
                      
    //=======================================================================TEST CODE ONLY: STARTS HERE. MUST COMMENT OUT DEFINITELY

//while(1 != 0 ){
//
//  double testratio = 3.0/1023.0;
//  double speedratio = 254.0/1023.0;
//  double knob7value;
//  double knob6value; 
//  
//  while (!startbutton()) {
//    knob6value = knob(6)*speedratio;
//    Mot_ClBlk_Speed = (int) knob6value;
//    LCD.clear();
//    LCD.home();
//    LCD.print("Speed: ");
//    LCD.print(Mot_ClBlk_Speed);
//    knob7value = (knob(7)*testratio);
//    LCD.setCursor(0,1);
//    LCD.print("CBdes:");
//    LCD.print((int) knob7value);
//    LCD.print(" CBreg:");
//    LCD.print(Postn_ClBlk_Register);
//  }
//  setClawBlockVerticalPosition((int) knob7value);
//  delay(1000);
//}


//while(1 != 0 ){
//
//  double testratio = 2.0/1023.0;
//  double speedratio = 254.0/1023.0;
//  double knob7value;
//  double knob6value; 
//  
//  while (!startbutton()) {
//    knob6value = knob(6)*speedratio;
//    Mot_Trol_Speed = (int) knob6value;
//    LCD.clear();
//    LCD.home();
//    LCD.print("Speed: ");
//    LCD.print(Mot_Trol_Speed);
//    LCD.setCursor(0,1);
//    knob7value = (knob(7)*testratio);
//    LCD.print("TRdes:");
//    LCD.print((int) knob7value);
//    LCD.print(" TRreg:");
//    LCD.print(Postn_Trol_Register);
//  }
//  setTrolleyHorizontalPosition((int) knob7value);
//  delay(1000);
//}

//  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentLow, Postn_Trol_MaxExtnsn);
//  
//  setClawBlockVerticalPosition(Postn_ClBlk_AtJib);        
//  setTrolleyHorizontalPosition(Postn_Trol_OverBasket); 
 
  
//  LCD.clear();
//  LCD.home();
//  LCD.print("Done test.");
//  delay(360000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. MUST COMMENT OUT DEFINITELY



  Time_BegngOfHeat = millis();

  State_Register = State_Go2IRGate;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Go2IRGate");


  //=======================================================================State_Go2IRGate
  
  //
  // Robot tape tracks for Time_ApprchRamp until at beginning of ramp.
  //
   
  Time_WhenTimerWasLastRead = millis();

  while(State_Register == State_Go2IRGate && (millis() < (Time_WhenTimerWasLastRead + Time_Go2IRGate)) ){  
    driveWheels();
  }

  State_Register = State_Wait4IRBeacon;

  LCD.clear();
  LCD.home();
  LCD.print("State_Wait4IRBeacon");


  //=======================================================================State_Wait4IRBeacon 

  // 
  // Robot waits until 10000Hz is sensed. Delay and repeat of check to "double check" that GO reading was not due to error.
  //

  //full stop in front of IR gate and crane out of way
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
//  while (!startbutton()) {
//    LCD.clear();
//    LCD.home();
//    LCD.print("Wait4IR.PushStart! ");
//    LCD.setCursor(0,1);
//    LCD.print("IR: ");
//    if(!digitalRead(A0)){
//      LCD.print("1 kHz");
//    }else{
//      LCD.print("10 kHz");
//    }
//  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  

  GoSignal = !digitalRead(A0);
  
  //If when you first start sensing for IR signal, the gate is open, just wait this one out. To risky to try to make it through without knowing how long the gate has already been open
  // TO DO: Consider sensing earlier to maybe save time
  while(GoSignal){
    //wait here
    GoSignal = !digitalRead(A0);
  }

  //while the gate is closed, continue to wait
  while(!GoSignal){

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
    LCD.clear();
    LCD.home();
    LCD.print("State_Wait4IRBeacon");
    LCD.setCursor(0,1);
    LCD.print("IN WHILE LOOP");
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
    
    delay(5);
    GoSignal = !digitalRead(A0);
    //if a go signal is detected, sample the signal continuously for some time to ensure the signal is staying high before going
    if(GoSignal){
      unsigned long transition = millis();
      unsigned long sample_time = 50;
      while(millis() - transition < sample_time){
        //if signal switches to no go at any time during sample period, give control back to the parent loop to look for the next go signal
        GoSignal = !digitalRead(A0);
        if(!GoSignal){
          break;
        }
      }
    }
  }

   


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
//    LCD.clear();
//    LCD.home();
//    LCD.print("State_Wait4IRBeacon");
//    LCD.setCursor(0,1);
//    LCD.print("OUT OF LOOP");
//    delay(5000);  //NOTE THIS DELAY
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK



  State_Register = State_ApprchRamp;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_ApprchRamp");


  //=======================================================================State_ApprchRamp
    
  //
  // Robot tape tracks for Time_ApprchRamp until at beginning of ramp.
  //
   
  Time_WhenTimerWasLastRead = millis();

  while(State_Register == State_ApprchRamp && (millis() < (Time_WhenTimerWasLastRead + Time_ApprchRamp)) ){  
    driveWheels();
  }

  State_Register == State_GoUpRamp;

  LCD.clear();
  LCD.home();   
  LCD.print("State_GoUpRamp");

  
  //=======================================================================State_GoUpRamp
    
  //
  // Robot tape tracks for Timing_MilliscndsApprchRamp milliseconds until ostensibly at ramp
  //
   
  Time_WhenTimerWasLastRead = millis();

  Mot_Wheels_Speed = Mot_Wheels_Speed_Ramp;

  while( !( millis() > (Time_WhenTimerWasLastRead + Time_GoUpRamp) ) ){ 
    driveWheels();
    
    LCD.clear();
    LCD.home();   
    LCD.print("State_GoUpRamp");
    LCD.setCursor(0,1);
    LCD.print(Mot_Wheels_Speed);        
  }

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
//  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
//  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
//    LCD.clear();
//    LCD.home();
//    LCD.print("StopAfterFerrari");
//    delay(1000);
      //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK


  Mot_Wheels_Speed = Mot_Wheels_Speed_Reset;

  Time_DelayStart = millis();
  
  //drive for a short time before starting to look for circle
  while ( (millis() < (Time_SensorReadDelay +Time_DelayStart) ){
    driveWheels();
    LCD.clear();
    LCD.home();
    LCD.print("PostRampDelay");
  }           

  Mot_Wheels_Speed = Mot_Wheels_Speed/Mot_Wheels_Speed_UpperPltfrmSlowDownFactor;
                                                  
  State_Register = State_AprchCircle;

  LCD.clear();
  LCD.home();   
  LCD.print("State_AprchCircle");

                                                       

  //=======================================================================State_AprchCircle
  //
  // Follow tape until Snsr_Drive_FrontOutR senses black tape. When does, pivot on right wheel (driving on left) until 
  // Snsr_Drive_FrontInL & Snsr_Drive_FrontInR see black again (i.e. are on the circle again). Then proceed to next state.
  // Delay is to ensure doesn't immediately scan for black tape, because will see immediately. 
  //


  //These next 4 lines of code bring claw block up and back to not knock agents over
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);    
  setClawBlockVerticalPosition(Postn_ClBlk_AtJib);        
  setTrolleyHorizontalPosition(Postn_Trol_OverBasket); 
                                                                      */

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Mot_Wheels_Speed = (double) Mot_Wheels_Speed/Mot_Wheels_Speed_UpperPltfrmSlowDownFactor;
    //=======================================================================TEST CODE ONLY: STOP HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  LCD.clear();
  LCD.home();   
  LCD.print("State_AprchCircle");
  State_Register = State_AprchCircle;


  while(State_Register == State_AprchCircle){ 
    driveWheels(NORMAL);
    LCD.setCursor(0,1);
    LCD.print(Mot_Wheels_Speed);

    if(analogRead(Pin_Snsr_Chassis_Corner_ApprchCircle) > ANALOGTHRESHOLD){      // Symmetrical (re: other surface)     //PRE CHANGE CONDITION: "if(analogRead(Pin_Snsr_Chassis_Corner_ApprchCircle) > ANALOGTHRESHOLD)", where threshold was 400
      motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed_Turn_LW);                   // Symmetrical (re: other surface)
      motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed_Turn_RW);                   // Symmetrical (re: other surface)
      LCD.setCursor(0,1);
      LCD.print("AutoEntrcTrn");
      delay(Time_TurnB4Sense);                                                      //Delay is to ensure doesn't immediately scan for black tape, because will see immediately. 

      while(digitalRead(Pin_Snsr_Chassis_Front_ApprchCircle) == White_Tape){     // Symmetrical (re: other surface)
        LCD.setCursor(0,1);
        LCD.print("Lkng4CrclLine");
      
        motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed_Turn_LW);                 // Symmetrical (re: other surface)
        motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed_Turn_RW);                 // Symmetrical (re: other surface)
      }

      LCD.clear();
      LCD.home();
      LCD.print("OnCrclLine");  

      //This line is required to exit parent while loop.
      State_Register = State_AprchNextLine;
                          
    }
  }




  //=======================================================================State_AprchNextLine
  //
  // Robot follows tape until sensors at tower base sense next line.
  //

  // This speed changes allows for additional slowing down when already in circle, depending on which surface we're on.
  Mot_Wheels_Speed = Mot_Wheels_Speed/Mot_Wheels_Speed_UpperPltfrmADDITIONALSlowDownFactor;

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 2 2 2");
  LCD.setCursor(0,1);
  LCD.print("InCircle: ");
  LCD.print(Mot_Wheels_Speed);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead + Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();

  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");


  //=======================================================================State_Retrvl

  //
  // Function called to retrieve agent at circle line, when perpendicular. Starts from turning crane to tub. 
  // Ends once claw is opened and drops agent. Second line (Line2) from entrance is first line
  //


  //============================================At Line 2
  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentLow, Postn_Trol_MaxExtnsn);               // MUST DO: THURSDAY: COMMENT BACK IN
  
  State_Register = State_AprchNextLine;

  //=======================================================================State_Retrvl, State_AprchNextLine, State_Retrvl, State_AprchNextLine, etc. ...
  //
  // From here, we move to Line3 thru Line6, doing low(3), high(4), medium(5), low(6),  code skips over entrance, 
  // and do wet retrieval and high(1) at Line1.
  //

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 3 3 3");

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();

  //============================================At Line3
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentHigh, Postn_Trol_MaxExtnsn);               // MUST DO: THURSDAY: COMMENT BACK IN
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 4 4 4");

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();

  //============================================At Line4
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentMedium, Postn_Trol_MaxExtnsn);               // MUST DO: THURSDAY: COMMENT BACK IN
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 5 5 5");

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();

  //============================================At Line5
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentLow, Postn_Trol_MaxExtnsn);               // MUST DO: THURSDAY: COMMENT BACK IN
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 6 6 6");

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();

  //============================================At Line6
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentHigh, Postn_Trol_MaxExtnsn);               // MUST DO: THURSDAY: COMMENT BACK IN
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("Aprch ENTRNC");

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();                    
  //============================================At Entrance

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 1 1 1");

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();
  //============================================At Line1
  //Assume it gets you to Line1
  //Do retrieval at Line1
  State_Register = State_Retrvl;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_Retrvl");

  RetrievalType = informIfWetOrDryRtrvl();
//  doAgentRtrvl(RetrievalType, Postn_ClBlk_AtDryAgentMedium, Postn_Trol_MaxExtnsn);               // MUST DO: THURSDAY: COMMENT BACK IN
  
  State_Register = State_AprchNextLine;

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 2 2 2");


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //Tape follow with delay to ensure not read next line while still on previous line
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();                   
  //============================================At Line2

  LCD.clear();  
  LCD.home();   
  LCD.print("AprchLine 3 3 3");

  //Tape follow with delay to ensure not read next line while still on previous line

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
    motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  }
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  Time_WhenTimerWasLastRead = millis();
  while( millis() < (Time_WhenTimerWasLastRead +   Time_AfterReadLineToCallApprchLineFxn) ){  
    driveWheels(NORMAL);
  }

  doAprchNextLine();                   
  //============================================At Line3 (or Line4)

  State_Register = State_TurnToRaise;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_TurnToRaise");


  //=======================================================================State_TurnToRaise

  Mot_Wheels_Speed = Mot_Wheels_Speed_Reset;

  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);

  //drive fwd for the delay time
  motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed);
  delay(Time_DriveFwd2Raise);

    
  //turn during the delay time
  motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed_Turn_LW);                   // Symmetrical (re: other surface)
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed_Turn_RW);                   // Symmetrical (re: other surface)
  delay(Time_Turn2Raise); 


  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);


  State_Register = State_RaisePltfrm;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_RaisePltfrm");


  //=======================================================================State_RaisePltfrm

  setCranePosition(Postn_Crane_FarTurn);              // Get crane out of way
  digitalWrite(Pin_SwitchMotTrol2Lift, LOW);        // Switches relay for motor. Pin on TINAH. As per TINAH log

  Postn_Pltfrm_Register = digitalRead(Pin_Snsr_Pltfrm);
    
  while(Postn_Pltfrm_Register != Postn_Pltfrm_AtZL) {
    motor.speed(Pin_Mot_Pltfrm, Mot_Pltfrm_Speed_Up);
    Postn_Pltfrm_Register = digitalRead(Pin_Snsr_Pltfrm);
  }

  motor.speed(Pin_Mot_Pltfrm, Mot_Speed_Stop);

  State_Register = State_ApprchZL;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_ApprchZL");


  //=======================================================================State_ApprchZL

  Mot_Wheels_Speed = Mot_Speed_ApprchZL;

  Postn_Robot_Register = digitalRead(Pin_Snsr_ZLArrvlSwtch);

  while(Postn_Robot_Register != Postn_Robot_UnderZL) {
    motor.speed(Pin_Mot_Wheels_L, Mot_Speed_ApprchZL*(-1));
    motor.speed(Pin_Mot_Wheels_R, Mot_Speed_ApprchZL);
    Postn_Robot_Register = digitalRead(Pin_Snsr_ZLArrvlSwtch);   
  }

  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);


  State_Register = State_LowerPltfrm;

  LCD.clear();  
  LCD.home();   
  LCD.print("State_LowerPltfrm");


  //=======================================================================State_LowerPltfrm

  Time_WhenTimerWasLastRead = millis();

  while( !( millis() > (Time_WhenTimerWasLastRead + Time_LowerPltfrm) ) ){ 
    motor.speed(Pin_Mot_Pltfrm, Mot_Pltfrm_Speed_Down);
  }

  motor.speed(Pin_Mot_Pltfrm, Mot_Speed_Stop);


    motor.speed(Pin_Mot_Wheels_L, -Mot_Speed_ApprchZL*(-1));
    motor.speed(Pin_Mot_Wheels_R, -Mot_Speed_ApprchZL);
    delay(500);
    motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
    motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);

    
    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.setCursor(0,1);
  LCD.print("AtEndVoidLoop.1hrDly");
  delay(3600000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK



}







//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: FUNCTONS, composite (call more than one other function) *************************************
//*****************************************************************************************************************************************************************************




//=======================================================================COMPOSITE FUNCTION: doAgentRtrvl() =========================================================

// Incoming argument: RetrievalType, AgentHeight. RetrievalType must be specified as either DryRetrieval or WetRetrieval.
// If DryRetrieval, AgentHeight must be specified. For WetRetrieval, AgentHeight does not need to be specified.
// Function meant to be called when tower-base aligned with  is at one of lines . Function calls other functions.
// Last function call (& argument): Trolley ends at max extension.

void doAgentRtrvl(int FXN_RetrievalType, int FXN_ClBlk_Destntn, int FXN_Postn_Trol_EnterExitTub) {
  
  int FXN_Trol_Destntn;
  int FXN_Crane_Destntn;
  int FXN_Claw_Destntn;
  unsigned long offset_time = 100;


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("TrolDestO:");
  LCD.print(FXN_Postn_Trol_EnterExitTub);
  LCD.setCursor(0, 1);
  LCD.print("TrolRegO:");
  LCD.print(Postn_Trol_Register);           
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  
  setTrolleyHorizontalPosition(FXN_Postn_Trol_EnterExitTub);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("ClDestO:");
  LCD.print(Mot_Claw_Angle_MaxOpen);
  LCD.setCursor(0,1);
  LCD.print("ClRegO:");
  LCD.print(Postn_Claw_Register);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  
  setClawPosition(Mot_Claw_Angle_MaxOpen);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("CrnDestO:");
  LCD.print(Postn_Crane_AngleTubLineStd);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  setCranePosition(Postn_Crane_AngleTubLineStd);            // Symmetrical (re: other surface)


  if(FXN_RetrievalType == WetRetrieval){
    FXN_Claw_Destntn = Mot_Claw_Angle_MaxOpen;
  }
  else {
    FXN_Claw_Destntn = Mot_Claw_Angle_Open;
  }
    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("ClDestO:");
  LCD.print(FXN_Claw_Destntn);
  LCD.setCursor(0,1);
  LCD.print("ClRegO:");
  LCD.print(Postn_Claw_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  
  setClawPosition(Mot_Claw_Angle_Open);


  if(FXN_RetrievalType == WetRetrieval){
    FXN_ClBlk_Destntn = Postn_ClBlk_AtDryAgentLow;
  }
    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("CBDestO:");
  LCD.print(FXN_ClBlk_Destntn);
  LCD.setCursor(0,1);
  LCD.print("CBRegO:");
  LCD.print(Postn_ClBlk_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  //move claw block down to agent. If it can't quite get there, move up slightly and continue.
  if(!setClawBlockVerticalPosition(FXN_ClBlk_Destntn)){
    driveVerticalMotor("UP",offset_time);
  }


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  LCD.clear();
  LCD.home();
  LCD.print("Retrval Postn");
  delay(5000);

//  LCD.clear();
//  LCD.home();
//  LCD.print("TrolDestO:");
//  LCD.print(Postn_Trol_OverDryAgent);  
//  LCD.print("TrolRegO:");
//  LCD.print(Postn_Trol_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK 

  //move claw block in towards agent. If it cant quite get there, move back out and continue
  if(!setTrolleyHorizontalPosition(Postn_Trol_OverDryAgent)){
    driveHorizontalMotor("OUT",offset_time);
  }


//  COMMENTED OUT: NO WATER RETRIEVAL YET
//  if(FXN_RetrievalType == WetRetrieval){
//    FXN_Trol_Destntn = Postn_Trol_OverDryAgent;                     //TO DO, CONFIRM THIS
//    setTrolleyHorizontalPosition(FXN_Trol_Destntn);      
//  }

  
    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("ClDestO:");
  LCD.print(Postn_Claw_Close);
  LCD.setCursor(0,1);
  LCD.print("ClRegO:");
  LCD.print(Postn_Claw_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  
  setClawPosition(Postn_Claw_Close);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("CBDestO:");
  LCD.print(Postn_ClBlk_AtJib);
  LCD.setCursor(0,1);
  LCD.print("CBRegO:");
  LCD.print(Postn_ClBlk_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  if(!setClawBlockVerticalPosition(Postn_ClBlk_AtJib)){
    //if the position has not changed from the destination, there is a good chance the claw grabbed the platform
    //attempt to release platform and continue and reset
    if(Postn_ClBlk_Register == FXN_ClBlk_Destntn){
      setClawPosition(Postn_Claw_Open);
      driveHorizontalMotor("OUT",offset_time/2);
      setClawBlockVerticalPosition(Postn_ClBlk_AtJib);   
    }else{
      //if the claw is not stuck on the platform, the agent likely is too high in the claw to allow the limit switch to be pressed
      //move the claw down very slightly
      driveVerticalMotor("DOWN",offset_time);
    }
  }


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("TrolDestO:");
  LCD.print(FXN_Postn_Trol_EnterExitTub);
  LCD.setCursor(0, 1);
  LCD.print("TrolRegO:");
  LCD.print(Postn_Trol_Register);
//  delay(2000);                  
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  setTrolleyHorizontalPosition(FXN_Postn_Trol_EnterExitTub);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("PreFxnCrnDstnO: ");
  LCD.print(Postn_Crane_AngleBot);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  setCranePosition(Postn_Crane_AngleBot);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("TrolDestO:");
  LCD.print(Postn_Trol_OverBasket);  
  LCD.print("TrolRegO:");
  LCD.print(Postn_Trol_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  setTrolleyHorizontalPosition(Postn_Trol_OverBasket);


    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  LCD.clear();
  LCD.home();
  LCD.print("ClDestO:");
  LCD.print(Mot_Claw_Angle_MaxOpen);
  LCD.setCursor(0,1);
  LCD.print("ClRegO:");
  LCD.print(Postn_Claw_Register);
//  delay(2000);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  setClawPosition(Mot_Claw_Angle_MaxOpen);


  return;
  
}




//*****************************************************************************************************************************************************************************
//***********************************************************************SECTION: FUNCTONS, non-composite (call one other function at most)************************************
//*****************************************************************************************************************************************************************************




//=======================================================================FUNCTION: driveWheels()=====================================================================
// 
//Function drives wheels while tape-tracking.
//

void driveWheels(int driveMode) {

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

  switch(driveMode){
    //NORMAL drive mode
    case 0:
      //Calculate derivative error correction component
      Drive_DerivError = Drive_Error - Drive_LastError;
      //Sum correction components into correction variable
      Drive_Correction = Drive_Error*KP + Drive_DerivError*KD; //Original "int Drive_Correction = error*kp + Drive_DerivError*kd" , but extracted Extracted "+ intError*ki" and earler declaration " intError += error;"

    //LEFT_CIRCLE_HASH drive mode
    case 1:
      Drive_Correction = leftTurnBias;

    //RIGHT_CIRCLE_HASH drive mode
    case 2:
      Drive_Correction = rightTurnBias;
  }
  
  //Add correction to wheel motor speeds
  motor.speed(Pin_Mot_Wheels_L, (Mot_Wheels_Speed - Drive_Correction)*-1);
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed + Drive_Correction);
  
  //To note last error to use in next iteration
  Drive_LastError = Drive_Error;


 /*       //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK (KD,KP,SPEED CONTROLS)
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

  while (startbutton()){
     Mot_Wheels_Speed = Mot_Wheels_Speed + 10;
     delay(100);
   }
  
  while (stopbutton()){
    Mot_Wheels_Speed = Mot_Wheels_Speed - 10;
    delay(100);
  }
*/
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

}



//=======================================================================FUNCTION: doAprchNextLine() =====================================================================
//
//No arguments. Simply approaches next line and, upon sensing, continues for Time_AfterReadLineToStop before stopping robot.
//

void doAprchNextLine() {

  unsigned long start_time = millis();
  unsigned long exit_hash_delay = 500;
  unsigned long realign_delay   = 1000;

  //get off the hash mark you are currently on without following it
  while(millis() - start_time < exit_hash_delay){
    if(Surface_Register == Surface_GoLeftSurf){
      driveWheels(LEFT_CIRCLE_HASH);
    }else{
      driveWheels(RIGHT_CIRCLE_HASH);
    }
  }

  //realign with the black tape
  while(millis() - start_time < realign_delay){
    driveWheels(NORMAL);
  }  

  //tape follow until next hash
  while(digitalRead(Pin_Snsr_Drive_FrontOutL) == White_Tape || digitalRead(Pin_Snsr_Drive_FrontOutR) == White_Tape){
    driveWheels(NORMAL);
  } 

  delay(Time_AfterReadLineToStop);
  
  motor.speed(Pin_Mot_Wheels_L, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);

}



//=======================================================================FUNCTION: informIfWetOrDryRtrvl() =====================================================================
// 
// Keeps track of time since beginning of heat. Returns whether or not it's time for wet retrievals.
//

int informIfWetOrDryRtrvl() {

    if( (millis() - Time_BegngOfHeat) > Time_BeginWetRtrvls ) {
      return WetRetrieval;
    }
    else {
      return DryRetrieval;
    }

}



//=======================================================================FUNCTION:   selectSurface()=====================================================================
// 
// Changes relevant variables to get robot ready for left (i.e. robot turns left after gate) and right (i.e. robot turns right after gate) surfaces. Returns nothing.
//

void  selectSurface(int FXN_Surface_Register) {

    if(FXN_Surface_Register == Surface_GoRightSurf) {
      Mot_Wheels_Speed_Turn_LW = Mot_Wheels_Speed/2;                 //TO DO: Ensure I was correct to change from Mot_Speed_Stop
      Mot_Wheels_Speed_Turn_RW = Mot_Wheels_Speed;
      Pin_Snsr_Chassis_Corner_ApprchCircle = Pin_Snsr_Chassis_FrontCrnrL;
      Pin_Snsr_Chassis_Front_ApprchCircle = Pin_Snsr_Drive_FrontInL;
      Postn_Crane_FarTurn = Postn_Crane_FarRight;
      Postn_Crane_IRMakeWay = Postn_Crane_IRMakeWay_R;
      ANALOGTHRESHOLD = ANALOGTHRESHOLD_FarLeftSnsr;
      Postn_Crane_AngleTubLineStd = Postn_Crane_AngleTubLineStd_RSurf;
      Mot_Wheels_Speed_UpperPltfrmSlowDownFactor = 1.1;
      Mot_Wheels_Speed_UpperPltfrmADDITIONALSlowDownFactor = 1;     //Currently no change (if value is 1)
    }

    if(FXN_Surface_Register == Surface_GoLeftSurf) {
      Mot_Wheels_Speed_Turn_LW = Mot_Wheels_Speed*(-1);
      Mot_Wheels_Speed_Turn_RW = -Mot_Wheels_Speed/2;                        //TO DO: Ensure I was correct to change from Mot_Speed_Stop
      Pin_Snsr_Chassis_Corner_ApprchCircle = Pin_Snsr_Chassis_FrontCrnrR;
      Pin_Snsr_Chassis_Front_ApprchCircle = Pin_Snsr_Drive_FrontInR;
      Postn_Crane_FarTurn = Postn_Crane_FarLeft;
      Postn_Crane_IRMakeWay = Postn_Crane_IRMakeWay_L;
      ANALOGTHRESHOLD = ANALOGTHRESHOLD_FarRightSnsr;
      Postn_Crane_AngleTubLineStd = Postn_Crane_AngleTubLineStd_LSurf;
    }
}



//=======================================================================FUNCTION: setCranePosition() =====================================================================
// 
// Incoming argument: Postn_Crane_Destntn . Must have min value 0s and max value 180s.
// Sets position of crane.
//

void setCranePosition(int FXN1_Crane_Destntn) {

  //Return if already at destination 
  if(FXN1_Crane_Destntn == Postn_Crane_Register){
    return;
  }

  //Set crane servo direction according to destination. 
  RCServo0.write(FXN1_Crane_Destntn); //sends desired angle change to servo

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  //Displays sensor's voltage and position register
  LCD.clear();  
  LCD.home();
  LCD.print("InFxnCrnDstnI:");
  LCD.print(FXN1_Crane_Destntn);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  delay(Time_CraneMvmtDelay);                                //TO DO: ensure correct time. MAKE INTO A VARIABLE. Delay to wait to ensure had time to turn
  Postn_Crane_Register = FXN1_Crane_Destntn; //No sensor; just assumes completed turn

  return;
}



//=======================================================================FUNCTION: setClawPosition() =====================================================================

// 
// Incoming argument: Postn_Claw_Destntn . Must have min value 0 and max value 1 (open or closed).
// Opens or closes claw, based on input argument. Registers whether final state is open, closed, or has agent.
// If has agent, increments variable NumOfAgentsSaved.
// CURRENTLY, SENSOR NOT USED, SO SOME CODE MAY BE IRRELEVANT. (E.G. NumOfAgentsSaved, Postn_Claw_Register)
//

void setClawPosition(int FXN1_Claw_Destntn) {

  //Set claw servo direction according to destination. Return if already at destination
  if(FXN1_Claw_Destntn == Postn_Claw_Register){
    return;
  } 

  //Drive motor until destination, checking if arrived
  RCServo1.write(FXN1_Claw_Destntn);//sends desired angle change to servo
  delay(500);                                                   //TO DO: ensure correct time. MAKE INTO A VARIABLE. Delay to wait to ensure had time to close/open             
  //Commented out because not using claw sensor right now
/*  Snsr_Claw_Postn = round( analogRead(Pin_Snsr_Claw) * Snsr_Claw_CnvrsnRatio ); //This rounding results in value of 0,1,2, where Postn_Claw_Open = 0, Postn_Claw_HaveAgent = 1, Postn_Claw_Close = 2
  Postn_Claw_Register = Snsr_Claw_Postn;
                                                                
  //Register if agent detected in grip
  if(Postn_Claw_Register == Postn_Claw_HaveAgent){                                          
    NumOfAgentsSaved++;
  } */

/*    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
    delay(1000); //This delay lets you see display command just before this funtion was called
  LCD.clear();  
  LCD.home();
  LCD.print("NumAgntsSvd: ");
  LCD.print(NumOfAgentsSaved);    
  LCD.setCursor(0,1);
  LCD.print("ClawPostn: ");
  LCD.print(Postn_Claw_Register);
  delay(1000);
*/    LCD.clear();  
  LCD.home();
  LCD.print("InFxnClawnDstnI");
  LCD.print(FXN1_Claw_Destntn);   
//  delay(3000);

  /*
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  delay(1000); //This delay lets you see menu after destination state reached AND,
               //Keep this delay in (via counter in switch case) to not take off before agent falls in thing
*/

  
  Postn_Claw_Register = FXN1_Claw_Destntn;

  return;
}



//=======================================================================FUNCTION: setHorizontalTrolleyPosition() =====================================================================
// 
// Incoming argument: Destination_ Trolley_Position . Must have min value 0 and max value 3.
// Destination_Trolley_Position compared with Postn_Trol_Register to see if higher or lower.
// Destination_Trolley_Position variable must have first value accurate (e.g. always start at TubRimMax) 
// Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
// (checks after every delay time, which is preset). If transition occurs, registers horizontal trolley 
// position and stops if same as destination.
//

boolean setTrolleyHorizontalPosition(int FXN_Trol_Destntn) {

  unsigned long call_time = millis();
  unsigned long timeout = 3000;

  //Set trolley motor direction according to destination. Return if already at destination 
  if(FXN_Trol_Destntn > Postn_Trol_Register)
    Mot_Trol_Dirctn = Mot_Trol_Dirctn_ToTwr;
  else if(FXN_Trol_Destntn < Postn_Trol_Register)
    Mot_Trol_Dirctn = Mot_Trol_Dirctn_ToEnd;
  else if(FXN_Trol_Destntn == Postn_Trol_Register)
    return true;


  //deal with swithcing situation first, and run to switch regardless of current register
  while(FXN_Trol_Destntn == 0 and digitalRead(Pin_Switch_Trol) != Pin_Switch_Trol_Pressed){
    if(millis() - call_time > timeout){
      return false;
    }
    motor.speed(Pin_Mot_Trol, (Mot_Trol_Speed*Mot_Trol_Dirctn_ToEnd)*Trol_Speed_ToEndMultplier);
  }

  if(digitalRead(Pin_Switch_Trol) == Pin_Switch_Trol_Pressed){
    Postn_Trol_Register = 0;
    return true;
  }

  

  //Drive motor until destination, checking if sense tape marker
  while(Postn_Trol_Register != FXN_Trol_Destntn) {

        
    //Check if the function should time out
    if(millis() - call_time > timeout){
      return false;
    }
    
    if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_ToTwr){
      motor.speed(Pin_Mot_Trol, (Mot_Trol_Speed*Mot_Trol_Dirctn)*Trol_Speed_ToEndMultplier);
    }
    else {
      motor.speed(Pin_Mot_Trol, Mot_Trol_Speed*Mot_Trol_Dirctn);
    }

    Postn_Trol_PrvsSnsr = Snsr_Trol_Postn;
    Snsr_Trol_Postn = digitalRead(Pin_Snsr_Trol);

    //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
    if(Postn_Trol_PrvsSnsr != Snsr_Trol_Postn & Postn_Trol_PrvsSnsr == Black_Tape){
      if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_ToTwr)
        Postn_Trol_Register++;
      else if(Postn_Trol_Register != 1) //position 0 should only be reachable by tripping the switch
        Postn_Trol_Register--;
    }


    //This code is to register when switch pushed at outer end of jib.
    //Meeting condition "!(Postn_Trol_Register == Postn_Trol_MaxExtnsn)" ensures you're not stuck at jib-end when trying to move down (i.e. don't enter next statement if start from jib-end)
    if( !(Postn_Trol_Register == Postn_Trol_MaxExtnsn) && (digitalRead(Pin_Switch_Trol) == Pin_Switch_Trol_Pressed) ){
      Postn_Trol_Register = Postn_Trol_MaxExtnsn;
      FXN_Trol_Destntn = Postn_Trol_Register;     //This affirms condition to exit parent while loop
    }

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("TrolDestnI:");
    LCD.print(FXN_Trol_Destntn);
    LCD.setCursor(0,1);
    LCD.print("TrolRegI:");
    LCD.print(Postn_Trol_Register);
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK

  }

////////////////////////////
        //PAWEL IS MESSING WITH THIS, AUGUST 2ND, NIGHT. COULD EASILY DELETE
//    //Stop motor here, since you've reached destination (condition for exiting above while loop)
//  if( (Postn_Trol_Register == FXN_Trol_Destntn) && (Postn_Trol_Register != Postn_Trol_MaxExtnsn){
//    motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
//  }
//  if( (Postn_Trol_Register == FXN_Trol_Destntn) && (Postn_Trol_Register == Postn_Trol_MaxExtnsn) && (digitalRead(Pin_Switch_Trol) == Pin_Switch_Trol_Pressed) ){
//     motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
//  }
///////////////////////////////

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_Trol_Register == FXN_Trol_Destntn){
    motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);

  //Required since the speed is changed in one direction
  Mot_Trol_Speed = Mot_Trol_SpeedReset;
  
  return true;
  }
}


//=======================================================================FUNCTION: setClawBlockVerticalPosition() =====================================================================

// 
// Incoming argument: Destination_ Claw_Block_Position . Must have min value 0 and max value 2.
// Destination_ Claw_Block_Position compared with Postn_ClBlk_Register to see if higher or lower.
// Destination_ Claw_Block_Position must have first value accurate (e.g. always start at jib)  
// Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
// (checks after every delay time, which is preset). If transition occurs, registers claw block 
// position and stops if same as destination.
//

boolean setClawBlockVerticalPosition(int FXN_ClBlk_Destntn) {

  unsigned long call_time = millis();
  unsigned long timeout = 3000;

  //Set claw block motor direction according to destination. Return if already at destination 
  if(FXN_ClBlk_Destntn > Postn_ClBlk_Register)
    Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Down;
  else if(FXN_ClBlk_Destntn < Postn_ClBlk_Register)
    Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Up;
  else if(FXN_ClBlk_Destntn == Postn_ClBlk_Register)
    return true;

  //Drive motor until destination, checking if sense tape marker
  while(Postn_ClBlk_Register != FXN_ClBlk_Destntn) {

    //Check if the function should time out
    if(millis() - call_time > timeout){
      return false;
    }
    
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
      if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Down)
        Postn_ClBlk_Register++;
      else if(Postn_ClBlk_Register != 1) //the only way to get to position 0 should be by tripping the switch
        Postn_ClBlk_Register--;
    }



  //This code is to register when switch pushed at jib.
  //Meeting condition "!(Postn_ClBlk_Register == Postn_ClBlk_AtJib)" ensures you're not stuck at jib when trying to move down (i.e. don't enter next statement if start from jib)
  if( !(Postn_ClBlk_Register == Postn_ClBlk_AtJib) && (digitalRead(Pin_Switch_ClBlk) == Pin_Switch_ClBlk_Pressed) ){
    Postn_ClBlk_Register = Postn_ClBlk_AtJib;
    FXN_ClBlk_Destntn = Postn_ClBlk_Register;     //This affirms condition to exit parent while loop
  }

    //=======================================================================TEST CODE ONLY: STARTS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
    //Displays sensor's voltage and position register
      LCD.clear();  
      LCD.home();
      LCD.print("CBDestnI:");
      LCD.print(FXN_ClBlk_Destntn);
      LCD.setCursor(0,1);
      LCD.print("CBPostnI:");
      LCD.print(Postn_ClBlk_Register);
      delay(30);        //FOR TESTING ONLY (Lets you see menu display after reaching destination); REMOVE AFTER
    //=======================================================================TEST CODE ONLY: ENDS HERE. CAN COMMENT OUT (or delete) ALL THIS BLOCK
  
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_ClBlk_Register == FXN_ClBlk_Destntn){
    motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);

  //Required since the speed is changed in one direction
  Mot_ClBlk_Speed = Mot_ClBlk_SpeedReset;
    
    return true;
  }
}

/* Drives the vertical actuating motor up or down for a given amount of time
 *  Inputs - 
 *    direction: the direction to move the vertical arm, either "UP" or "DOWN"
 *    time: the number of milliseconds to move the claw for      
 */
void driveVerticalMotor(String direction, unsigned long time){
  unsigned long start_time = millis();

  if(direction == "UP"){
    motor.speed(Pin_Mot_ClBlk, (Mot_ClBlk_Speed*Mot_ClBlk_Dirctn_Up));
  }else if(direction == "DOWN"){
    motor.speed(Pin_Mot_ClBlk, (Mot_ClBlk_Speed*Mot_ClBlk_Dirctn_Down)/ClbBlkDwdSpeedDivider);
  }

  while(!(millis()-start_time > time)){
    //idly spin until the appropriate time has passed
  }

  motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);
  
}

/* Drives the horizontal actuating motor out or in for a given amount of time
 *  Inputs - 
 *    direction: the direction to move the vertical arm, either "OUT" or "IN", where "OUT" is towards the end of the boom
 *    time: the number of milliseconds to move the claw for      
 */
void driveHorizontalMotor(String direction, unsigned long time){
  unsigned long start_time = millis();

  if(direction == "OUT"){
    motor.speed(Pin_Mot_Trol, (Mot_Trol_Speed*Mot_Trol_Dirctn_ToEnd));
  }else if(direction == "IN"){
    motor.speed(Pin_Mot_Trol, (Mot_Trol_Speed*Mot_Trol_Dirctn_ToTwr)/Trol_Speed_ToEndMultplier);
  }

  while(!(millis()-start_time > time)){
    //idly spin until the appropriate time has passed
  }

  motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
  
}


