#include <phys253.h>          
#include <LiquidCrystal.h> 

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(0,INPUT);
}

//=======================================================================VARIABLES ONLY FOR FUNCTION drive==========================================

int Drive_Correction = 0;               // Initialization value, keep at 0. 
int Drive_Error = 0;                    // Initialization value, keep at 0.
int Drive_DerivError = 0;               // Initialization value, keep at 0.
int Drive_LastError = 0;                // Initialization value, keep at 0.

int     Snsr_Drive_FrontInL = 0;              // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontInL = 8;
int     Snsr_Drive_FrontInR = 0;              // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontInR = 9;
int     Snsr_Drive_FrontOutL = 0;               // Initialization value completely arbitrary 
int Pin_Snsr_Drive_FrontOutL = 10; 
int     Snsr_Drive_FrontOutR = 0;               // Initialization value completely arbitrary . DIGITAL PIN 7 FOR NOW, BUT YOU'LL HAVE TO USE AN ANALOG INPUT (as per TINAH) WITH THRESHOLD VALUE
int Pin_Snsr_Drive_FrontOutR = 11;

int     Snsr_Chassis_FrontCrnrR = 0;               // Initialization value completely arbitrary . DIGITAL PIN 7 FOR NOW, BUT YOU'LL HAVE TO USE AN ANALOG INPUT (as per TINAH) WITH THRESHOLD VALUE
int Pin_Snsr_Chassis_FrontCrnrR = 42;
int ANALOGTHRESHOLD = 400; // TEMPORARY VARIABLE

boolean Postn_Drive_Snsr_rightLast;
boolean Postn_Drive_Snsr_leftLast;

int Mot_Wheels_Speed = 70;             //Initialization value VERY IMPORTANT
int Pin_Mot_Wheels_L = 0;
int Pin_Mot_Wheels_R = 1;
int KD = 20;                            //Drive_Prprtnl     16 or 20 was good value during testing
int KP = 16;                            //Drive_Dervtv      16 or 20 was good value during testing

//USED ONLY FOR TESTING driveWheels() to display menu
int count = 0; 


//TO DO AFTER INTEGRATION: 

//State_Go2IRGate (RANDOM NUMBERS! CHANGE WHEN KNOW IF USING AND THUS MAKING)
int Snsr_Laser = 419284;
int Pin_Snsr_Laser = 32423;

//State_ApprchRamp
int Time_WhenFunctionWasCalled = 0;
int Time_ApprchRamp = 5000; // Three seconds. TO DO: determine this!!!

//State_Time_GoUpRamp
int Time_GoUpRamp = 5000; // Three seconds. TO DO: determine this!!!
int Mot_Wheels_Speed_Ramp = 100;
int Mot_Wheels_Speed_Reset = Mot_Wheels_Speed;             //Used to reset Mot_Wheels_Speed when value changes

//State_AprchLine1      TO DO: determine if can put just one in, far enough back
int Pin_Snsr_TowerBottom_Line1 = 99999;    
int Snsr_TowerBottom_Line1 = 99999; 

//State_AprchNextLine
int Pin_Snsr_TowerBottom_Lines2to6_L = 99999;    
int Snsr_TowerBottom_Lines2to6_L = 99999; 
int Pin_Snsr_TowerBottom_Lines2to6_R = 99999;    
int Snsr_TowerBottom_Lines2to6_R = 99999; 

//State_Go2ZipLine
int Pin_Snsr_ZiplineArrival = 9999999;
int Snsr_ZiplineArrival = 9999999;
int Postn_Robot_UnderZL = 1;        //Arbitrary until know what it needs to be re: sensor reading once zipline trips sensor

// TO DO AFTER INTEGRATION: delete all these variables since they're already in main loop
int State_WaitAtStart = 0;                   
int State_Go2IRGate = 1;                  
int State_Wait4IRBeacon = 2;                
int State_ApprchRamp = 3;                   
int State_GoUpRamp = 4;                     
int State_AprchCircle = 5;              
int State_EntrgCircle = 6;
int State_AprchLine1 = 7;
int State_DryRetrvLine1 = 8;
int State_WetRetrvLine1 = 9;
int State_AprchNextLine = 10;
int State_DryRetrv = 11;
int State_WetRetrv = 12;
int State_Go2ZipLine = 13;
int State_Register = 0;     // Initialization value completely arbitrary        
int Black_Tape = 1;         // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0;         // See Black_Tape comment
int Mot_Speed_Stop = 0;     // Use this to stop motors


void loop() {
driveWheels();

/*
//===================NO IR SENSOR YET; SO COMMENT OUT TOTALLY=====================State_Go2IRGate============================================================
  //incorporate these next two lines in
//State_Register = State_Go2IRGate;
//driveWheels();

// Robot keeps going while sensing with laser until laser sensor registers that at IR gate
  if(State_Register == State_Go2IRGate){
 Snsr_Laser == digitalRead(Pin_Snsr_Laser);      //COMMENT THIS BACK IN ONCE U KNOW HOW LASER COULD WORK
  }
// Currently, this returns automatically because nothing done for Postn_Laser_AtIRGate
  if(State_Register == State_Go2IRGate && Snsr_Laser == Postn_Laser_AtIRGate){
    return;
  }
*/

                                                            

//=======================================================================State_ApprchRamp=====================================================================
// Robot tape tracks for Timing_MilliscndsApprchRamp milliseconds until ostensibly at ramp
Time_WhenFunctionWasCalled = millis();
State_Register = State_ApprchRamp;

  while(State_Register == State_ApprchRamp && !(millis() > (Time_WhenFunctionWasCalled + Time_ApprchRamp)) ){  
    driveWheels();
  }


//=======================================================================State_GoUpRamp=====================================================================
// Robot tape tracks for Timing_MilliscndsApprchRamp milliseconds until ostensibly at ramp
Time_WhenFunctionWasCalled = millis();
State_Register = State_GoUpRamp;

  if(State_Register == State_GoUpRamp){
    Mot_Wheels_Speed_Reset == Mot_Wheels_Speed;
    Mot_Wheels_Speed = Mot_Wheels_Speed_Ramp;
  }
    
  while( !( millis() > (Time_WhenFunctionWasCalled + Time_GoUpRamp) ) ){ 
    driveWheels();      
  }

Mot_Wheels_Speed = Mot_Wheels_Speed_Reset;


//=======================================================================State_AprchCircle=====================================================================
// Follow tape until Snsr_Drive_FrontOutR senses black tape. When does, pivot on right wheel (driving on left) until 
// Snsr_Drive_FrontInL & Snsr_Drive_FrontInR see black again (i.e. are on the circle again). Then return to main void loop (i.e. stop driving).
// Pause is to ensure doesn't immediately scan for black tape, because will see immediately. 


State_Register = State_AprchCircle;
    while(State_Register == State_AprchCircle){ 
      driveWheels();
      Snsr_Chassis_FrontCrnrR = analogRead(Pin_Snsr_Chassis_FrontCrnrR);

      if(Snsr_Chassis_FrontCrnrR > ANALOGTHRESHOLD){
        motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
        motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
        delay(1000);
        Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR);

                        //IT'S NOT WORKING RIGHT HERE, BABY

        while(Snsr_Drive_FrontInR == White_Tape){
          Snsr_Drive_FrontInR = digitalRead(Pin_Snsr_Drive_FrontInR);
          motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
          motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
        }
          State_Register = State_AprchNextLine; // THIHS LIN JUST FOR TESTING      
//        State_Register = State_AprchLine1;
      }
      
    }

      /*

//=======================================================================State_AprchLine1=====================================================================
//State_Register = State_AprchLine1 executed in previous code block. Here, robot follows tape until sensor at tower base senses first line.
  while(State_Register == State_AprchLine1 && digitalRead(Pin_Snsr_TowerBottom_Line1) == White_Tape){
    driveWheels();
  }

      */

//=======================================================================State_AprchNextLine==================================================================
//Robot follows tape until sensors at tower base sense next line.
//State_Register = State_AprchNextLine;

//START OF JUST TESTING : stop at next line
 while(digitalRead(Pin_Snsr_Drive_FrontOutL) == White_Tape && digitalRead(Pin_Snsr_Drive_FrontOutR) == White_Tape){
        driveWheels();
 }

  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  motor.speed(Pin_Mot_Wheels_R, Mot_Speed_Stop);
  delay(10000);

//END OF JUST TESTING


//  while(State_Register == State_AprchNextLine && digitalRead(Pin_Snsr_TowerBottom_Lines2to6_L) == White_Tape && digitalRead(Pin_Snsr_TowerBottom_Lines2to6_R) == White_Tape){
//    driveWheels();
//  }

      /*

//=======================================================================State_Go2ZipLine=====================================================================
//Robot drives forward until zipline_arrival sensor is triggered
State_Register == State_Go2ZipLine;

    while(digitalRead(Pin_Snsr_ZiplineArrival) != Postn_Robot_UnderZL ){
      motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed*(-1));
      motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed);
    }

                */
                                            
}

//=======================================================================FUNCTION: driveWheels()=====================================================================
void driveWheels() {

//=======================================================================P.I.D. CODE BLOCK==========================================
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


//=======================================================================START: (SCREEN + KD,KP,SPEED CONTROLS); CODE FOR TESTING ONLY; CAN DELETE THIS CODE BLOCK==========================================
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
  
  //Control KD, KP, & Mot_Wheels_Speed
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
//=======================================================================END: CODE FOR TESTING ONLY; CAN DELETE THIS CODE BLOCK==========================================

}
