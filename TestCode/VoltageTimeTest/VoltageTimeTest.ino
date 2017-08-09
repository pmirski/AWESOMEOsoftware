#include <phys253.h>          
#include <LiquidCrystal.h>

int Drive_Error = 0;
int Drive_LastError = 0;
double Drive_DerivError = 0;
double Drive_Correction = 0;

int Mot_Wheels_Speed = 150;

int KP = 20;
int KD = 20;

int leftTurnBias = 0;
int rightTurnBias = 0;

int Black_Tape = 1;
int White_Tape = 0;

int Pin_Mot_Wheels_L = 0;
int Pin_Mot_Wheels_R = 1;

int Pin_Snsr_Battery_Voltage = 5;
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

unsigned long start_time;
unsigned long runtime;
int voltage;

boolean start_flag = false;
boolean record_flag = false;




void setup() {
  #include <phys253setup.txt>
  RCServo0.write(90);   // crane
  RCServo1.write(90);   // claw

  while(!startbutton()){
    
  }
}

void loop() {
  if(!start_flag){
    start_time = millis();
    start_flag = !start_flag;   
  }

  if(digitalRead(Pin_Snsr_Drive_FrontOutL) == 1 && digitalRead(Pin_Snsr_Drive_FrontOutR) == 1){
    motor.stop_all();
    if(!record_flag){
      runtime = millis() - start_time;
      voltage = analogRead(Pin_Snsr_Battery_Voltage);
      record_flag = !record_flag; 
    }
    LCD.clear();
    LCD.home();
    LCD.print("Voltage: ");
    LCD.print(voltage);
    LCD.setCursor(0,1);
    LCD.print("Time: ");
    LCD.print(runtime);
    delay(30000);
    
  }else{
    if(!record_flag){
      driveWheels(0);
    }else{
      motor.stop_all();
    }
  }
  

}

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
      break;
    //LEFT_CIRCLE_HASH drive mode
    case 1:
      Drive_Correction = leftTurnBias;
      break;

    //RIGHT_CIRCLE_HASH drive mode
    case 2:
      Drive_Correction = rightTurnBias;
      break;
  }
  
  //Add correction to wheel motor speeds
  motor.speed(Pin_Mot_Wheels_L, (Mot_Wheels_Speed - Drive_Correction)*-1);
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed + Drive_Correction);
  
  //To note last error to use in next iteration
  Drive_LastError = Drive_Error;
}
