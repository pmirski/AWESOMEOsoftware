#include <phys253.h>          
#include <LiquidCrystal.h> 

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(0,INPUT);
}

int error = 0;
int intError = 0;
int derError = 0;
int lastError = 0;
boolean rightLast;
boolean leftLast;
int count = 0;

int KP_RANGE = 20; 
int INT_RANGE = 50;
int DER_RANGE = 10;
int GAIN_RANGE = 50;
int SPEED = 120;

//====================================start: PAWEL'S VARIABLE ADDITION (all new unless otherwise stipulated)=================

// TO DO AFTER INTEGRATION: 

int Snsr_EdgeTrkr_R = 0;            // Initialization value completely arbitrary 
int Pin_Snsr_EdgeTrkr_R = 0;
int Snsr_Front_Far_L = 0;         // Initialization value completely arbitrary 
int Pin_Snsr_Front_Far_L = 4; 
int Snsr_Front_Far_R = 0;        // Initialization value completely arbitrary 
int Pin_Snsr_Front_Far_R = 5;

// TO DO AFTER INTEGRATION: create this class of variables, and corresponding states
// For testing, before this code block entered into main code,  RobotState_Rgstr & RobotState_EntrgCircle 
// must initialize to NOT-the-same values. Otherwise, values are arbitrary.
int RobotState_Rgstr = 0;           
int RobotState_EntrgCircle = 1;  
   
//************Start: old variables already declared in workingcircleretrieval (Delete this code block
int Black_Tape = 1;       // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0;       // See Black_Tape comment
//************End: old variables already declared in workingcircleretrieval

// Next six variables: taken out of void loop() where constantly re-dyanmically-initialized, so can put in variables section of main code when integrating
  int lSensor = 0;      // Initialization value completely arbitrary 
  int rSensor = 0;      // Initialization value completely arbitrary 
  int proportional = 0; // Initialization value completely arbitrary 
  int derivative = 0;   // Initialization value completely arbitrary 
  int integral = 0;
  int gain = 1;

//====================================end: PAWEL'S VARIABLE ADDITION (all new unless otherwise stipulated)===================

void loop() {


//====================================PAWEL'S CODE ADDITION STARTS=================
//Next 4 lines: you changed from original code. It's fine. Just rename variables later.
  lSensor = digitalRead(3); //Snsr_Front_L
  rSensor = digitalRead(0); //Snsr_Front_R
  proportional = knob(6);   //Drive_Prprtnl
  derivative = knob(7);     //Drive_Dervtv

//TO DO AFTER INTEGRATION: Make this take an input argument as to what state it's in: State_CircleApprch

//Follow tape until Snsr_EdgeTrkr_R senses black tape
  if(Snsr_EdgeTrkr_R == Black_Tape){                   // TO DO AFTER INTEGRATION: Will have to change this comparison re: robot's ambidexterity: 
    RobotState_Rgstr = RobotState_EntrgCircle;         // Change this state after while loop to RobotState_InCircle
  }

//Once Snsr_EdgeTrkr_R has sensed black tape, wait until all four front sensors at front&center "sensor cage" see white. Then
//Pivot on right wheel (driving on left) until lSensor & rSensor see black again (i.e. are on the circle again).
//Then exit this while loop and start driving.
  if(RobotState_Rgstr == RobotState_EntrgCircle && Snsr_Front_Far_L == Black_Tape && Snsr_Front_Far_R == Black_Tape && lSensor == Black_Tape && rSensor == Black_Tape){
    while(lSensor != Black_Tape && lSensor != White_Tape){
      motor.speed(0, 0);
      motor.speed(1, SPEED);
    }
  }
//====================================PAWEL'S CODE ADDITION ENDS===================
  

  // PID code

  //Proportional

  if(lSensor && rSensor){
    error = 0;
  }
  if(!lSensor && rSensor){
    error = -1;    //turn right
    rightLast = true;
    leftLast = false;
  }
  if(!rSensor && lSensor){
    error = 1;    //turn left
    leftLast = true;
    rightLast = false;
  }
  if(!lSensor && !rSensor){
    if(rightLast == true)
      error = -5;   //turn hard right
    if(leftLast == true)
      error = 5;    //turn hard left
  }

  //Integeral

  intError += error;

  //Derivative
  derError = error - lastError;
  
  
  //Correction
  
  //int kg = 4 * GAIN_RANGE/1023;
  int kp = proportional * KP_RANGE/1023 * gain;
  int ki = integral * INT_RANGE/1023;
  int kd = derivative * DER_RANGE/1023 * gain;

  
  int correction = error*kp + intError*ki + derError*kd;

  motor.speed(0, (SPEED - correction));
  motor.speed(1, SPEED + correction);

  count += 1;
  lastError = error;

  //Screen Display

  if(count%300 == 0){
      LCD.clear();
      LCD.home();
      LCD.setCursor(0, 0);
      //LCD.print("L: ");
      //LCD.printf(lSensor);
      //LCD.print(", R: ");
      //LCD.print(rSensor);
      LCD.print("P: ");
      LCD.print(kp);
      LCD.print("; I: ");
      LCD.print(ki);
      LCD.setCursor(0, 1);
      LCD.print("D: ");
      LCD.print(kd);
      LCD.print("; G: ");
      //LCD.print(kg);
      LCD.print(" C:");
      LCD.print(correction);
      //LCD.print(kg);
  }
}

void stop(){
    motor.speed(0,0);
    motor.speed(1,0);
}


