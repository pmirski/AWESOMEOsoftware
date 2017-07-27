#include <phys253.h>          
#include <LiquidCrystal.h>

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);

}

int Snsr_Chassis_FrontCrnrR = 0;                // Initialization value completely arbitrary 
int Pin_Snsr_Chassis_FrontCrnrR = 42;
int Snsr_Chassis_FrontCrnrL = 0;                // Initialization value completely arbitrary 
int Pin_Snsr_Chassis_FrontCrnrL = 41;
int Pin_Mot_Wheels_L = 0;
int Pin_Mot_Wheels_R = 1;
int Mot_Wheels_Speed = 190;

//*******************************ALL VARIABLES ABOVE ARE ALREADY IN FULLCODE******************************

int EdgeSense_Error = 0;
int EdgeSense_Counter = 0;

int EdgeSense_WHITE_SURFACE = 100;     // TO DO: ENSURE CORRECT VALUE
int EdgeSense_MID_POINT = 400;          // TO DO: ENSURE CORRECT VALUE
int EdgeSense_OVER_EDGE = 800;        // TO DO: ENSURE CORRECT VALUE
int EdgeSense_M_CORRECT = 0; // medium correction
int EdgeSense_H_CORRECT = 0; // hard corretion

void loop() {
  
  Snsr_Chassis_FrontCrnrR = analogRead(Pin_Snsr_Chassis_FrontCrnrR);
  Snsr_Chassis_FrontCrnrL = analogRead(Pin_Snsr_Chassis_FrontCrnrL);
    
  EdgeSense_Counter++;

  if(EdgeSense_Counter %300 == 0){
    
    LCD.clear();
    LCD.home();
    LCD.setCursor(0, 0);
    LCD.print("Sensor: ");
    LCD.print(Snsr_Chassis_FrontCrnrR);
  }

  //FOR AN EDGE SENSOR ON THE FRONT RIGHT CORNER OF THE ROBOT
  //High sensor readings correcct to the left; low sensor readings correct to the right
  //Positive EdgeSense_Error = left correction; Negative EdgeSense_Error = right correction

  //medium corrections
  if(Snsr_Chassis_FrontCrnrR > EdgeSense_WHITE_SURFACE && Snsr_Chassis_FrontCrnrR <= EdgeSense_MID_POINT){
    EdgeSense_Error = -1*EdgeSense_M_CORRECT;
  }
  if(Snsr_Chassis_FrontCrnrR > EdgeSense_MID_POINT && Snsr_Chassis_FrontCrnrR < EdgeSense_OVER_EDGE){
    EdgeSense_Error = EdgeSense_M_CORRECT;
  }

  //hard corrections
  if(Snsr_Chassis_FrontCrnrR <= EdgeSense_WHITE_SURFACE){
    EdgeSense_Error = -1*EdgeSense_H_CORRECT;
  }
  if(Snsr_Chassis_FrontCrnrR >= EdgeSense_OVER_EDGE){  
    EdgeSense_Error = EdgeSense_H_CORRECT;
  }
  
  motor.speed(Pin_Mot_Wheels_L, Mot_Wheels_Speed - EdgeSense_Error);      // TO DO: ENSURE CORRECT VALUE
  motor.speed(Pin_Mot_Wheels_R, Mot_Wheels_Speed + EdgeSense_Error);     // TO DO: ENSURE CORRECT VALUE
}
