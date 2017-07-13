#include <phys253.h>          
#include <LiquidCrystal.h>    

//README: Variabe values do not reset during testing, unless you reset them.
//Both position values are initially 0



int inPin = 0;

//          VARIABLES USE IN MORE THAN ONE FUNCTION
int Black_Tape = 1; // Used to determine whether trolley and claw block sensors read white or black tape
int White_Tape = 0; // CONCERN: White tape = 0? test
int Mot_Speed_Stop = 0; // Use this to stop motors //Mot_Speed_Stop

//          VARIABLES ONLY FOR FUNCTION setTrolleyHorizontalPosition
// Important concerns: delay length, initial trolley position, trolley motor speed
int Postn_Trol_PrvsSnsr = 0;  //Initialization value; MAKE SO CAN CHOOSE FROM MENU 
int Snsr_Trol_Postn = 0; //Initialization value arbitrary 
int Mot_Trol_Dirctn_Bkwrd = -1; 
int Mot_Trol_Dirctn_Fwrd = 1; 
int Pin_Snsr_Trol = 6; // Pin on TINAH. As per TINAH log
int Pin_Mot_Trol = 2; // Pin on TINAH. As per TINAH log
int Mot_Trol_Speed = 100; // Value is 1/2.55 of maximum, CAN CHANGE SPEED 
int Mot_Trol_Delay_ms; // Delay time arbitrary; determines accuracy of stoppages; CAN CHANGE DELAY TIME 
    // Values below are arbitrary – set in menu?
int Mot_Trol_Dirctn = 1; 
int Postn_Trol_Destntn = 3; //ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_Trol_Register = 0; //Values 0 to 3 correspond to trolley position: (AtBoom) = 0, (TubRimMax) = 1 , (OverBasket) = 2 , (OverDryAgent) = 3 , (MaxExtension) = 4 . First position: OverBasket

//          VARIABLES ONLY FOR FUNCTION setClawBlockVerticalPosition
//Important concerns: delay length, initial claw block position, claw block motor speed,
int Postn_ClBlk_PrvsSnsr = White_Tape;  //Initialization value; MAKE SO CAN CHOOSE FROM MENU //Postn_ClBlk_PrvsSns
int Snsr_ClBlk_Postn = 0; //Initialization value arbitrary 
int Mot_ClBlk_Dirctn_Down = -1;
int Mot_ClBlk_Dirctn_Up = 1;
int Pin_Snsr_ClBlk = 7; // Pin on TINAH. As per TINAH log 
int Pin_Mot_ClBlk = 3; // Pin on TINAH. As per TINAH log
int Mot_ClBlk_Speed = 100; // Value is 1/2.55 of maximum, CAN CHANGE SPEED 
int Mot_ClBlk_Delay_ms; // Delay time arbitrary; determines accuracy of stoppages; CAN CHANGE DELAY TIME
    // Values below are arbitrary – set in menu?
int Mot_ClBlk_Dirctn = 1; 
int Postn_ClBlk_Destntn = 2; //ARBITRARY DYNAMIC INITIALIZATION VALUE 
int Postn_ClBlk_Register = 0; //Values 0 to 3 correspond to claw block position: (AtJib) = 0 , (AtDryAgent) = 1 , (AtWater) = 2 , (MaxExtension) = 3. First position: AtJib

//          VARIABLES ONLY FOR MENU (in main loop)
double Menu_Destntn_Ratio = 3.0/ 1023.0;  //Ratio to convert knob(6) input in menu to get correct destination
double Menu_DCMotor_Ratio = 255.0 / 1023.0;   //Ratio to convert knob(6) input in menu to get correct DC motor speeds
int knob6value;                               //Used to store value from knob 6



//    THIS IS REQUIRED ARDUINO STUFF
void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
}

 //   THIS IS YOUR MAIN LOOP
void loop()
{

  while (!stopbutton()) {
    LCD.clear();
    LCD.home();
    LCD.print("PushStopToInputVals");
    delay(50);
  }

  if (stopbutton())
    {
        motor.stop_all();

        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterClawBlkDestination");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_Destntn_Ratio*knob6value));
         delay(100);
        }
    
        Postn_ClBlk_Destntn = round(Menu_Destntn_Ratio*knob6value); //prob hafta change this to int...YEA!
        delay(500);
        
        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterClawBlkMotorSpeed");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_DCMotor_Ratio*knob6value));
        delay(100);
        }
    
        Mot_ClBlk_Speed = round(Menu_DCMotor_Ratio*knob6value); //prob hafta change this to int...YEA!
        delay(500);
        
        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterTrolleyDestination");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_Destntn_Ratio*knob6value));
        delay(100);
        }
    
        Postn_Trol_Destntn = round(Menu_Destntn_Ratio*knob6value); //prob hafta change this to int...YEA!
        delay(500);

        while (!startbutton()) {
          LCD.clear();
          LCD.home();
          LCD.print("EnterTrolleyMotorSpeed");
          LCD.setCursor(0, 1);
          knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
          LCD.print(round(Menu_DCMotor_Ratio*knob6value));
        delay(100);
        }
    
        Mot_Trol_Speed = round(Menu_DCMotor_Ratio*knob6value); //prob hafta change this to int...YEA!
    
    }

    //Call to both functions, in succession
    setTrolleyHorizontalPosition(Postn_Trol_Destntn);
    //setClawBlockVerticalPosition(Postn_ClBlk_Destntn);

}

//      FUNCTIONS START HERE


/* 
  Incoming argument: Destination_ Trolley_Position . Must have min value 0 and max value 3.
  Destination_ Trolley_Position compared with Postn_Trol_Register to see if higher or lower.
  Destination_ Trolley_Position variable must have first value accurate (e.g. always start at TubRimMax) 
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers horizontal trolley 
  position and stops if same as destination.
*/
void setTrolleyHorizontalPosition(int Postn_Trol_Destntn) {

//Set trolley motor direction according to destination. Return if already at destination 
if(Postn_Trol_Destntn > Postn_Trol_Register)
  Mot_Trol_Dirctn = Mot_Trol_Dirctn_Fwrd;
else if(Postn_Trol_Destntn < Postn_Trol_Register)
  Mot_Trol_Dirctn = Mot_Trol_Dirctn_Bkwrd;
else if(Postn_Trol_Destntn = Postn_Trol_Register)
  return;

//Drive motor until destination, checking if sense tape marker
while(Postn_Trol_Register != Postn_Trol_Destntn) {
  motor.speed(Pin_Mot_Trol, Mot_Trol_Speed*Mot_Trol_Dirctn);
  Postn_Trol_PrvsSnsr = Snsr_Trol_Postn;
  Snsr_Trol_Postn = digitalRead(Pin_Snsr_Trol);

  //If see tape marker transition (from black to white patch (which is position marker)), increment trolley position register according to motor direction
  if(Postn_Trol_PrvsSnsr != Snsr_Trol_Postn & Postn_Trol_PrvsSnsr == Black_Tape){
    if(Mot_Trol_Dirctn == Mot_Trol_Dirctn_Fwrd)
      Postn_Trol_Register++;
    else
      Postn_Trol_Register--;
  }
    
  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Snsr_Trol_Postn);
    LCD.setCursor(0,1);
    LCD.print("TrolPostn: ");
    LCD.print(Postn_Trol_Register);
                                                                            delay(20);  //THIS DELAY LIMITS SPEED
  
}

    //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_Trol_Register == Postn_Trol_Destntn){
  motor.speed(Pin_Mot_Trol, Mot_Speed_Stop);
  }

                                                                            delay(1000);  //FOR TESTING ONLY; REMOVE AFTER
  
//return after while loop
return;
}



/* 
  Incoming argument: Destination_ Claw_Block_Position . Must have min value 0 and max value 2.
  Destination_ Claw_Block_Position compared with Postn_ClBlk_Register to see if higher or lower.
  Destination_ Claw_Block_Position must have first value accurate (e.g. always start at jib)  
  Motor direction thusly determined. As motor runs, loop runs that checks for tape color transition 
  (checks after every delay time, which is preset). If transition occurs, registers claw block 
  position and stops if same as destination.
*/
void setClawBlockVerticalPosition(int Postn_ClBlk_Destntn) {

//Set claw block motor direction according to destination. Return if already at destination 
if(Postn_ClBlk_Destntn > Postn_ClBlk_Register)
  Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Up;
else if(Postn_ClBlk_Destntn < Postn_ClBlk_Register)
  Mot_ClBlk_Dirctn = Mot_ClBlk_Dirctn_Down;
else if(Postn_ClBlk_Destntn = Postn_ClBlk_Register)
  return;

//Drive motor until destination, checking if sense tape marker
while(Postn_ClBlk_Register != Postn_ClBlk_Destntn) {
  motor.speed(Pin_Mot_ClBlk, Mot_ClBlk_Speed*Mot_ClBlk_Dirctn);
  Postn_ClBlk_PrvsSnsr = Snsr_ClBlk_Postn;
  Snsr_ClBlk_Postn = digitalRead(Pin_Snsr_ClBlk);
  
  //If see tape marker transition (from black to white patch (which is position marker)), increment claw block position register according to motor direction
  if(Postn_ClBlk_PrvsSnsr != Snsr_ClBlk_Postn & Postn_ClBlk_PrvsSnsr == Black_Tape){
    if(Mot_ClBlk_Dirctn == Mot_ClBlk_Dirctn_Up)
      Postn_ClBlk_Register++;
    else
      Postn_ClBlk_Register--;
  }

  //Displays sensor's voltage and position register
    LCD.clear();  
    LCD.home();
    LCD.print("SensorVoltage: ");
    LCD.print(Snsr_ClBlk_Postn);
    LCD.setCursor(0,1);
    LCD.print("ClBlkPostn: ");
    LCD.print(Postn_ClBlk_Register);
                                                                            delay(20);  //THIS DELAY LIMITS SPEED
  
  }

  //Stop motor here, since you've reached destination (condition for exiting above while loop)
  if (Postn_ClBlk_Register == Postn_ClBlk_Destntn){
  motor.speed(Pin_Mot_ClBlk, Mot_Speed_Stop);
  }

                                                                            delay(1000);  //FOR TESTING ONLY; REMOVE AFTER

//return after while loop
return;
}



