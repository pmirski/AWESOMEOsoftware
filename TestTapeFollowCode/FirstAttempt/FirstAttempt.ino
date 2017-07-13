//Tape Follow Based On Lab5 K and J

    //DRIVING SENSORS: 
    //  2: front tape trackers (FTT), 
    //  2: edge trackers (ET), 
    //  2: on sides-of/adjacent-to 2 front ones (AT)
//At beginning: follow tape until Right-ET triggered
//Actuate Wait-For until all four front sensors (Both-FTT + Both-AT) see white.
//Pivot right (i.e. Drive left wheel, not right) until same line-following as before
//Actuate Wait-For until Both-FTT sense tape and start driving forward
//EVERYTHING AFTER IS UNKNOWN: Around circle (2to6), follow tape until both under-tower sensors. Then stop & retrieve UNTIL DONE ALL AGENTS

//MILESTONES:
//1. tape-follow + making transition onto circle
//2. do retrieve (+ include a pause if u can)

//Get snapback for ur sensor! :P


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

int THRESH = 100;
int KP_RANGE = 20;
int INT_RANGE = 50;
int DER_RANGE = 10;
int GAIN_RANGE = 50;
int SPEED = 120;

void loop() {
  int lSensor = analogRead(3);
  int rSensor = analogRead(0);
  int proportional = knob(6);
  int derivative = knob(7);
  int integral = 0;
  int gain = 1 ;

  
  // PID code

  //Proportional

  if(lSensor > THRESH && rSensor > THRESH){
    error = 0;
  }
  if(lSensor < THRESH && rSensor > THRESH){
    error = -1;    //turn right
    rightLast = true;
    leftLast = false;
  }
  if(rSensor < THRESH && lSensor > THRESH){
    error = 1;    //turn left
    leftLast = true;
    rightLast = false;
  }
  if(lSensor < 100 && rSensor < 100){
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

  motor.speed(0, (SPEED - correction)*-1);
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

void forward(int pace){
    motor.speed(0,pace);
    motor.speed(1,pace);
}

void stop(){
    motor.speed(0,0);
    motor.speed(1,0);
}


