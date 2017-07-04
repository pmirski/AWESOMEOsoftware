/*
   TINAH Template Program - UBC Engineering Physics 253
   (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
*/

#include <phys253.h>
#include <LiquidCrystal.h>

int inPin = 0;
int sens_L = 0; //sensor 1 reading
int sens_R = 0; //sensor 2 reading
double cutoff = 50.0; //curoff voltage
double k_p = 5; //proportional gain
double k_d = 0.0; //derivative gain
int Sensor_R = 40;
int Sensor_L = 42;
double error = 0.0;
double speed = 0.0;
double con = 0.0;
unsigned long knob6value = 0.0;
double motorspeed = -200.00;
double p = 0.0;
int i = 0;
double lerr = 0.0;

void setup()
{
#include <phys253setup.txt>
  Serial.begin(9600);
  pinMode(inPin, INPUT);
}

void loop()
{

  while (!stopbutton()) {

    i++;

    motor.speed(0, motorspeed);
    motor.speed(1, motorspeed);

    sens_L = analogRead(Sensor_R);
    sens_R = analogRead(Sensor_L);

    //    LCD.clear();
    //    LCD.home();
    //    LCD.print("Left" + sens_L);
    //    LCD.setCursor(0, 1);
    //    LCD.print("Right" + sens_R);


    if ((sens_L > cutoff) && (sens_R > cutoff)) error = 0.0;
    if ((sens_L > cutoff) && (sens_R < cutoff)) error = 1.0;
    if ((sens_L < cutoff) && (sens_R > cutoff)) error = -1.0;
    if ((sens_L < cutoff) && (sens_R < cutoff))
    {
     if (lerr>0) error = 5.0;
     if (lerr<=0) error=-5.0;
    }

    p = k_p * error;
    con = p;
    motor.speed(0 , (motorspeed - con)); //Left Motor
    motor.speed(1 , (motorspeed + con)); //Right Motor

    if (i == 100) {
      LCD.clear();
      LCD.home();
      LCD.print("Left = ");
      LCD.print(motorspeed - con);
      LCD.setCursor(0, 1);
      LCD.print("Right = "); 
      LCD.print(motorspeed + con);
      delay(100);
      i = 0;
    }

    lerr=error;
  }

  if (stopbutton())
  {
    motor.stop_all();

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter Speed of robot");
      LCD.setCursor(0, 1);
      knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
      LCD.print((((double) (255.0 / 1023.0))*knob6value));
      delay(100);
    }
    motorspeed = -1 * ((double) (255.0 / 1023.0)) * knob6value;
    delay(1000);

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter k_p of robot");
      LCD.setCursor(0, 1);
      knob6value = knob(6);
      LCD.print(((double) (100.0 / 1023.0))*knob6value);
      delay(10);
    }
    k_p = ((double) (100.0 / 1023.0)) * knob6value;
    delay(1000);

    while (!startbutton()) {
      LCD.clear();
      LCD.home();
      LCD.print("Enter threshold of robot");
      LCD.setCursor(0, 1);
      knob6value = knob(6); //This syntax is required to get knob6 input, not from other 46
      LCD.print((((double) (1000.0 / 1023.0))*knob6value));
      delay(100);
    }
    cutoff = ((double) (1000.0 / 1023.0)) * knob6value;
  }
}

