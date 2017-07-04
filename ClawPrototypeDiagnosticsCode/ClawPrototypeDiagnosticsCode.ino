/*
 * TINAH Template Program - UBC Engineering Physics 253
 * (nakane, 2015 Jan 2)  - Updated for use in Arduino IDE (1.0.6)
 */
 
#include <phys253.h>          
#include <LiquidCrystal.h>    

int inPin = 0;


int VerticalQRDSensor = 40;  //40 = A0

// Digital input: (white tape reading) = 0, (black tape reading) = 1
int QRD_Sensor_Jib_Position_Horizontal = 42;  // 42 = A2

// Variable value and corresponding position:
// 0 = TubRimMax = 0; OverBasket = 1; OverDryAgent = 2; MaxExtension = 3;
int = Jib_Position_Horizontal_Register = 1;   // 1 = initial position


void setup()
{
    #include <phys253setup.txt>
    Serial.begin(9600);  
    pinMode(inPin, INPUT);
}

if(analogRead(Sensor1Pin)<500)

void loop()
{
  LCD.clear();  LCD.home();
  LCD.print("Sensor1 is: ");
  LCD.print(analogRead(Sensor1Pin));
  LCD.setCursor(0,1);
  LCD.print("Sensor2 is: ");
  LCD.print(analogRead(Sensor2Pin));
 
  delay(100);
}
