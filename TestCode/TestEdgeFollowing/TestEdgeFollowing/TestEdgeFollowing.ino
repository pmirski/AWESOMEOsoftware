#include <phys253.h>          
#include <LiquidCrystal.h>

void setup() {
  #include <phys253setup.txt>
  Serial.begin(9600);

}

int error = 0;
int SPEED = 190;
int timer = 0;

int edge_sensor_left = analogRead(1);
int edge_sensor_right = analogRead(2);
int motor_leftWheel = 0;
int motor_rightWheel = 1;

int WHITE_SURFACE = 50;
int MID_POINT = 400;
int OVER_EDGE = 950;
int kP = 10;         //proportional gain factor
int M_CORRECT = 0; // medium correction
int H_CORRECT = 0; // hard corretion

void loop() {

  edge_sensor_left = analogRead(1);
  edge_sensor_right = analogRead(2);
  
  timer++;

  if(timer % 300 == 0){
    
    LCD.clear();
    LCD.home();
    LCD.setCursor(0, 0);
    LCD.print("Sensor: ");
    LCD.print(edge_sensor_right);
  }

  // lower readings result in a negative correction (to the right) while high readings have a positive correction (to the left)
  error = (edge_sensor_right - MID_POINT)/1023*kP;

//  //FOR AN EDGE SENSOR ON THE FRONT RIGHT CORNER OF THE ROBOT
//  //High sensor readings correcct to the left; low sensor readings correct to the right
//  //Positive error = left correction; Negative error = right correction
//
//  //medium corrections
//  if(edge_sensor_right > WHITE_SURFACE && edge_sensor_right <= MID_POINT){
//    error = -1*M_CORRECT;
//  }
//  if(edge_sensor_right > MID_POINT && edge_sensor_right < OVER_EDGE){
//    error = M_CORRECT;
//  }
//
//  //hard corrections
//  if(edge_sensor_right <= WHITE_SURFACE){
//    error = -1*H_CORRECT;
//  }
//  if(edge_sensor_right >= OVER_EDGE){  
//    error = H_CORRECT;
//  }
  
  motor.speed(motor_leftWheel, SPEED - error);
  motor.speed(motor_rightWheel, SPEED + error);
}
