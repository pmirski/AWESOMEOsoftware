#include <avr/interrupt.h>
#include <phys253.h>          
#include <LiquidCrystal.h>

/**
   Encoder Behavior  (CW)

   State | Sig 1 | Sig 2
     1   |   0   |   1
     2   |   0   |   0
     3   |   1   |   0
     4   |   1   |   1
**/
volatile boolean sig1 = false;
volatile boolean sig2 = false;

volatile int motor_dir = 1;
 
volatile unsigned int INT_0 = 0;
volatile unsigned int INT_1 = 0;
volatile unsigned int INT_2 = 0;
volatile unsigned int INT_3 = 0;


 
ISR(INT0_vect) {trackSig1();};
ISR(INT1_vect) {trackSig2();};
ISR(INT2_vect) {};
ISR(INT3_vect) {};

/*  Enables an external interrupt pin
INTX: Which interrupt should be configured?
    INT0    - will trigger ISR(INT0_vect)
    INT1    - will trigger ISR(INT1_vect)
    INT2    - will trigger ISR(INT2_vect)
    INT3    - will trigger ISR(INT3_vect)
mode: Which pin state should trigger the interrupt?
    LOW     - trigger whenever pin state is LOW
    FALLING - trigger when pin state changes from HIGH to LOW
    RISING  - trigger when pin state changes from LOW  to HIGH 
*/
void enableExternalInterrupt(unsigned int INTX, unsigned int mode)
{
  if (INTX > 3 || mode > 3 || mode == 1) return;
  cli();
  /* Allow pin to trigger interrupts        */
  EIMSK |= (1 << INTX);
  /* Clear the interrupt configuration bits */
  EICRA &= ~(1 << (INTX*2+0));
  EICRA &= ~(1 << (INTX*2+1));
  /* Set new interrupt configuration bits   */
  EICRA |= mode << (INTX*2);
  sei();
}
 
void disableExternalInterrupt(unsigned int INTX)
{
  if (INTX > 3) return;
  EIMSK &= ~(1 << INTX);
}
 
void setup()
{
  #include <phys253setup.txt>
  
  //enableExternalInterrupt(INT0, LOW);
  // enableExternalInterrupt(INT1, FALLING);
  enableExternalInterrupt(INT0, RISING);
  enableExternalInterrupt(INT1, RISING);
}
 
void loop()
{
}

void trackSig1(){
  //when motor is turning clockwise, only allow for clockwise encoder behavior
  if(!sig2 && !sig1){
    LCD.clear();
    LCD.home();
    LCD.print(INT_3++);
    sig1 = true;
    sig2 = false;
  }
}
    
 
void trackSig2(){
 if (sig1 && !sig2) {
    LCD.clear();
    LCD.home();
    LCD.print(INT_3++);
    sig1 = false;
    sig2 = false;
 }
}

void printStuff(){
  LCD.clear(); LCD.home(); LCD.print("INT1: "); LCD.print(INT_1++);
}


