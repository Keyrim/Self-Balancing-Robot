//This skecth is an example of the class stepper_moteur with just 2 motors 

#include "Stepper_moteur.h"
//Motor's pins
#define pin_left_step 11
#define pin_right_step 10
#define pin_left_dir 12
#define pin_right_dir 9

Stepper_moteur stepper;
void setup()
{
  //Serial.begin(115200);
  
  
  stepper.initialize_stepper(0,pin_left_step, pin_left_dir );
  stepper.initialize_stepper(1,pin_right_step, pin_right_dir );
  stepper.set_periode(0, -1000);
  stepper.set_periode(1, 1000);

  stepper.initialize_timer();
}

ISR(TIMER1_COMPA_vect)
{     
    stepper.timer_interupt();
}

void loop()
{
  
}


