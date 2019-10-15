#include "Arduino.h"
#include "Stepper.h"

Stepper::Stepper()
{
    
}

void Stepper::set_periode(byte m, float speed)
{
  unsigned int periode = speed ;
  timer_consigne[m] = speed ;
}

void Stepper::initialize_stepper(byte m, byte step, byte dir)
{
  dir_pin[m] = dir ;
  step_pin[m] = step ;
  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);
}

//Void to setup timer's registers
void Stepper::initialize_timer()
{
  delay(100);
  //Setup interuptions 
  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15999;// = ( 16000000 / (prescaler * fre cible))
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //  CS10 bits for 1prescaler
  TCCR1B |=  (1 << CS10);      
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
  sei();//allow interrupts
}


//What hapepns when there is a timer interuption 
void Stepper::timer_interupt()
{
  
}
