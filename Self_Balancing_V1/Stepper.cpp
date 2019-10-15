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
