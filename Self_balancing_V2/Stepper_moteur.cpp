#include "Arduino.h"
#include "Stepper_moteur.h"

Stepper_moteur::Stepper_moteur(){
  
}


//the speed is given in degrees per seconds 
void Stepper_moteur::set_periode(byte m, float speed)
{
   if (abs(speed) < 50)
  {
    moteur_actifs[m] = false;
  }
  else 
  {
    moteur_actifs[m] = true;
    moteur_direction[m] = speed > 0 ;
  }
  unsigned int frequency = abs(speed) / dregrees_per_steps ;
  timer_consigne[m] = born_timer(16000000 / (8 * frequency));
  if(timer_compteur[m] == 0)timer_compteur[m] = timer_consigne[m];

}

void Stepper_moteur::initialize_stepper(byte m, byte step, byte dir)
{
  dir_pin[m] = dir ;
  step_pin[m] = step ;
  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);
}

void Stepper_moteur::initialize_timer()
{
  for (byte m =0; m < number_of_motor; m++)actual_OCR1A = min(actual_OCR1A, timer_consigne[m]);
  delay(100);
  //Setup interuptions 
  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = actual_OCR1A;// = ( 16000000 / (prescaler * fre cible))
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //  CS11 bits for 8 prescaler
  TCCR1B |=  (1 << CS11);        
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
  sei();//allow interrupts
}

void Stepper_moteur::timer_interupt()
{
  unsigned int min = 65535;
    for(byte m=0; m < number_of_motor; m ++)
    {
      
      /*Serial.print("\t");
      Serial.print(actual_OCR1A);*/
      timer_compteur[m] -= actual_OCR1A ;
      if(timer_compteur[m] == 0)
      {
        timer_compteur[m] = timer_consigne[m];
        if(moteur_actifs[m])moove(m);
      }
      min = min(min, timer_compteur[m]);

    }
  actual_OCR1A = min ;
  OCR1A = min ;
}

void Stepper_moteur::moove(byte moteur)
{
  digitalWrite(dir_pin[moteur], moteur_direction[moteur]);     //On met le pin de direction dans le bon etat
  digitalWrite(step_pin[moteur], HIGH);                   //On envoi une impulsions pour que le moteur fasse un pas 
  //delayMicroseconds();
  digitalWrite(step_pin[moteur], LOW);
}

unsigned int Stepper_moteur::born_timer(unsigned int in_timer)
{
  if(in_timer < 1999)return 3999;
  else if (in_timer > 65000)return 65000;
  else return in_timer;
}