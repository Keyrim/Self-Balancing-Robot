#include "Arduino.h"
#include "Stepper_moteur.h"

Stepper_moteur::Stepper_moteur(){
  
}

//Value must be 1, 2, 4, 8, 16 for the microstepping
void Stepper_moteur::set_micro_stepping(byte divider){
  micrro_stepping_div = divider ;
  for(int p = 0; p < 3; p++)pinMode(MS1 + p, OUTPUT);
  byte output ;
  switch (divider)
  {
  case 1:
    output = B00000000 ;
    break;
  case 2:
    output = B00000001 ;
    break;
  case 4:
    output = B00000010 ;
    break;
  case 8:
    output = B00000011 ;
    break;
  case 16:
    output = B00000111 ;
    break;
  
  default:
    output = B00000000 ;
    break;
  }
  digitalWrite(MS1, (output & B00000001)); 
  digitalWrite(MS2, (output & B00000010)>>1);
  digitalWrite(MS3, (output & B00000100)>>2);
}

//the speed is given in degrees per seconds 
void Stepper_moteur::set_speed(byte m, float speed)
{
    
   if (abs(speed) < 10)
  {
    //Si on est trop lent autant ne rien faire
    moteur_actifs[m] = false;
    speed = 1 ;    
  }
  else 
  {
    //Si moteur pas trop lent moteur actif
    moteur_actifs[m] = true;
    //Détermine le sens en fonction du signe de la vitesse 
    moteur_direction[m] = speed > 0 ;
  }
  float frequency = (abs(speed) * micrro_stepping_div) ; 
  frequency /=  dregrees_per_steps ;
  
  //The 8 comes from the prescaler set in the initialize_timer function
  timer_consigne[m] = born_timer((16000000 / (8 * frequency)));
  
  if(timer_compteur[m] == 0)timer_compteur[m] = timer_consigne[m];

}

//initialize the stepper giving is number is step in and his direction pin 
void Stepper_moteur::initialize_stepper(byte m, byte step, byte dir)
{
  dir_pin[m] = dir ;
  step_pin[m] = step ;
  pinMode(step, OUTPUT);
  pinMode(dir, OUTPUT);
}

//Initialize the timer 1
void Stepper_moteur::initialize_timer()
{
  //Prescaler set at 8 
  //f_comparaison = (16 000 000 / (prescaler * f_cible))
  

  for (byte m =0; m < number_of_motor; m++)actual_OCR1A = min(actual_OCR1A, timer_consigne[m]);
  delay(100);
  //Setup interuptions 
  cli();//stop interrupts
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = actual_OCR1A;// = ( 16 000 000 / (prescaler * fre cible))
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //  CS11 bits for 8 prescaler
  TCCR1B |=  (1 << CS11);        
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 
  sei();//allow interrupts
}

//Timer 1 interuption making our motors mooving at the right speed 
void Stepper_moteur::timer_interupt()
{
  //Serial.print(micros());
  unsigned int min = 65535;
    for(byte m=1; m < number_of_motor; m ++)
    {
      timer_compteur[m] -= actual_OCR1A ;
      if(timer_compteur[m] < 40)
      {
        timer_compteur[m] = timer_consigne[m];
        if(moteur_actifs[m])moove(m);
      }
      min = min(min, timer_compteur[m]);

    }
  //Serial.print("\t");
  //Serial.println(micros());
  
  
  actual_OCR1A = min ;
  OCR1A = min ;
  //digitalWrite(MS1, 0);
}

//The desired motor mooves in the direction according to moteur_direction[moteur] private variable
void Stepper_moteur::moove(byte moteur)
{
  digitalWrite(dir_pin[moteur], moteur_direction[moteur]);     //On met le pin de direction dans le bon etat
  digitalWrite(step_pin[moteur], HIGH);                        //On envoi une impulsions pour que le moteur fasse un pas 
  delayMicroseconds(1);
  digitalWrite(step_pin[moteur], LOW);
}

//Private function, we dont want our target compter to be 1 cuz it sux 
unsigned int Stepper_moteur::born_timer(unsigned int in_timer)
{
  if(in_timer < 99)return 99;
  else if (in_timer > 65000)return 65000;
  else return in_timer;
}