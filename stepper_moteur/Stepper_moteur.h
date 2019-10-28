#ifndef Stepper_moteur_h
#define Stepper_moteur_h

#include<Arduino.h>

//Must precise the number of motor used to not loose time in the interuption routine
#define number_of_motor  2

class Stepper_moteur
{
  public:
  //Public Fonctions
    Stepper_moteur();
    void set_periode(byte m, float speed);                  //Void to set the periode for each stepper    
    void initialize_stepper(byte m, byte step, byte dir);   //Void to initialize a stepper, specifiyng the ports it is conected to
    void moove(byte m);                                      //Desired motor makes a step
    
    void initialize_timer();                                //Fonctions init pour le timer1 de l'atmega328p , a placé dans le setup
    void timer_interupt();                                  //Fonction d'interuption a place dans une fonction declaré comme cela : ISR(TIMER1_COMPA_vect){timer_interupt()}
    unsigned int actual_OCR1A = 65535 ;
    unsigned int timer_compteur[number_of_motor] ;  //Timer to remember how much time is left before the next step
    byte dir_pin[number_of_motor];                  //Direction pin attributions
    byte step_pin[number_of_motor];                 //Step pin attributions
    unsigned int timer_consigne[number_of_motor];   //Store the periode of each motor
    
  //Public Variables

  private:
    //Private Functions

    //Private Variables
    bool moteur_actifs[number_of_motor];            //Boolean to know if the motor as to spin or not
    bool moteur_direction[number_of_motor];         //Boolean to know, if the  motor is spinning, in wich way he spins
    
    
};

#endif
