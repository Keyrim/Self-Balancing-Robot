#ifndef Stepper_moteur_h
#define Stepper_moteur_h
#include<Arduino.h>

//Micro stepping 
#define MS3 8
#define MS2 7
#define MS1 6

//Acceleration in degrees per seconds squared
#define acceleration_motor_max 50 
#define max_speed 3000



//Must precise the right number of motor used to not loose time in the interuption routine
#define number_of_motor  2
//Degrees for one step, we then divide this value by micrro_stepping_div to know what the real angle is actualy
#define dregrees_per_steps 1.8

class Stepper_moteur
{
  public:
  //Public Fonctions
    Stepper_moteur();
    void set_speed(byte m, float speed);                  //Void to set the periode for each stepper    
    void initialize_stepper(byte m, byte step, byte dir);   //Void to initialize a stepper, specifiyng the ports it is conected to
    void set_micro_stepping(byte divider);                  //Void that set up 
    
    void initialize_timer();                                //Fonctions init pour le timer1 de l'atmega328p , a placé dans le setup
    void timer_interupt();                                  //Fonction d'interuption a place dans une fonction declaré comme cela : ISR(TIMER1_COMPA_vect){timer_interupt()}

    byte dir_pin[number_of_motor];                  //Direction pin attributions
    byte step_pin[number_of_motor];                 //Step pin attributions
    
    
  //Public Variables
  unsigned int timer_consigne[number_of_motor];   //Store the periode of each motor

  private:
  //Private Functions

    unsigned int born_timer(unsigned int in_timer);         //We dont want our period to be to short or to long
    void moove(byte m);                                     //Desired motor makes a step

  //Private Variables

    bool moteur_actifs[number_of_motor];            //Boolean to know if the motor as to spin or not
    bool moteur_direction[number_of_motor];         //Boolean to know, if the  motor is spinning, in wich way he spins
    unsigned int actual_OCR1A = 65535 ;
    int previous_speed = 0 ;                        //Used to check if the acceleration is not to high for that por motor 

    //Utilisé niveau de l'interuption
    unsigned int timer_compteur[number_of_motor] ;  //Timer to remember how much time is left before the next step        
    
    byte micrro_stepping_div = 1 ;                  //We devide 1,8 by this to know what a step means in term of degrees, required for the speed 
    
    
};

#endif
