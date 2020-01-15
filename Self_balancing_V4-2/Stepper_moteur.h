#ifndef Stepper_moteur_h
#define Stepper_moteur_h
#include<Arduino.h>
/*
#define pin_right_step 9 // also 
#define pin_right_dir 10
#define pin_left_step 11
#define pin_left_dir 12
*/
//Equivalent to digitalWrite but faster theoricaly
#define left_dir 4
#define left_dir_high (PORTB |= B00010000)
#define left_dir_low (PORTB &= B11101111)

#define right_dir 2
#define right_dir_high (PORTB |= B00000100)
#define right_dir_low (PORTB &= B11111011)

#define left_step 3
#define left_step_high (PORTB |= B00001000)
#define left_step_low (PORTB &= B11110111)

#define right_step 1 
#define right_step_high (PORTB |= B00000010)
#define right_step_low (PORTB &= B11111101)

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
  long  it_duration = 0 ;

  private:
  //Private Functions

    unsigned int born_timer(unsigned int in_timer);         //We dont want our period to be to short or to long
    void moove(byte m);                                     //Desired motor makes a step

  //Private Variables

    bool moteur_actifs[number_of_motor];            //Boolean to know if the motor as to spin or not
    bool moteur_direction[number_of_motor];         //Boolean to know, if the  motor is spinning, in wich direction it spins
    bool previous_moteur_direction[number_of_motor];//To check if we need to digital_write again or if it is the same as before
    unsigned int actual_OCR1A = 59 ;
    int previous_speed = 0 ;                        //Used to check if the acceleration is not to high for that por motor, not used but w/e will be used soon , i think, theoricaly 

    //Utilisé niveau de l'interuption
    unsigned int timer_compteur[number_of_motor] ;  //Timer to remember how much time is left before the next step        
    
    byte micrro_stepping_div = 1 ;                  //Store the micro stepping setting
    
    
};

#endif
