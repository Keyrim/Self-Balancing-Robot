//Classe qui permet de controller un nombre donné de moteur pas à pas par interuptions timer 
#ifndef Stepper_h
#define Stepper_h
#include "Arduino.h"

//Must precise the number of motor used to not loose time in the interuption routine
#define number_of_motor  2

class Stepper
{
    public:
        //          Functions
        Stepper();   
        //Void to set the periode for each stepper
        void set_periode(byte m, float speed);
        //Void to initialize a stepper, specifiyng the ports it is conected to
        void initialize_stepper(byte m, byte step, byte dir);

        //Fonctions liées aux timers
        void initialize_timer();
        void timer_interupt();

        //        Variables
        
        
        
    private:
        //        Functions
    
        //        Variables 
        bool moteur_actifs[number_of_motor];            //Boolean to know if the motor as to spin or not
        bool moteur_direction[number_of_motor];         //Boolean to know, if the  motor is spinning, in wich way he spins
        unsigned int timer_compteur[number_of_motor] ;  //Timer to remember how much time is left before the next step
        byte dir_pin[number_of_motor];                  //Direction pin attributions
        byte step_pin[number_of_motor];                 //Step pin attributions
        unsigned int timer_consigne[number_of_motor];   //Store the periode of each motor
};
#endif
