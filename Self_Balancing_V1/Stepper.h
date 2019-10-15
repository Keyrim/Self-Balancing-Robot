#ifndef Stepper_h
#define Stepper_h
#include "Arduino.h"

//Must precise the number of motor used to not loose time in the interuption routine
#define number_of_motor  2

class Stepper
{
    public:
        Stepper();   
        void set_periode(byte m, float speed);
        void initialize_stepper(byte m, byte step, byte dir);
        unsigned int timer_consigne[number_of_motor];
        
    private:
        bool moteur_actifs[number_of_motor];
        bool moteur_direction[number_of_motor];
        unsigned int timer_compteur[number_of_motor] ;
        byte dir_pin[number_of_motor];
        byte step_pin[number_of_motor];
};
#endif
