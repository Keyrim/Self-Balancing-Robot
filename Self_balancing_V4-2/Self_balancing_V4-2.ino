//Description
//5th version, GOAL : same as the latest one but now :
//-- Motors are not controlled as one motor anymore, this way we can turn ehe 
//-- The main idea here is to see what is the max computation power i have 
//-- Once i know how much computation power i have left, we know if we swap on an STM32, or if i keep the arduino with a raspberry pi zero module to make wireless etc 
//-- This new code has to : 
//      TURN :                              DONE
//      Log w/e i want on a raspberry :     TODO


// ----------------------------------------------TASK LIST--------------------------------------------
//IS the MAE worth it ? probably not                                                    TODO
//Motors working separately                                                             DONE
//Control digital outputs using registers                                               DONE 
//Turning on a side                                                                     DONE 
//Smooth the RC input with a mooving average                                            IN PROGRESS
//Limit PID outputs                                                                     TODO

#include <SoftwareSerial.h>

//Include the class 
#include "Stepper_moteur.h"
//Library for the MPU
#include <Wire.h>   
unsigned long loop_timer = 0 ;


//Variable resistor pin
#define pin_Analog_P 3
#define pin_Analog_I 6
#define pin_Analog_D 7

//Motors Pinout, motor 0 is the left one and motor 1 is the right one 
#define pin_right_step 9 // also 
#define pin_right_dir 10
#define pin_left_step 11
#define pin_left_dir 12


//Gloabl Parameters
#define ACY_compensation 4 //Si part trop en avant à augmenter si trop en arrière à diminuer
#define angle_max 20
#define angle_min 0 

//Pid consigne vitesse moteur avec en entrée l'angle 
const float kP_vitesse = 7.8 , kI_vitesse = 0 , kD_vitesse = 50 ;
float p_vitesse = 0, i_vitesse = 0 , d_vitesse = 0 ;
float pid_vitesse = 0 ;
float consigne_angle = 0 ;
float previous_error_angle = 0 , error_angle = 0 ;
//Parameters
#define max_kP_vitesse 20
#define max_kI_vitesse 0.001
#define max_kD_vitesse 50
#define max_pid_vitesse max_speed  //Max_speed is defined in sttpper_moteur.has

//Pid consigne angle 
float kP_angle = 0, kI_angle= 0 , kD_angle = 0 ;
float p_angle = 0, i_angle = 0, d_angle = 0 ;
float pid_angle = 0 ;
float consigne_vitesse = 0 ;
float error_vitesse = 0, previous_error_vitesse = 0 ;
byte pid_angle_compteur = 0 ;
//Parameters
//Coefficients divisé par 1000 à terme 
#define max_kP_angle 10 
#define max_kI_angle 0.1
#define max_kD_angle 70
#define max_pid_angle 20 


//bluetooth 
#define BTSPEED 115200
#define rx 3
#define tx 4
SoftwareSerial bt(tx, rx); 

//RC
#define interupt_ppm_pin 2
//Read PPM signal
const byte number_of_chanel = 11 ;
byte actual_chanel = 0 ;
unsigned long previous_timer_rising = 0 ;
unsigned int  chanels[number_of_chanel] ;

int tourner = 100 ;

//Create a stepper_moteur object    
Stepper_moteur stepper ;

//Read angle 
const int MPU=0x68;  // I2C address of the MPU-6050
const int frequence = 250; //Loop Frequence in hz
//raw data
float AcX, AcY, AcZ, GyX=0, GyY=0, GyZ=0;
float X=0, Y=0;

/*
    -------------- states ------------------
0   Arret, moteur ON , changement par bouton
1   En marche, compensation active 
2   Dead band, angle trop faible
3   Arret, moteur ON, angle trop important
*/

byte state = 1 ;

float borne_float(float to_born, float min, float max)
{
    if(to_born > max)
        return max;
    if(to_born < min)
        return min ;
    return to_born ;
}

float borne_long(long to_born, long min, long max)
{
    if(to_born > max)
        return max;
    if(to_born < min)
        return min ;
    return to_born ;
}



//read the mpu 
void read_mpu()
{
    
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);       //Send the starting register (accelerometer)
    Wire.endTransmission();
    Wire.requestFrom(MPU, 6);
    while (Wire.available()< 6);
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();
    AcZ = Wire.read()<<8|Wire.read();

    //Get the true raws values in g according to our setting
    AcZ /= 8192;
    AcX /= 8192;
    AcY /= 8192;
    
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission();
    Wire.requestFrom(MPU, 6);

    while(Wire.available()< 6);
    GyX = (Wire.read()<<8|Wire.read())/65.5;
    GyY = (Wire.read()<<8|Wire.read())/65.5;  
    GyZ = (Wire.read()<<8|Wire.read())/65.5;  
    

} 


void setup()
{
    
    //                                                      SERIAL PART
    Serial.begin(1000000);
    Serial.println("c est le debut");
    /*
    //Bt conection
    bt.begin(BTSPEED);
    bt.println("Début du programe");
    */
    //                                                      MPU6050 INIT PART
    // Wake up the mpu 
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission();

    //Set up the accelerometer
    //1g = 8192
    //+- 4g     
    Wire.beginTransmission(MPU);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission();

    //Set up the gyroscope
    //1deg/s = 65.5 
    //+- 500
    Wire.beginTransmission(MPU);
    Wire.write(0x1B);
    Wire.write(0X08);
    Wire.endTransmission();

    delay(100);

    //                                                      STEPPER INIT PART PART
    //Initialize our stepper_class 
    stepper.initialize_stepper(0,pin_left_step, pin_left_dir );         //Define the stepper 0 as the left one 
    stepper.initialize_stepper(1,pin_right_step, pin_right_dir );       //And the stepper 1 as the right one
    stepper.set_speed(0, 0);
    stepper.set_speed(1, 0);
    stepper.initialize_timer();
    stepper.set_micro_stepping(4);


    //                                                      PPM IT INIT PART
    attachInterrupt(digitalPinToInterrupt(interupt_ppm_pin), rising, RISING);
    

}

ISR(TIMER1_COMPA_vect)
{     
    stepper.timer_interupt();
    
}

void loop()
{
    // ================================ ANGLE - MPU 6050 ==========================================
    read_mpu();                             //Get angles and accélérations
    float total_vector = sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);    
    AcX = -asin(AcX/total_vector)*57.32; 
    AcY = asin(AcY/total_vector)*57.32; 
    
    //Complementary filter
    Y += GyX / frequence ;
    Y = Y * 0.996 + (AcY + ACY_compensation) * 0.004;   
    

    // ================================= STATE DETERMINATION ======================================
    if(abs(Y) > angle_max || chanels[5] < 1500 )
        //coupe les moteurs car angle trop important
        //ou switch télécomande désactivé 
        state = 2;
    else if(abs(Y) < angle_min)
        //On stope la compensation car angle trop faible
        state = 2;
    else 
        //On est dans l'interval donc on compense
        state = 1;

    //Guess the P I D values aacording to the potentiometer
    kP_angle = max_kP_angle - ((float)analogRead(pin_Analog_D)*max_kP_angle)/1023.0;
    kI_angle = max_kI_angle - ((float)analogRead(pin_Analog_I)*max_kI_angle)/1023.0;
    kD_angle = max_kD_angle - ((float)analogRead(pin_Analog_P)*max_kD_angle)/1023.0;

    //Vitesse_consigne 
    consigne_vitesse = -7 * ((int)chanels[2] - 1500) ;
    tourner =  ((int)chanels[1] - 1500) ;


    // ================================== MAIN SWITCH ===============================================
    switch(state){
    case 0 :    //Moteur à l'arret on attent le switch 
        stepper.set_speed(0, 0); 
        stepper.set_speed(1, 0); 
        break;

    case 1 :    //Compensation active

        //======================== PID COMPUTATION ======================

        //Premier pid on corige la consigne de l'angle 
        error_vitesse = consigne_vitesse - pid_vitesse ;
        p_angle = kP_angle * error_vitesse ;
        i_angle += kI_angle * error_vitesse ;
        d_angle = kD_angle * (error_vitesse - previous_error_vitesse);
        previous_error_vitesse = error_vitesse ;
        pid_angle = p_angle + i_angle + d_angle ;

        //this part allows the system to wait 1 sec before starting to follow the speed order given by the user on the rc remote
        //This way, the system has the time to stabalize it self on the center and then start mooving
        if(pid_angle_compteur < 250){
            pid_angle_compteur ++ ;
            pid_angle = 0 ;
        }else
            pid_angle /= 1000 ;
        
        pid_angle = borne_float(pid_angle, -max_pid_angle, max_pid_angle);
        consigne_angle = pid_angle ;
        

        //Second pid on corige la consigne de la vitesse des roues / des moteurs
        error_angle = Y - consigne_angle ;                                          //Compute the error      
        p_vitesse = kP_vitesse * error_angle ;                                      //Compute the P compensation
        i_vitesse += kI_vitesse * error_angle ;                                     //----------- I compensation
        d_vitesse = kD_vitesse * (error_angle - previous_error_angle);              //----------- D compensation
        previous_error_angle = error_angle ;                                        //Update the previous error 
        pid_vitesse += p_vitesse + i_vitesse + d_vitesse ;                          //Pid comande l'accélération, on doit l'intégre pour avoir la consigne de la vitesse d'où la somme 
        pid_vitesse = borne_float(pid_vitesse, -max_pid_vitesse, max_pid_vitesse);

        //Update the motor's speed    
        // Speed in oppostie direction because motors are in my case conected in the opposite way
        stepper.set_speed(0, -pid_vitesse + tourner);                
        stepper.set_speed(1, pid_vitesse + tourner); 

        break;

    case 2 :    //Angle trop faible ou trop fort on ne compense pas 
        stepper.set_speed(0, 0);
        stepper.set_speed(1, 0); 

        //Set every thing back to zero as a reset would do 
       
        pid_angle_compteur = 0 ;

        pid_vitesse = 0 ;
        p_vitesse = 0 ;
        i_vitesse = 0 ; 
        d_vitesse = 0 ;
        error_vitesse = 0 ;
        previous_error_vitesse = 0 ;

        pid_angle = 0;
        p_angle = 0 ;
        i_angle = 0 ;
        d_angle = 0 ;
        error_angle = 0;
        previous_error_angle = 0 ;

        break;
    }


    //Afficher les donné que l'on veut 
    /*
    Serial.print(Y);
    Serial.print("\t");
    Serial.print(pid_vitesse);
    Serial.print("\t");
    Serial.println(kD_angle);
    */
    //bt.println((String)chanels[1]+ ";" + (String)kD_angle +  ";" + (String)d_angle ); //+ ";" + (String)(micros() - loop_timer)); 

    //
    //Serial.println((String)stepper.it_duration );
    //Serial.println(pid_vitesse);
    //Frequency regulation
    Serial.println(chanels[2]);
    while(micros()<loop_timer + 1000000/frequence);
    loop_timer = micros();
    
}


//This function is called when the pin 2 state changes
void rising()
{
  unsigned int signal_ppm = micros()-previous_timer_rising ;
  if (signal_ppm > 3000) 
  {
    actual_chanel = 0 ;
    previous_timer_rising = micros();
    chanels[actual_chanel] = signal_ppm ;
  }  

  else 
  {
    actual_chanel ++;
    if (actual_chanel == number_of_chanel) actual_chanel = 0 ;
    chanels[actual_chanel] = signal_ppm ;
    previous_timer_rising = micros();
  }  
  
}

