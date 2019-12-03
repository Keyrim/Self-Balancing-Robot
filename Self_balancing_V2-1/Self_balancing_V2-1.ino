//Third version, this time using the stepper_moteur class wich is based on interuption timer
//Both motors are handeled as once

//Include the class 
#include "Stepper_moteur.h"
//Library for the MPU
#include <Wire.h>   
unsigned long loop_timer = 0 ;

//Variable resistor pin
#define pin_Analog_P 1
#define pin_Analog_I 2
#define pin_Analog_D 3

//Motor's pins
#define pin_left_step 11
#define pin_right_step 10
#define pin_left_dir 12
#define pin_right_dir 9


#define ACY_compensation 2.6 //Si part trop en avant à augmenter si trop en arrière à diminuer

//Pid 
#define angle_max 30                         //The robot doesnt compensate anymore after this angle
#define angle_min 5
float kP = 170 , kI = 12, kD = 0 ;
float p, i = 0 , d ;
float consigne = 0 ;
float previous_error = 0 , error ;

#define min_kP 140
#define max_kP 220

#define min_kI 5
#define max_kI 15

#define min_kD 0
#define max_kD 10

//Create a stepper_moteur object
Stepper_moteur stepper ;

//Read angle 
const int MPU=0x68;  // I2C address of the MPU-6050
const int frequence = 250; //Frequence of the loop in hz
//raw data
float AcX, AcY, AcZ, GyX=0, GyY=0, GyZ=0;
float X=0, Y=0;



/*
    states
0   Arret, moteur ON , changement par bouton
1   En marche, compensation active 
2   Dead band, angle trop faible
3   Arret, moteur ON, angle trop important
*/

byte state = 1 ;

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
    Serial.begin(1000000);
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

    //Initialize our stepper_class 
    stepper.initialize_stepper(0,pin_left_step, pin_left_dir );
    stepper.initialize_stepper(1,pin_right_step, pin_right_dir );
    stepper.set_speed(0, 2000);
    stepper.set_speed(1, 2000);
    stepper.initialize_timer();
    stepper.set_micro_stepping(8);
    //Initialize the MPU6050

    

}

ISR(TIMER1_COMPA_vect)
{     
    stepper.timer_interupt();
    
}

void loop()
{
    read_mpu();                             //Get angles and accélérations
    float total_vector = sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);    
    AcX = -asin(AcX/total_vector)*57.32; 
    AcY = asin(AcY/total_vector)*57.32; 
    
    //Complementary filter
    Y += GyX / frequence ;
    Y = Y * 0.996 + (AcY + ACY_compensation) * 0.004;   
        
    if(abs(error) > angle_max  )
        //coupe les moteurs car angle trop important
        state = 2;
    else if(abs(error) < angle_min)
        //On stope la compensation car angle trop faible
        state = 2;
    else 
        //On est dans l'interval donc on compense
        state = 1;

    //Guess the P I D values aacording to the potentiometer
    kP = (long)map(analogRead(pin_Analog_P), 0, 1023, (long)min_kP*1000, (long)max_kP*1000) / 1000.0;
    kI = (long)map(analogRead(pin_Analog_I), 0, 1023, (long)min_kI*1000, (long)max_kI*1000) / 1000.0;
    kD = (long)map(analogRead(pin_Analog_D), 0, 1023, (long)min_kD*1000, (long)max_kD*1000) / 1000.0;

    switch(state){

    case 0 :    //Moteur à l'arret on attent le switch 
        stepper.set_speed(0, 0); 
        break;

    case 1 :    //Compensation active
        //PID computation   
        error = Y - consigne ;                  //Compute the error      
        p = kP * error ;                        //Compute the P compensation
        d = kD * (error - previous_error);      //----------- D compensation
        i += kI * error ;                       //----------- I compensation
        previous_error = error ;                //Update the previous error 
        stepper.set_speed(0, p+i);              //Update the speed for each motors        
        break;

    case 2 :    //Angle trop faible ou trop fort on ne compense pas 
        stepper.set_speed(0, 0); 
        break;
    }
    Serial.print(kP);
    Serial.print("\t");
    Serial.print(kI);
    Serial.print("\t");
    Serial.println(kD);

    //Frequence regulation here 
    while(micros()<loop_timer + 1000000/frequence);
    loop_timer = micros();
    
}


