//Third version, this time using the stepper_moteur class wich is based on interuption timer

//Include the class 
#include "Stepper_moteur.h"
//Library for the MPU
#include <Wire.h>   
unsigned long loop_timer = 0 ;

//Motor's pins
#define pin_left_step 11
#define pin_right_step 10
#define pin_left_dir 12
#define pin_right_dir 9



//Create a stepper_moteur object
Stepper_moteur stepper ;

//Read angle 
const int MPU=0x68;  // I2C address of the MPU-6050
const int frequence = 250; //Frequence of the loop in hz
//raw data
float AcX, AcY, AcZ, GyX=0, GyY=0, GyZ=0;
float X=0, Y=0;

//Pid 
#define angle_max 35                         //The robot doesnt compensate anymore after this angle
#define angle_min 2
const float kP = 200 , kI = 0 , kD = 0 ;
float p, i = 0 , d ;
float consigne = 0 ;
float corection = 0 ;
float previous_error = 0 , error ;

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
    Serial.begin(115200);
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
    Y = Y * 0.996 + (AcY + 2.33) * 0.004;   

    if(abs(Y) < angle_max && abs(Y) > angle_min)
    {
        //PID computation
        error = Y - consigne ;                  //Compute the error 
        p = kP * error ;                        //Compute the P compensation
        d = kD * (error - previous_error);      //----------- D compensation
        //I missing                             //----------- I compensation 
        previous_error = error ;                //Update the previous error 

        stepper.set_speed(0, p);              //Update the speed for each motors
        stepper.set_speed(1, p);
    }
    else 
    {
        //Update the speed for each motors
        stepper.set_speed(0, 0);              
        stepper.set_speed(1, 0);
    }
    //Serial.println(Y);

    //Frequence regulation here 
    while(micros()<loop_timer + 1000000/frequence);
    loop_timer = micros();
    
}


