//Program forthe balancing robot
//Library for the MPU
#include <Wire.h>   

//Motor conections
#define pin_sens_m1 2
#define pin_sens_m2 3
#define pin_step_m1 4
#define pin_step_m2 5

//Read angle 
const int frequence = 250; //Frequence of the loop in hz
//raw data
float AcX, AcY, AcZ, GyX=0, GyY=0, GyZ=0;
float X=0, Y=0;




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

void step(int pin, bool sens)
{
    digitalWrite(pin, sens);
}



void setup()
{
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

}

void loop()
{
    read_mpu();
    //Compute our raw values
    float total_vector = sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);    
    AcX = -asin(AcX/total_vector)*57.32; 
    AcY = asin(AcY/total_vector)*57.32;
    
    //Complementary filter
    X += GyX / frequence ; 
    X = X * 0.996 + AcY * 0.004 ;
    Y += GyY / frequence ;
    Y = Y * 0.996 + AcX * 0.004;

    //Frequence regulation here 
    while(micros()<loop_timer + 1000000/frequence);
    loop_timer = micros();

}