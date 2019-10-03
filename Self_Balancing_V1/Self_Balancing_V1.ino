//Program forthe balancing robot
//Library for the MPU
#include <Wire.h>   
unsigned long loop_timer = 0 ;

//Motor conections
//A4988 steps settings



//Motor 1 is the right motor and m2 is the left one 
#define pin_sens_m1 3
#define pin_sens_m2 4
#define pin_step_m1 2
#define pin_step_m2 5

//Read angle 
const int MPU=0x68;  // I2C address of the MPU-6050
const int frequence = 250; //Frequence of the loop in hz
//raw data
float AcX, AcY, AcZ, GyX=0, GyY=0, GyZ=0;
float X=0, Y=0;

//Moove side motors
void left_step(bool sens)
{
    digitalWrite(pin_sens_m2, sens);
    digitalWrite(pin_step_m2, HIGH);
    delay(LOW);
    digitalWrite(pin_step_m2, LOW);
}
void right_step(bool sens)
{
    digitalWrite(pin_sens_m1, sens);
    digitalWrite(pin_step_m1, HIGH);
    delay(LOW);
    digitalWrite(pin_step_m1, LOW);
}

//Timmer motor speed 
unsigned long pre_timer_left = 0 ;
unsigned long pre_timer_right = 0 ;
//The speed is defined in degrees per seconds 
int left_speed = 0 ;
int right_speed = 0 ; 


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
    Serial.begin(115200);
    Serial.println("Init 1");
    
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
    Serial.println("End init");

}

void loop()
{
    read_mpu();
    //Compute our raw values
    float total_vector = sqrt(AcX*AcX + AcY*AcY + AcZ*AcZ);    
    AcX = -asin(AcX/total_vector)*57.32; 
    AcY = asin(AcY/total_vector)*57.32;
    
    //Complementary filter
    Y += GyY / frequence ;
    Y = Y * 0.996 + AcX * 0.004;
    Serial.println(Y);

    if(Y >= 10)
    {
        if(micros() > pre_timer_left +400)
        {
            left_step(true);
            pre_timer_left = micros();
        }
    }

    //Frequence regulation here 
    while(micros()<loop_timer + 1000000/frequence);
    loop_timer = micros();

}