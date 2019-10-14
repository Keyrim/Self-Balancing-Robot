//Program forthe balancing robot
//Library for the MPU
#include <Wire.h>   
unsigned long loop_timer = 0 ;

//Motor conections
//A4988 steps settings



//Motor's pins
#define pin_left_step 11
#define pin_right_step 10
#define pin_left_dir 12
#define pin_right_dir 9

//Read angle 
const int MPU=0x68;  // I2C address of the MPU-6050
const int frequence = 250; //Frequence of the loop in hz
//raw data
float AcX, AcY, AcZ, GyX=0, GyY=0, GyZ=0;
float X=0, Y=0;

//Moove side motors
void moove_left(bool dir)
{
    digitalWrite(pin_left_dir, dir);
    digitalWrite(pin_left_step, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_left_step, LOW);
}
void moove_right(bool dir)
{
    digitalWrite(pin_right_dir, dir);
    digitalWrite(pin_right_step, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin_right_step, LOW);
}


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





void setup()
{
    //Serial.begin(115200);

    //Setup the motors
    for(byte o = 9 ;  o <= 12; o++)
        pinMode(o, OUTPUT);
    
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
    //On lui un peu de temps sinon le mpu est pas bien init
    delay(100);

    //Setup interuptions 
    cli();//stop interrupts
    //set timer1 interrupt at 1Hz
    TCCR1A = 0;// set entire TCCR1A register to 0
    TCCR1B = 0;// same for TCCR1B
    TCNT1  = 0;//initialize counter value to 0
    // set compare match register for 1hz increments
    OCR1A = 15999;// = ( 16000000 / (prescaler * fre cible))
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    //  CS10 bits for 1prescaler
    TCCR1B |=  (1 << CS10);      
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A); 
    sei();//allow interrupts
    

}
ISR(TIMER1_COMPA_vect)
{
    
    
    moove_right(true);
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
    
    //speed setting
    
    
    unsigned tic = abs(Y) * 1000 ;
    //Serial.println(tic);
    if(tic < 13000)tic = 13000 ;
    if (tic > 65000)tic = 65000 ;
    

    OCR1A = tic ;
    
    
    //Frequence regulation here 
    while(micros()<loop_timer + 1000000/frequence);
    loop_timer = micros();
    
    
    
    

}