#define step 10
#define dir 9
//The code is made for an increasing speed and not a decresaing number_of_increment
//Must be carefull if using a decreasing speed
#define starting_period 1000
#define increment 100
#define number_of_increment 16
#define step_every_speed 500

void setup()
{
    pinMode(step, OUTPUT);
    pinMode(dir, OUTPUT);
    digitalWrite(dir, HIGH);
    
}

void loop()
{
    for (int i = 0; i < number_of_increment ; i++)
    {
        for (int n = 0; n < step_every_speed; n++)
        {
            digitalWrite(step, HIGH);
            delayMicroseconds(starting_period + i * increment / 2);
            digitalWrite(step, LOW);
            delayMicroseconds(starting_period + i * increment / 2);
        }
        delay(500);
    }
}