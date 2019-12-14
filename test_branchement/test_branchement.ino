#define step_pin 11
#define dir_pin 12

void setup()
{
    pinMode(step_pin, OUTPUT);
    pinMode(dir_pin, OUTPUT);
    digitalWrite(dir_pin, LOW);
}

void loop()
{
    digitalWrite(step_pin, HIGH);
    delay(1);    
    digitalWrite(step_pin, LOW);
    delay(1);
    

}
