#define step_pin 4
#define sens_pin 5


void setup()
{
    pinMode(4, OUTPUT);
    pinMode(sens_pin, OUTPUT);
    digitalWrite(sens_pin, LOW);
}

void loop()
{
    digitalWrite(4, HIGH);
    delay(500);