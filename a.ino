#define step 10
#define dir 9


void setup()
{
  
  //set pins as outputs
  pinMode(step, OUTPUT); //Step
  pinMode(dir, OUTPUT);  //dir
  digitalWrite(dir, HIGH);
cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1439;// = ( 16000000 / (prescaler * fre cible))
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //  CS10 bits for 1prescaler
  TCCR1B |=  (1 << CS10);      
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A); 


sei();//allow interrupts

}//end setup

ISR(TIMER1_COMPA_vect)
{
    digitalWrite(step, HIGH);
    delayMicroseconds(10);
    digitalWrite(step, LOW);
}
  


void loop()
{
  //do other things here
}