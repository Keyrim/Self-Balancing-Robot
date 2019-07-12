
//Librairie pour le servo
#include<Wire.h>

//Variable pour le gyro et calcule angle
const unsigned char constante_moyenne_glissante = 50 ;
unsigned char i = 0;
int valeursX[constante_moyenne_glissante];
int valeursY[constante_moyenne_glissante];
float somme[2] ;    //0 pour x et 1 pour y
float angles[2] ;    //0 pour x et 1 pour y 
const int MPU=0x68;  // I2C address of the MPU-6050

//Fonction qui met a jour la valeur du tableau angle
void update_angles()
{
  float AcX,AcY,AcZ,GyX,GyY,GyZ,AcTotal, Tmp;
  float AcXangle , AcYangle;
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  AcTotal = sqrt(AcX*AcX + AcZ*AcZ + AcY*AcY);
  AcXangle = asin(AcX/AcTotal) * 57.296 ;
  AcYangle = asin(AcY/AcTotal) * -57.296 ;
  somme[0]-=valeursY[i];
  somme[1]-=valeursX[i];
  valeursX[i] = AcXangle ;
  valeursY[i] = AcYangle ; 
  somme[1]+= valeursX[i];
  somme[0]+= valeursY[i];
  angles[0] = somme[1] / constante_moyenne_glissante; // constante_moyenne_glissante
  angles[1] = somme[0] / constante_moyenne_glissante; // constante_moyenne_glissante
  i++;
  if(i == constante_moyenne_glissante ) i = 0 ;
}

//Varaibles pour le controle des moteurs
byte state_stepper[2] = {0,0} ;
float vitesse_stepper[2] ;
int sens_stepper[2] ;
int vitesse_mini = 700 ;
int vitesse_max = 6000 ;
unsigned long timer_moteur[2] ;

//Fonction pour faire avancer un moteur
void stepper_step(int moteur,   int sens)
{
  unsigned int analog_value = 200 ;
  if( state_stepper[moteur] == 0 )
  {
    analogWrite(2+4*(moteur), analog_value);
    analogWrite(3+4*(moteur), 0);
    analogWrite(4+4*(moteur), analog_value);
    analogWrite(5+4*(moteur), 0);
    
  }
  else if( state_stepper[moteur] == 64 )
  {
    analogWrite(2+4*(moteur), 0);
    analogWrite(3+4*(moteur), analog_value);
    analogWrite(4+4*(moteur), analog_value);
    analogWrite(5+4*(moteur), 0);
  }
  else if( state_stepper[moteur] == 128 )
  {
    analogWrite(2+4*(moteur), 0);
    analogWrite(3+4*(moteur), analog_value);
    analogWrite(4+4*(moteur), 0);
    analogWrite(5+4*(moteur), analog_value);
  }
  else if( state_stepper[moteur] == 192 )
  {
    analogWrite(2+4*(moteur), analog_value);
    analogWrite(3+4*(moteur), 0);
    analogWrite(4+4*(moteur), 0);
    analogWrite(5+4*(moteur), analog_value);
  }
  state_stepper[moteur] += 64 * sens  ;
}


void bouge(int stepper, int sens)
{
  digitalWrite(22+2*stepper, sens);
  digitalWrite(23+2*stepper, HIGH);
  delayMicroseconds(50);
  digitalWrite(23+2*stepper, LOW);
}


void setup() {
  //On envoit les requetes pour "reveiller" le servo
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  //Initialise la liaison série a une vitesse de 9600 bauds/s
  Serial.begin(9600);

  //Initialise les pin sen sortie de 2 a 9( 8 pins , 4 par moteur)
  for( int i = 2; i<30; i++)pinMode(i, OUTPUT);
  for(int i = 0; i<2; i ++) timer_moteur[i] = micros();

}

void loop() {
  //update_angles();
  float angle = angles[1];
  /*for (int i = 0; i<2; i++)
  {
    //On definit le sens de rotation du moteur
    if(angle > 0) sens_stepper[i] = 1 ;
    else sens_stepper[i] = 0 ;
    //On definit la vitesse du moteur, PID a faire ici
    vitesse_stepper[i] = 1000;
    if ( vitesse_stepper[i] > vitesse_max) vitesse_stepper[i] = vitesse_max ;
    if ( vitesse_stepper[i] < vitesse_mini) vitesse_stepper[i] = vitesse_mini ;
  }*/

  //Ici c'est pour faire tourner les moteurs qaund il le faut 
  for (int i = 0; i<2; i ++)
  {
    if(micros() >timer_moteur[i]+vitesse_stepper[i])
    {
      if(sens_stepper[i] !=-1)
      {
        bouge(i, 1);
        timer_moteur[i] = micros();
      }
    }
  }

 

}
