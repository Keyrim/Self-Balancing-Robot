//Version 0 supposé suivante à la version "précurseur"
//Rien d'ineressant
//Mesure de l'angle avec l'accéléromètre et une moenne constante_moyenne_glissante
//Période des moteurs avec un delayMicroseconds



#include<Wire.h>
float angleMoy, angle;
const byte constante_moyenne_glissante = 20 ;
float somme_angles = 0 ;
float valeurs[constante_moyenne_glissante];
byte n = 0 ;


float angle_mpu()
{
  float AcX, AcY, AcZ, AcTotal ;
  //Debut liason i2c avec le mpu
  Wire.beginTransmission(0x68);                                        
  Wire.write(0x3B);                                                    
  Wire.endTransmission();                                             
  Wire.requestFrom(0x68,6); 
  while(Wire.available() < 6);  
  //Lit les données du mpu 
  AcX = Wire.read()<<8|Wire.read();  
  AcY = Wire.read()<<8|Wire.read();  
  AcZ = Wire.read()<<8|Wire.read(); 
  AcTotal = sqrt(AcX*AcX + AcZ*AcZ + AcY*AcY);
  AcX = asin(AcX/AcTotal) * 57.296 ;
  AcY = asin(AcY/AcTotal) * -57.296 ;
  return AcX ;
   
}



void setup() {
  for(byte n = 2; n < 10; n++)pinMode(n, OUTPUT);
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

}

void loop() {
  constante_moyenne_glissante;
  //Serial.print(micros());
  //Serial.print("----------");
  
  somme_angles -= valeurs[n];
  valeurs[n] = angle_mpu();
  somme_angles += valeurs[n];
  n++ ;
  if (n >= constante_moyenne_glissante)n = 0 ;
  angle = somme_angles / constante_moyenne_glissante ;
  //Serial.println(micros());
  
  
  digitalWrite(9, HIGH);
  digitalWrite(9, LOW);
  digitalWrite(8, LOW);
  delayMicroseconds(map(abs(angle), 0, 90, 4000, 00));
}
