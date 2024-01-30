/*
This sample code is for testing the 2 stepper motors
The rotation velocity can be adjusted by the code switch
Microcontroller: Arduino UNO
*/
int M1dirpin = 7; //Motor X direction pin
int M1steppin = 6; //Motor X step pin
int M1en=8; //Motor X enable pin
int M2dirpin = 4; //Motor Y direction pin
int M2steppin = 5; //Motor Y step pin
int M2en=12; //Motor Y enable pin

int Direction = 1; //Control Motor Direction
int StepRate = 3; //Control Rotational Speed

void setup()
{
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M1en,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
  pinMode(M2en,OUTPUT);
  digitalWrite(M1en,LOW);// Low Level Enable
  digitalWrite(M2en,LOW);// Low Level 
  //digitalWrite(M1dirpin,LOW);
  //digitalWrite(M2dirpin,LOW);     
}

void loop()
{
  int i;

  for(i=0;i<=1250;i++)
  {
    digitalWrite(M1steppin,LOW);
    digitalWrite(M2steppin,LOW);
    delayMicroseconds(2);
    digitalWrite(M1steppin,HIGH); //Rising step
    digitalWrite(M2steppin,HIGH);
    delay(StepRate); 
  }
    
  
}
