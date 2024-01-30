#include <Wire.h>

// Define Slave I2C address and Slave Answer Size
#define SLAVE_ADDR 9
#define ANSWER_SIZE 5

int M1dirpin = 7; //Motor X direction pin
int M1steppin = 6; //Motor X step pin
int M1en=8; //Motor X enable pin
int M2dirpin = 4; //Motor Y direction pin
int M2steppin = 5; //Motor Y step pin
int M2en=12; //Motor Y enable pin

int Direction = 1; //Control Motor Direction
int StepRate = 3; //Control Rotational Speed

byte receivedValue = 0; // Variable to store the received value

void setup() {
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M1en,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
  pinMode(M2en,OUTPUT);
  digitalWrite(M1en,LOW);// Low Level Enable
  digitalWrite(M2en,LOW);// Low Level 

  Wire.begin(SLAVE_ADDR);
  // Wire.on Request(requestEvent); // 1 Way connection ONLY
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
  Serial.println("I2C Motor Connection");
}

void receiveEvent(){
  // Read while data received

  while (Wire.available() > 0) {
    receivedValue = Wire.read();
    Serial.println("Receive Event: " + String(receivedValue));
  }
}


void loop() {
  //delay(50); 
  if (receivedValue == 1) {
    int i;
    for (i = 0; i <= 1250; i++) {
      digitalWrite(M1steppin, LOW);
      digitalWrite(M2steppin, LOW);
      delayMicroseconds(2);
      digitalWrite(M1steppin, HIGH); // Rising step
      digitalWrite(M2steppin, HIGH);
      delayMicroseconds(StepRate);
    }

    // Reset the received value after processing
    receivedValue = 0;
  }
}
 
