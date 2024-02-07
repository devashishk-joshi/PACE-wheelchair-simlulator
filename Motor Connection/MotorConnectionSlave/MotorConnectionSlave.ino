#include <Wire.h>

// Define Slave I2C address and Slave Answer Size
#define SLAVE_ADDR 9
#define ANSWER_SIZE 5

int M1dirpin = 7; //Motor X direction pin
int M1steppin = 6; //Motor X step pin
int M1en = 8; //Motor X enable pin
int M2dirpin = 4; //Motor Y direction pin
int M2steppin = 5; //Motor Y step pin
int M2en = 12; //Motor Y enable pin

int StepRate = 5; //Control Rotational Speed

byte receivedValue = 0; // Variable to store the received value

int direction;
int velocity;
int actualStepRate;

void setup() {
  pinMode(M1dirpin, OUTPUT);
  pinMode(M1steppin, OUTPUT);
  pinMode(M1en, OUTPUT);
  pinMode(M2dirpin, OUTPUT);
  pinMode(M2steppin, OUTPUT);
  pinMode(M2en, OUTPUT);
  digitalWrite(M1en, LOW); // Low Level Enable
  digitalWrite(M2en, LOW); // Low Level 

  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);
  Serial.begin(9600);
  Serial.println("I2C Motor Connection");
}

void receiveEvent(int byteCount) {
  if (byteCount >= 1) {
    byte dataReceived = Wire.read();
    
    // Extract direction and velocity from the received byte
    direction = (dataReceived & 0x80) == 0 ? 1 : -1; // Update the global direction variable
    velocity = dataReceived & 0x0F; // Bits 0-6 represent velocity
    
    actualStepRate = 8 - velocity;

    // Print received data
    Serial.print("Received data from Master: Direction=");
    Serial.print(direction);
    Serial.print(", Velocity=");
    Serial.println(velocity);
  }
}


void loop() {
  delay(50); 

  for(int j = 0; j <= 1; j++) {    
    if (direction < 0) {
      digitalWrite(M1dirpin, HIGH);
      digitalWrite(M2dirpin, HIGH);
    } else {
      digitalWrite(M1dirpin, LOW);
      digitalWrite(M2dirpin, LOW);
    }
    
    for (int i = 0; i <= 250; i++) {
      digitalWrite(M1steppin, LOW);
      digitalWrite(M2steppin, LOW);
      delayMicroseconds(2);
      digitalWrite(M1steppin, HIGH); // Rising step
      digitalWrite(M2steppin, HIGH);
      delay(actualStepRate); 
    }
  }
  
  // Reset the received value after processing
  receivedValue = 0;
}
