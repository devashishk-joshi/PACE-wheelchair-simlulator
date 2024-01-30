#include <Wire.h>

// Define Slave I2C address and Slave Answer Size
#define SLAVE_ADDR 9
#define ANSWER_SIZE 5

String answer = "Hello";

void setup() {
  // put your setup code here, to run once:
  Wire.begin(SLAVE_ADDR);

  // Wire.on Request(requestEvent); // 1 Way connection ONLY
  Wire.onReceive(receiveEvent);

  Serial.begin(9600);
  Serial.println("I2C Slave Connection Test");
}

void receiveEvent(){
  // Read while data received
  while (Wire.available() > 0){
    byte receivedByte = Wire.read();
    Serial.print("Received byte: ");

    if (receivedByte == 0){
        Serial.println("Velocity: 0 m/s");
    }
    else if (receivedByte == 1){
        Serial.println("Velocity: 1 m/s");
    }
    else if (receivedByte == 2){
        Serial.println("Velocity: 2 m/s");
    }
    else if (receivedByte == 3){
        Serial.println("Velocity: 3 m/s");
    }
    else if (receivedByte == 4){
        Serial.println("Velocity: 3 m/s");
    }
    else if (receivedByte == 5){
        Serial.println("Velocity: 4 m/s");
    }
  }
}

void requestEvent(){
  // Set up byte variable in the correct size
  byte response[ANSWER_SIZE];

  for(byte i=0; i<ANSWER_SIZE; i++){
    response[i] = (byte)answer.charAt(i);
  }

  // Send response back to Master
  Wire.write(response, sizeof(response));

  // Print to Serial Monitor
  Serial.println("Request Event");
}


void loop() {
  delay(50);
}
