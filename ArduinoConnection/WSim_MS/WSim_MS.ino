// If something doesn't work use a 10k pull-up resistor
// GROUND BOTH ARDUINO's TOGETHER
// CONNECT BOTH ANALOG PIN A4 AND ANALOG PIN A5

#include <Wire.h>

// Define Slave I2C address and Slave Answer Size
#define SLAVE_ADDR 9
#define ANSWER_SIZE 5

void setup() {  
  Wire.begin();

  Serial.begin(9600);
  Serial.println("I2C Testing on Arduino");
}

void loop() {
    delay(50); //
    Serial.println("Write Data to Slave");

    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(0);
    Wire.endTransmission();

    // Read Response from Slave
    // Read back 5 characters
    // Don't need right now since we are only using 1 way transmission
    Wire.requestFrom(SLAVE_ADDR, ANSWER_SIZE);

    // Add characters to string

    String response = "";
    while(Wire.available()){
      char b = Wire.read();
      response += b;
    }

    Serial.println(response);
}
