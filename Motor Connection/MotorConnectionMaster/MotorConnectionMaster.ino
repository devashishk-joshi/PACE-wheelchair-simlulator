#include <Wire.h>

#define SLAVE_ADDR 9

int maxVelocity = 5; // This variable can be changed using buttons. This basically limits the maximum speed that the simulator can go at.

void setup() {  
  Wire.begin();
  Serial.begin(9600);
  Serial.println("I2C Testing on Arduino");
}

void loop() {
  // Sweep through velocity levels for both directions
  for (int direction = -1; direction <= 1; direction += 2) { // direction = -1 for negative direction, 1 for positive direction
    for (int velocity = 0; velocity <= 5; velocity++) {
      // Pack direction and velocity into a single byte
      byte dataToSend = (direction == 1 ? 0x00 : 0x80) | velocity; // Direction in bit 7, velocity in bits 0-6
      
      // Send data to the slave
      Serial.print("Sending data to Slave: Direction=");
      Serial.print(direction);
      Serial.print(", Velocity=");
      Serial.println(velocity);

      Wire.beginTransmission(SLAVE_ADDR);
      Wire.write(dataToSend);
      Wire.endTransmission();
      
      // Wait for some time before sending the next data
      delay(1000);
    }
  }
}