#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>

#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Set frequency. All modules must match.
#define RF95_FREQ 915.0

#define RF95_NAME "BASE STATION"

#define LED 13

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// make a struct for the messages?
// and maybe make some functions to create / decode them?

void setup() {
  delay(1000);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Ensure serial communication is active
  while (!Serial);
  Serial.begin(9600);
  delay(100);

  Serial.println("\nInitializing the " RF95_NAME " module");

  // Manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.print(RF95_FREQ); Serial.println(" MHz");

  // Can set transmitter power from 5 to 23 dBm
  rf95.setTxPower(23, false);

  Serial.println("-------------------");
}

// I think the Serial buffer can hold a maximum of 64 bytes?

void loop() {
   if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n'); // Read input from Serial until newline
      Serial.print("Received from Serial: ");
      Serial.println(input);

      // Convert the input string to a char array
      char radiopacket[input.length() + 1];
      input.toCharArray(radiopacket, input.length() + 1);

      // Send the input over LoRa
      rf95.send((uint8_t *)radiopacket, input.length() + 1); // Include null terminator

      Serial.println("Waiting for packet to complete...");
      rf95.waitPacketSent();
      Serial.println("Packet send successfully.");
   }
}