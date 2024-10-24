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

  if (Serial.read() == 'y') {
    Serial.println("got it, lol");
  }

  // Serial.println("Sending a packet from 1");
  // what I want a packaet to look like
  // [destination, origin, clarifier, message]
  // could define a struct to do this...

  Serial.println("Sending a message!");

  // constantly just send out a message....
  uint8_t radiopacket[4] = {2,1,0,7};
  rf95.send((uint8_t *)radiopacket, 4); // 4 bytes. max is 255, I believe
  rf95.waitPacketSent(); // wait for the transmitter to become available

  // // Now, wait for a reply
  // // uint

  delay(1000);

  // uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  // uint8_t len = sizeof(buf);

  // if (rf95.recv(buf, &len)) {
  //   digitalWrite(LED, HIGH);
  //   RH_RF95::printBuffer("Received: ", buf, len);
  //   Serial.println((char*)buf);
    
  // }
  // else {
  //   Serial.println("No message received.");
  // }

  



  
}