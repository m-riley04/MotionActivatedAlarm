#include <SoftwareSerial.h>
#include "TFMini.h"

// Initialize pin names
#define PIN_TFMINI_RX 11
#define PIN_TFMINI_TX 10
#define PIN_BUZZER 13

// Initialize global objects
SoftwareSerial mySerial(PIN_TFMINI_TX, PIN_TFMINI_RX);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

void setup() {
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only

  // Send message to serial
  Serial.println ("Initializing...");

  // Initialize board pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT); // Set buzzer - pin 9 as an output

  mySerial.begin(TFMINI_BAUDRATE);

  //Initialize the TF Mini sensor
  tfmini.begin(&mySerial);
}

void loop() {
  // put your main code here, to run repeatedly:

}
