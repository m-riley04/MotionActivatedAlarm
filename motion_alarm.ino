#include <SoftwareSerial.h>
#include "TFMini.h"

// Initialize pin names
#define PIN_TFMINI_RX 11
#define PIN_TFMINI_TX 10
#define PIN_BUZZER 13

// Initialize global objects
SoftwareSerial mySerial(PIN_TFMINI_TX, PIN_TFMINI_RX);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

// Initialize global vars
uint16_t minDistance = 0;
uint16_t distanceBuffer = 5;

void countdown_beeps(uint8_t buzzer_pin, int beeps, unsigned int frequency = 1000, unsigned long beepDuration = 100, unsigned long beepGapDuration = 1000);
uint16_t read_lidar(TFMini &tfmini, unsigned long readDelay = 50);

void setup() {
  // Initialize board pin modes
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Initialize the serial
  Serial.begin(9600);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only

  // Send message to serial
  Serial.println ("Initializing...");
  mySerial.begin(TFMINI_BAUDRATE);

  // Initialize the TF Mini sensor
  tfmini.begin(&mySerial);
}

void loop() {
  // put your main code here, to run repeatedly:

}
