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
uint16_t distanceBuffer = 10;

void countdown_beeps(uint8_t buzzer_pin, int beeps, unsigned int frequency = 1000, unsigned long beepDuration = 100, unsigned long beepGapDuration = 1000);
uint16_t read_lidar(TFMini &tfmini, unsigned long readDelay = 50);
void check_distance(uint16_t distance, uint16_t distanceThreshold, uint16_t buffer = 5, int buzzerPin = 9, int ledPin = LED_BUILTIN);

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

  // Get the initial/max distance
  countdown_beeps(PIN_BUZZER, 3); // Tell the user that it is reading the distance
  while (minDistance == 0 || minDistance > 1000) {
    minDistance = read_lidar(tfmini);
  }

}

void loop() {
  // Get the distance
  uint16_t distance = read_lidar(tfmini);

  // Check if something crosses the beam
  check_distance(distance, minDistance, distanceBuffer, PIN_BUZZER, LED_BUILTIN);

  // Print data to serial
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm      Min Distance: ");
  Serial.print(minDistance);
  Serial.println(" cm");

  // Delay to wait for catchups
  delay(25);
}

void check_distance(uint16_t distance, uint16_t distanceThreshold, uint16_t buffer = 5, int buzzerPin = 9, int ledPin = LED_BUILTIN) {
  if ((distance + buffer) < distanceThreshold) {
    digitalWrite(ledPin, HIGH); // Turn on the on-board LED
    tone(buzzerPin, 1000); // Turn on buzzer
  } else {
    digitalWrite(ledPin, LOW); // Turn off the LED
    noTone(buzzerPin); // Turn off the tone
  }
}

void countdown_beeps(uint8_t buzzerPin, int beeps, unsigned int frequency = 1000, unsigned long beepDuration = 100, unsigned long beepGapDuration = 1000) {
  for (int i = 0; i < beeps; i++) {
    tone(buzzerPin, frequency);
    delay(beepDuration);
    noTone(PIN_BUZZER);
    delay(beepGapDuration);
  }
}

uint16_t read_lidar(TFMini &tfmini, unsigned long readDelay = 50) {
  uint16_t d = tfmini.getDistance();
  delay(readDelay);
  return d;
}
