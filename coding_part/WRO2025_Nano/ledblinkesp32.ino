/*
 * ESP32 Built-in LED Blink Example
 * Blinks the on-board LED at a 1-second pulse
 */

// The built-in LED on the DevKit V1 is connected to GPIO 2.
// You can also use the predefined constant LED_BUILTIN.
#define LED_PIN 2 

// The setup function runs once when you press reset or power the board
void setup() {
  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

// The loop function runs over and over again forever
void loop() {
  // Turn the LED on (HIGH is the voltage level)
  digitalWrite(LED_PIN, HIGH);
  // Wait for 1000 milliseconds (1 second)
  delay(1000);
  
  // Turn the LED off by making the voltage LOW
  digitalWrite(LED_PIN, LOW);
  // Wait for another 1000 milliseconds
  delay(1000);
}
