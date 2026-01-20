/**
 * GNC-Airbrakes Firmware
 * Teensy 4.1 Entry Point
 */

#include <Arduino.h>

// Teensy 4.1 built-in LED pin
constexpr uint8_t LED_PIN = 13;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Configure LED pin as output
    pinMode(LED_PIN, OUTPUT);

    Serial.println("GNC-Airbrakes firmware initialized");
}

void loop() {
    // Simple LED blink to verify firmware is running
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}
