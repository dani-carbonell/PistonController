#include <Arduino.h>
#include <esp_dmx.h>

#include "PistonController.h"

// Pin definitions
const int VALVE_PIN_EXTEND  = 25;  // ESP32 GPIO pin for valve direction 1
const int VALVE_PIN_RETRACT = 26;  // ESP32 GPIO pin for valve direction 2
const int ENCODER_PIN_A = 32;      // ESP32 GPIO pin for encoder A
const int ENCODER_PIN_B = 33;      // ESP32 GPIO pin for encoder B
const int DMX_RX = 16;
const int DMX_TX = 17;
const int DMX_EN = 21;

// Create controller instance
PistonController piston(
    VALVE_PIN_EXTEND,
    VALVE_PIN_RETRACT,
    ENCODER_PIN_A,
    ENCODER_PIN_B,
    DMX_RX,
    DMX_TX,
    DMX_EN
);

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 5000);  // Wait for serial connection

    // Initialize the controller with optimized PID values
    piston.setup();
}

void loop() {
    piston.loop();
}
