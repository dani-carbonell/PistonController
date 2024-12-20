// CompleteExample.ino
#include <PistonController.h>
#include <esp_dmx.h>
#include <Arduino.h>

// Pin definitions
const int VALVE_PIN_1 = 25;    // ESP32 GPIO pin for valve direction 1
const int VALVE_PIN_2 = 26;    // ESP32 GPIO pin for valve direction 2
const int ENCODER_PIN_A = 32;  // ESP32 GPIO pin for encoder A
const int ENCODER_PIN_B = 33;  // ESP32 GPIO pin for encoder B
const int ENCODER_PIN_Z = 34;  // ESP32 GPIO pin for encoder Z (home)
const int DMX_RX = 16;
const int DMX_TX = 17;
const int DMX_EN = 21;


// Button pins for manual control
const int BUTTON_HOME = 14;    // Home search button
const int BUTTON_EXTEND = 12;  // Extend button
const int BUTTON_RETRACT = 13; // Retract button
const int BUTTON_STOP = 27;    // Emergency stop button

// Movement parameters
const long POSITION_1 = 1000;  // Extended position
const long POSITION_2 = 0;     // Retracted position
const long HOME_TIMEOUT = 30000; // Home search timeout in milliseconds

// Create controller instance
PistonController piston(VALVE_PIN_1, VALVE_PIN_2, ENCODER_PIN_A, ENCODER_PIN_B, ENCODER_PIN_Z, DMX_RX, DMX_TX, DMX_EN);

// System state
enum SystemState {
    STATE_INIT,
    STATE_HOMING,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_SWITCH
} currentState = STATE_SWITCH;

// Movement mode
enum MovementMode {
    MODE_DMX,
    MODE_MANUAL,
    MODE_AUTO
} movementMode = MODE_MANUAL;

// Timer variables
unsigned long lastStatusPrint = 0;
const int STATUS_INTERVAL = 500;  // Status print interval in milliseconds

// Auto mode variables
unsigned long lastPositionChange = 0;
const unsigned long POSITION_CHANGE_INTERVAL = 5000;  // Time between automatic movements
bool movingToPosition1 = true;

void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial && millis() < 5000);  // Wait for serial connection
    
    // Setup button pins
    pinMode(BUTTON_HOME, INPUT_PULLUP);
    pinMode(BUTTON_EXTEND, INPUT_PULLUP);
    pinMode(BUTTON_RETRACT, INPUT_PULLUP);
    pinMode(BUTTON_STOP, INPUT_PULLUP);
    
    // Initialize the controller with optimized PID values
    piston.begin();
    piston.setPIDParameters(1.2, 0.1, 0.05);  // Adjust these values for your system
    
    Serial.println("System initialized. Press Home button to start homing sequence.");
    printHelp();
}

void loop() {
    // Check for serial commands
    handleSerialCommands();
    
    // Check emergency stop button
    if (digitalRead(BUTTON_STOP) == LOW) {
        handleEmergencyStop();
        return;
    }
    
    // State machine
    switch (currentState) {
        case STATE_INIT:
            handleInitState();
            break;
            
        case STATE_HOMING:
            handleHomingState();
            break;
            
        case STATE_IDLE:
            handleIdleState();
            break;
            
        case STATE_RUNNING:
            handleRunningState();
            break;
            
        case STATE_ERROR:
            handleErrorState();
            break;

        case STATE_SWITCH:
            handleSwitchState();
            break;
    }
    
    // Update status display periodically
    if (millis() - lastStatusPrint >= STATUS_INTERVAL) {
        printStatus();
        lastStatusPrint = millis();
    }
}

void handleInitState() {
    if (digitalRead(BUTTON_HOME) == LOW) {
        Serial.println("Starting homing sequence...");
        currentState = STATE_HOMING;
        delay(200);  // Simple debounce
    }
}

void handleHomingState() {
    static bool homingStarted = false;
    
    if (!homingStarted) {
        homingStarted = true;
        if (piston.findHome(HOME_TIMEOUT)) {
            Serial.println("Homing successful!");
            currentState = STATE_IDLE;
        } else {
            Serial.println("Homing failed!");
            currentState = STATE_ERROR;
        }
        homingStarted = false;
    }
}

void handleIdleState() {
    if (movementMode == MODE_MANUAL) {
        // Manual control using buttons
        if (digitalRead(BUTTON_EXTEND) == LOW) {
            piston.setTargetPosition(POSITION_1);
            currentState = STATE_RUNNING;
        } else if (digitalRead(BUTTON_RETRACT) == LOW) {
            piston.setTargetPosition(POSITION_2);
            currentState = STATE_RUNNING;
        }
    } else {
        // Start automatic movement
        currentState = STATE_RUNNING;
        lastPositionChange = millis();
    }
}

void handleRunningState() {
    piston.update();  // Update position control
    
    if (movementMode == MODE_AUTO) {
        // Automatic back-and-forth movement
        if (millis() - lastPositionChange >= POSITION_CHANGE_INTERVAL) {
            if (piston.isTargetReached()) {
                if (movingToPosition1) {
                    piston.setTargetPosition(POSITION_2);
                } else {
                    piston.setTargetPosition(POSITION_1);
                }
                movingToPosition1 = !movingToPosition1;
                lastPositionChange = millis();
            }
        }
    } else {
        // Manual mode - return to idle when target reached
        if (piston.isTargetReached()) {
            currentState = STATE_IDLE;
        }
    }
}

void handleErrorState() {
    static unsigned long lastBlink = 0;
    const int BLINK_INTERVAL = 500;
    
    // Blink or indicate error state
    if (millis() - lastBlink >= BLINK_INTERVAL) {
        Serial.println("System in ERROR state. Press Home button to restart.");
        lastBlink = millis();
    }
    
    // Allow restart by homing
    if (digitalRead(BUTTON_HOME) == LOW) {
        currentState = STATE_INIT;
        delay(200);  // Simple debounce
    }
}

void handleEmergencyStop() {
    piston.emergencyStop();
    currentState = STATE_ERROR;
    Serial.println("EMERGENCY STOP ACTIVATED!");
}

void handleSerialCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'h':  // Home
                currentState = STATE_HOMING;
                break;
            case 'a':  // Toggle auto mode
                movementMode = (movementMode == MODE_AUTO) ? MODE_MANUAL : MODE_AUTO;
                Serial.printf("Mode changed to: %s\n", 
                            (movementMode == MODE_AUTO) ? "AUTO" : "MANUAL");
                break;
            case 's':  // Stop
                piston.emergencyStop();
                currentState = STATE_IDLE;
                break;
            case '?':  // Help
                printHelp();
                break;
        }
    }
}

void handleSwitchState() {
    piston.checkDmxSignal();
    piston.dmxSwitch();
}

void printStatus() {
    Serial.printf("Position: %ld, State: %s, Mode: %s\n",
                 piston.getCurrentPosition(),
                 getStateString(currentState),
                 (movementMode == MODE_AUTO) ? "AUTO" : "MANUAL");
}

void printHelp() {
    Serial.println("\nAvailable commands:");
    Serial.println("h - Start homing sequence");
    Serial.println("a - Toggle auto/manual mode");
    Serial.println("s - Stop movement");
    Serial.println("? - Show this help");
    Serial.println("\nButtons:");
    Serial.println("HOME - Start homing sequence");
    Serial.println("EXTEND - Move to extended position");
    Serial.println("RETRACT - Move to retracted position");
    Serial.println("STOP - Emergency stop");
}

const char* getStateString(SystemState state) {
    switch (state) {
        case STATE_INIT: return "INIT";
        case STATE_HOMING: return "HOMING";
        case STATE_IDLE: return "IDLE";
        case STATE_RUNNING: return "RUNNING";
        case STATE_ERROR: return "ERROR";
        case STATE_SWITCH: return "SWITCH";
        default: return "UNKNOWN";
    }
}
