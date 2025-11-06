#ifndef PISTON_CONTROLLER_H
#define PISTON_CONTROLLER_H

// Debug configuration
#ifndef PISTON_DEBUG
#define PISTON_DEBUG 1  // Set to 0 to disable debug prints
#endif

#if PISTON_DEBUG
    #define debugPrintf(...) Serial.printf(__VA_ARGS__)
#else
    #define debugPrintf(...)
#endif

#define DMX_CHAN_EXTEND 0
#define DMX_CHAN_RETRACT 1
#define DMX_CHAN_POS_MSB 2
#define DMX_CHAN_POS_LSB 3
#define DMX_CHAN_CTRL 4

#define TIMEOUT_HOME_START 3000
#define MIN_TIME_HOME_EXTEND 60000
#define TIMEOUT_HOME_EXTEND 600000
#define MIN_TIME_HOME_RETRACT 45000
#define TIMEOUT_HOME_RETRACT 600000

#define MIN_DISTANCE_HOMING 200
#define THRESHOLD_STABLE 25
#define TIMEOUT_STABLE 5000

#define VALVE_DUTY_THRESHOLD 40

enum ValveState {
    EXTEND,
    RETRACT,
    HOLD,
    INVALID
};

enum PistonControllerState {
    PISTON_CONTROLLER_INIT,
    PISTON_CONTROLLER_HOME_START,
    PISTON_CONTROLLER_HOME_EXTEND,
    PISTON_CONTROLLER_HOME_RETRACT,
    PISTON_CONTROLLER_HOMED,
    PISTON_CONTROLLER_REHOME_START,
    PISTON_CONTROLLER_INVALID
};

class PistonController {
public:
    // Constructor
    PistonController(
        int valvePin1,      // Pin for valve direction 1
        int valvePin2,      // Pin for valve direction 2
        int encoderPinA,    // Encoder pin A
        int encoderPinB,    // Encoder pin B
        int dmxPinRx,
        int dmxPinTx,
        int dmxPinEn
    );

    // Initialize the controller
    void setup();
    void loop();

    void tick();
private:
    // Pins
    const int _valvePin1;
    const int _valvePin2;
    const int _encoderPinA;
    const int _encoderPinB;
    const int _dmxPinRx;
    const int _dmxPinTx;
    const int _dmxPinEn;

    // State
    enum PistonControllerState _state;
    void enterState(enum PistonControllerState state);

    // DMX
    byte _dmxData[DMX_PACKET_SIZE];       // Last received DMX frame
    dmx_port_t _dmxPort;                  // Port id of the esp_dmx driver
    uint16_t _dmx_start_address;

    void readDmx();                       // ~Read a DMX frame if available
    void dmxLinear();                     // Set position from DMX

    // TIMER pwm
    hw_timer_t *_timer = NULL;
    friend void IRAM_ATTR timerISR(void* arg);
    uint8_t _dutyCycle1;
    uint8_t _dutyCycle2;

    // Piston Valve
    ValveState _currentValveState;        // Stores the current valve state
    void setValveState(ValveState state); // Update the current valve state

    // Position variables
    volatile long _currentPosition;       // Current position of encoder, updated by ISR
    long _targetPosition;                 // TargetPosition
    void setTargetPosition(long position);
    const int _positionTolerance = 0;     // Encoder Counts

    long _homeExtended;                   // Used during homing
    long _homeRetracted;                  // Used during homing
    unsigned long _homeStable;            // Used during homing to check if stable
                                          // position is reached

    volatile long _travelLength;          // Set after homing

    // Update PID control loop
    void runPID();
    void runSimple();
    void resetPID();

    // Jogging variables
    bool _isJogging;

    // Control variables
    float _kp = 0.12;
    float _ki = 0.00;
    float _kd = 0.00;
    float _integral;
    float _lastError;
    unsigned long _lastTime;
    unsigned long _lastStateChange;
    // Private methods
    void updateEncoder();

    float calculatePID(float error);

    // ISR friend function for encoder
    friend void IRAM_ATTR encoderISR(void* arg);
};

#endif
