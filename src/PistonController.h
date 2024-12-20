#ifndef PISTON_CONTROLLER_H
#define PISTON_CONTROLLER_H

#include <Arduino.h>
#include <esp_dmx.h>

// Debug configuration
#ifndef PISTON_DEBUG
#define PISTON_DEBUG 1  // Set to 0 to disable debug prints
#endif

#if PISTON_DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTF(x, ...)
#endif

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
        int dmxPinEn,
        bool enableDebug = true
    );

    // Initialize the controller
    void begin();
    
    // Set target position in encoder counts
    void setTargetPosition(long position);
    
    // Get current position in encoder counts
    long getCurrentPosition() const;
    
    // Update control loop (call this in main loop)
    void update();

    void checkDmxSignal();

    void dmxSwitch();

    void dmxLinear();
    
    // Set PID control parameters
    void setPIDParameters(float kp, float ki, float kd);
    
    // Emergency stop
    void emergencyStop();
    
    // Check if target position is reached (within tolerance)
    bool isTargetReached() const;
    
    // Calibrate the position (set current position as zero)
    void calibrate();

    // Jogging functions
    void jogExtend();
    void jogRetract();
    void stopJog();
    bool isJogging() const { return _isJogging; }

    // Find home position
    bool findHome(unsigned long timeout = 30000);

    // Enable/disable debug prints at runtime
    void setDebug(bool enable) { _debugEnabled = enable; }

private:
    // Debug flag
    bool _debugEnabled;
    
    // Pins
    const int _valvePin1;
    const int _valvePin2;
    const int _encoderPinA;
    const int _encoderPinB;
    const int _dmxPinRx;
    const int _dmxPinTx;
    const int _dmxPinEn;

    // Dmx variables
    byte _data[DMX_PACKET_SIZE];
    dmx_port_t _dmxPort = 1;
    
    // Position variables
    volatile long _currentPosition;
    long _targetPosition;
    const int _positionTolerance = 10; // encoder counts
    volatile long _homeExtend;
    volatile long _homeRetract;
    volatile long _travelLength;
    
    // Jogging variables
    bool _isJogging = false;

    // Control variables
    float _kp = 1.0;
    float _ki = 0.0;
    float _kd = 0.0;
    float _integral = 0.0;
    float _lastError = 0.0;
    unsigned long _lastTime = 0;
    
    // Valve control states
    enum ValveState {
        EXTEND,
        RETRACT,
        HOLD
    };
    ValveState _currentState;
    
    // Private methods
    void updateEncoder();
    void setValveState(ValveState state);
    float calculatePID(float error);
    
    void debugPrint(const char* msg) const { // Added const here
        #if PISTON_DEBUG
        if (_debugEnabled) {
            Serial.println(msg);
        }
        #endif
    }
    
    template<typename T>
    void debugPrint(const char* msg, T value) const {  // Added const here
        #if PISTON_DEBUG
        if (_debugEnabled) {
            Serial.print(msg);
            Serial.println(value);
        }
        #endif
    }
    
    // ISR friend function for encoder
    friend void IRAM_ATTR encoderISR(void* arg);
};

#endif
