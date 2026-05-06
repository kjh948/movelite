#pragma once
#include <SimpleFOC.h>
#include "../config.h"

// ─────────────────────────────────────────────
//  FOCMotor — thin wrapper around SimpleFOC
//  for MKS-ESP32-FOC V1.0 (voltage mode, no current sensing)
// ─────────────────────────────────────────────
class SimpleFOCWrapper {
public:
    // pole_pairs: number of motor pole pairs (default 7 for YT2804)
    SimpleFOCWrapper(int uh, int vh, int wh, int en,
                     TwoWire* i2c_bus, int pole_pairs = 7);

    // Call once in setup()
    // supply_voltage: actual power supply voltage in volts
    bool begin(float supply_voltage = 12.0f);

    // Call as fast as possible in real-time loop (Core 1)
    void loopFOC();

    // Set target velocity in rad/s
    void setVelocity(float rad_per_sec);

    // Get current velocity from encoder (rad/s)
    float getVelocity();

    // Get cumulative angle (radians)
    float getAngle();

private:
    BLDCMotor          _motor;
    BLDCDriver3PWM     _driver;
    MagneticSensorI2C  _sensor;
    TwoWire*           _i2c;
};
