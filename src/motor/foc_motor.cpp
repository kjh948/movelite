#include "foc_motor.h"

SimpleFOCWrapper::SimpleFOCWrapper(int uh, int vh, int wh, int en,
                   TwoWire* i2c_bus, int pole_pairs)
    : _motor(pole_pairs),
      _driver(uh, vh, wh, en),
      _sensor(AS5600_I2C),
      _i2c(i2c_bus)
{}

bool SimpleFOCWrapper::begin(float supply_voltage) {
    // ── Encoder ──────────────────────────────
    _sensor.init(_i2c);
    _motor.linkSensor(&_sensor);

    // ── Driver ───────────────────────────────
    _driver.voltage_power_supply = supply_voltage;
    if (!_driver.init()) return false;
    _motor.linkDriver(&_driver);

    // ── Control mode: velocity (voltage torque) ──
    _motor.foc_modulation  = FOCModulationType::SpaceVectorPWM;
    _motor.controller      = MotionControlType::velocity;
    _motor.torque_controller = TorqueControlType::voltage;

    // ── Voltage limits ───────────────────────
    // Keep conservative for safety; tune to actual supply
    _motor.voltage_limit   = supply_voltage * 0.5f;
    _motor.velocity_limit  = 40.0f;  // rad/s max

    // ── Velocity PID (tuned for YT2804 / adjust for your motor) ──
    _motor.PID_velocity.P  = 0.10f;
    _motor.PID_velocity.I  = 1.00f;
    _motor.PID_velocity.D  = 0.00f;
    _motor.LPF_velocity.Tf = 0.01f;

    // ── Initialize ───────────────────────────
    _motor.init();
    _motor.initFOC();
    _motor.target = 0.0f;
    return true;
}

void SimpleFOCWrapper::loopFOC() {
    _motor.loopFOC();
    _motor.move();
}

void SimpleFOCWrapper::setVelocity(float rad_per_sec) {
    _motor.target = rad_per_sec;
}

float SimpleFOCWrapper::getVelocity() {
    return _motor.shaft_velocity;
}

float SimpleFOCWrapper::getAngle() {
    return _motor.shaft_angle;
}
