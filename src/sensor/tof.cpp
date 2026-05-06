#include "tof.h"

ToFSensors::ToFSensors(TwoWire* wire,
                       int xshut_front, int xshut_rear,
                       uint8_t addr_front, uint8_t addr_rear)
    : _wire(wire),
      _xshut_front(xshut_front), _xshut_rear(xshut_rear),
      _addr_front(addr_front),   _addr_rear(addr_rear)
{}

bool ToFSensors::begin() {
    // Pull both XSHUT LOW → reset both sensors to 0x29
    pinMode(_xshut_front, OUTPUT);
    pinMode(_xshut_rear,  OUTPUT);
    digitalWrite(_xshut_front, LOW);
    digitalWrite(_xshut_rear,  LOW);
    delay(10);

    // Bring up front sensor alone, reassign its address
    digitalWrite(_xshut_front, HIGH);
    delay(10);
    if (!initSensor(_sensor_front, _addr_front)) return false;

    // Bring up rear sensor, reassign its address
    digitalWrite(_xshut_rear, HIGH);
    delay(10);
    if (!initSensor(_sensor_rear, _addr_rear)) return false;

    return true;
}

bool ToFSensors::initSensor(VL53L0X& sensor, uint8_t new_addr) {
    sensor.setBus(_wire);
    if (!sensor.init()) return false;
    sensor.setAddress(new_addr);
    // High-speed mode: ~33ms timing budget (30Hz capable)
    sensor.setMeasurementTimingBudget(33000);
    sensor.startContinuous();
    return true;
}

bool ToFSensors::read(float& front_m, float& rear_m) {
    uint16_t f_mm = _sensor_front.readRangeContinuousMillimeters();
    uint16_t r_mm = _sensor_rear .readRangeContinuousMillimeters();

    bool ok = true;
    if (_sensor_front.timeoutOccurred() || f_mm == 0) {
        ok = false;
    } else {
        _front_m = f_mm * 0.001f;
        if (_front_m > MAX_RANGE) _front_m = MAX_RANGE;
    }

    if (_sensor_rear.timeoutOccurred() || r_mm == 0) {
        ok = false;
    } else {
        _rear_m = r_mm * 0.001f;
        if (_rear_m > MAX_RANGE) _rear_m = MAX_RANGE;
    }

    front_m = _front_m;
    rear_m  = _rear_m;
    return ok;
}
