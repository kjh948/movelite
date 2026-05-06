#pragma once
#include <Wire.h>
#include <VL53L0X.h>

// ─────────────────────────────────────────────
//  ToF — dual VL53L0X (front + rear)
//  Addresses reassigned at boot via XSHUT pins
// ─────────────────────────────────────────────
class ToFSensors {
public:
    // xshut_front / xshut_rear: GPIO pins for XSHUT lines
    // addr_front / addr_rear  : I2C addresses after reassignment
    ToFSensors(TwoWire* wire,
               int xshut_front, int xshut_rear,
               uint8_t addr_front = TOF_FRONT_ADDR,
               uint8_t addr_rear  = TOF_REAR_ADDR);

    // Returns false if either sensor not found
    bool begin();

    // Read both sensors; fills out distances in meters
    // Returns false on read error; on error keeps last valid reading
    bool read(float& front_m, float& rear_m);

    float getFront() const { return _front_m; }
    float getRear()  const { return _rear_m; }

    // Maximum valid range (meters); readings above treated as "no obstacle"
    static constexpr float MAX_RANGE = 1.2f;

private:
    TwoWire*  _wire;
    int       _xshut_front, _xshut_rear;
    uint8_t   _addr_front, _addr_rear;
    VL53L0X   _sensor_front, _sensor_rear;
    float     _front_m = MAX_RANGE;
    float     _rear_m  = MAX_RANGE;

    bool initSensor(VL53L0X& sensor, uint8_t new_addr);
};
