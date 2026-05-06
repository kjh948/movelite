#pragma once
#include <Wire.h>

// ─────────────────────────────────────────────
//  IMU — MPU6050 Yaw complementary filter
//  Uses gyro Z integration + accel heading fallback
//  (full 6-axis fusion not needed for 2D navigation)
// ─────────────────────────────────────────────
class IMU {
public:
    // i2c_addr: 0x68 (AD0=GND) or 0x69 (AD0=VCC)
    IMU(TwoWire* wire, uint8_t i2c_addr = 0x68);

    bool begin();

    // Call at fixed rate; dt = elapsed seconds since last call
    // Returns complementary-filtered yaw in radians [-π, π]
    float update(float dt);

    float getYaw()   const { return _yaw; }
    float getGyroZ() const { return _gz_rad; }  // raw gyro Z (rad/s)

    void resetYaw(float yaw = 0.0f) { _yaw = yaw; }

private:
    TwoWire* _wire;
    uint8_t  _addr;
    float    _yaw     = 0.0f;
    float    _gz_rad  = 0.0f;
    bool     _initialized = false;

    // Complementary filter coefficient
    // 0.98 = trust gyro heavily (MPU6050 2D case — no mag)
    static constexpr float ALPHA = 0.98f;

    void     writeReg(uint8_t reg, uint8_t val);
    uint8_t  readReg(uint8_t reg);
    void     readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                     int16_t& gx, int16_t& gy, int16_t& gz);

    static float wrapAngle(float a);
};
