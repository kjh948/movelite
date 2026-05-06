#include "imu.h"
#include <math.h>

// MPU6050 register map
#define MPU_PWR_MGMT_1   0x6B
#define MPU_CONFIG       0x1A
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_XOUT_H  0x43

// Gyro ±250°/s → 131 LSB/(°/s) → convert to rad/s
static constexpr float GYRO_SCALE = (1.0f / 131.0f) * (M_PI / 180.0f);
// Accel ±2g → 16384 LSB/g
static constexpr float ACCEL_SCALE = 1.0f / 16384.0f;

IMU::IMU(TwoWire* wire, uint8_t i2c_addr)
    : _wire(wire), _addr(i2c_addr) {}

bool IMU::begin() {
    // Wake up device
    writeReg(MPU_PWR_MGMT_1, 0x00);
    delay(100);

    // DLPF: ~44Hz bandwidth (reduces noise)
    writeReg(MPU_CONFIG, 0x03);

    // Gyro: ±250°/s
    writeReg(MPU_GYRO_CONFIG, 0x00);

    // Accel: ±2g
    writeReg(MPU_ACCEL_CONFIG, 0x00);

    // Verify device is alive
    uint8_t who = readReg(0x75);
    _initialized = (who == 0x68);
    return _initialized;
}

float IMU::update(float dt) {
    if (!_initialized) return _yaw;

    int16_t ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);

    // Gyro Z in rad/s
    _gz_rad = (float)gz * GYRO_SCALE;

    // Gyro integration
    float gyro_yaw = _yaw + _gz_rad * dt;

    // Accel heading (works only when robot is roughly flat)
    float ax_f = (float)ax * ACCEL_SCALE;
    float ay_f = (float)ay * ACCEL_SCALE;
    float accel_heading = atan2f(ay_f, ax_f);

    // Complementary filter
    // Note: for 2D floor robot, accel-based heading is weak —
    // set ALPHA high (0.98+) and rely primarily on gyro.
    _yaw = wrapAngle(ALPHA * gyro_yaw + (1.0f - ALPHA) * accel_heading);
    return _yaw;
}

// ─── Private helpers ──────────────────────────────────────────────────────────

void IMU::writeReg(uint8_t reg, uint8_t val) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

uint8_t IMU::readReg(uint8_t reg) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)1);
    return _wire->available() ? _wire->read() : 0xFF;
}

void IMU::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                  int16_t& gx, int16_t& gy, int16_t& gz) {
    _wire->beginTransmission(_addr);
    _wire->write(MPU_ACCEL_XOUT_H);
    _wire->endTransmission(false);
    _wire->requestFrom(_addr, (uint8_t)14);

    auto read16 = [this]() -> int16_t {
        return (int16_t)((_wire->read() << 8) | _wire->read());
    };
    ax = read16(); ay = read16(); az = read16();
    read16();   // temperature — discard
    gx = read16(); gy = read16(); gz = read16();
}

float IMU::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}
