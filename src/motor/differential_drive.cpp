#include "differential_drive.h"
#include <math.h>

DifferentialDrive::DifferentialDrive(float wheel_radius, float wheel_base)
    : _r(wheel_radius), _wheel_base(wheel_base) {}

void DifferentialDrive::setVelocity(float v, float omega,
                                     float& out_left_rad_s,
                                     float& out_right_rad_s) {
    // v     = (vR + vL) / 2
    // omega = (vR - vL) / _wheel_base
    // => vL = v - omega*_wheel_base/2,  vR = v + omega*_wheel_base/2
    float vL = v - omega * _wheel_base * 0.5f;
    float vR = v + omega * _wheel_base * 0.5f;
    out_left_rad_s  = vL / _r;
    out_right_rad_s = vR / _r;
}

void DifferentialDrive::updateOdometry(float left_rad_s,
                                        float right_rad_s,
                                        float dt) {
    float vL = left_rad_s  * _r;
    float vR = right_rad_s * _r;

    float v     = (vR + vL) * 0.5f;
    float omega = (vR - vL) / _wheel_base;

    // Integrate pose using midpoint method
    float dtheta = omega * dt;
    float mid    = _theta + dtheta * 0.5f;
    _x     += v * cosf(mid) * dt;
    _y     += v * sinf(mid) * dt;
    _theta  = wrapAngle(_theta + dtheta);
}

float DifferentialDrive::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}
