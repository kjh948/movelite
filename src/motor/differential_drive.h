#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────
//  DifferentialDrive
//  Converts (v, omega) ↔ (left_rad_s, right_rad_s)
//  and maintains odometry pose (x, y, theta)
// ─────────────────────────────────────────────
class DifferentialDrive {
public:
    // wheel_radius: meters
    // wheel_base  : distance between left and right wheels, meters
    DifferentialDrive(float wheel_radius, float wheel_base);

    // Set target linear + angular velocity
    // v     : m/s  (positive = forward)
    // omega : rad/s (positive = counter-clockwise / left turn)
    void setVelocity(float v, float omega,
                     float& out_left_rad_s, float& out_right_rad_s);

    // Update odometry; call at fixed rate (dt seconds)
    // left_rad_s / right_rad_s: measured wheel velocities
    void updateOdometry(float left_rad_s, float right_rad_s, float dt);

    float getX()     const { return _x; }
    float getY()     const { return _y; }
    float getTheta() const { return _theta; }

    // Override heading from IMU fusion
    void setTheta(float theta) { _theta = theta; }

    void resetPose(float x = 0, float y = 0, float theta = 0) {
        _x = x; _y = y; _theta = theta;
    }

private:
    float _r;     // wheel radius
    float _wheel_base;     // wheel base
    float _x     = 0.0f;
    float _y     = 0.0f;
    float _theta = 0.0f;

    // Normalize angle to [-π, π]
    static float wrapAngle(float a);
};
