#include "hw_test.h"
#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include "config.h"
#include "sensor/imu.h"
#include "sensor/tof.h"

// ────────────────────────────────────────────────────────
// 1. Encoder Test
// ────────────────────────────────────────────────────────
void run_test_encoder() {
    Serial.println("\n[Test 1] Starting Dual AS5600 Encoder Test...");
    TwoWire I2C0 = TwoWire(0);
    TwoWire I2C1 = TwoWire(1);
    MagneticSensorI2C sensorL = MagneticSensorI2C(AS5600_I2C);
    MagneticSensorI2C sensorR = MagneticSensorI2C(AS5600_I2C);

    I2C0.begin(19, 18, 400000);
    I2C1.begin(23, 5, 400000);

    sensorL.init(&I2C0);
    sensorR.init(&I2C1);
    
    Serial.println("Sensors initialized. Rotate motors by hand.");
    while (true) {
        sensorL.update();
        sensorR.update();
        Serial.print("L_Ang: "); Serial.print(sensorL.getAngle());
        Serial.print("\tR_Ang: "); Serial.println(sensorR.getAngle());
        delay(100);
    }
}

// ────────────────────────────────────────────────────────
// 2. Motor Open-Loop (PWM) Test
// ────────────────────────────────────────────────────────
void run_test_motor_pwm() {
    Serial.println("\n[Test 2] Starting Motor Open-Loop (PWM) Test...");
    BLDCMotor motor = BLDCMotor(7);
    BLDCDriver3PWM driver = BLDCDriver3PWM(M0_UH, M0_VH, M0_WH, M0_EN);

    driver.voltage_power_supply = 12.0f;
    driver.init();
    motor.linkDriver(&driver);

    motor.controller = MotionControlType::velocity_openloop;
    motor.voltage_limit = 2.0f; // Safe low voltage
    motor.init();
    
    Serial.println("Motor initialized. Target velocity: 2 rad/s");
    motor.target = 2.0f;
    while (true) {
        motor.move(); // No delay in FOC loop
    }
}

// ────────────────────────────────────────────────────────
// 3. Motor Closed-Loop (Velocity) Test
// ────────────────────────────────────────────────────────
void run_test_motor_velocity() {
    Serial.println("\n[Test 3] Starting Motor Closed-Loop Velocity Test...");
    TwoWire I2C0 = TwoWire(0);
    MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
    BLDCMotor motor = BLDCMotor(7);
    BLDCDriver3PWM driver = BLDCDriver3PWM(M0_UH, M0_VH, M0_WH, M0_EN);
    Commander command = Commander(Serial);
    
    I2C0.begin(19, 18, 400000);
    sensor.init(&I2C0);
    motor.linkSensor(&sensor);

    driver.voltage_power_supply = 12.0f;
    driver.init();
    motor.linkDriver(&driver);

    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;

    motor.PID_velocity.P = 0.1f;
    motor.PID_velocity.I = 1.0f;
    motor.voltage_limit = 6.0f;
    motor.velocity_limit = 20.0f;

    motor.init();
    motor.initFOC();

    command.add('T', [](char* cmd) { /* cannot capture in non-capturing lambda easily without global, use wrapper or direct */ }, "Target");
    // Workaround for lambda Commander binding in local scope:
    Serial.println("Send 'T5' to set 5 rad/s, 'T0' to stop.");
    motor.target = 0.0f;

    while (true) {
        motor.loopFOC();
        motor.move();
        
        // Simple inline command reading instead of Commander for self-contained test
        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'T' || c == 't') {
                float target = Serial.parseFloat();
                motor.target = target;
                Serial.print("Target set to: "); Serial.println(target);
            }
        }
    }
}

// ────────────────────────────────────────────────────────
// 4. IMU Test
// ────────────────────────────────────────────────────────
void run_test_imu() {
    Serial.println("\n[Test 4] Starting IMU (MPU6050) Test...");
    TwoWire I2C0 = TwoWire(0);
    I2C0.begin(19, 18, 400000);
    IMU imu(&I2C0);

    if (!imu.begin()) {
        Serial.println("IMU Init Failed! Check wiring.");
        while(true) delay(10);
    }
    Serial.println("IMU Init OK. Please rotate the board on flat surface.");
    
    uint32_t last_time = millis();
    while (true) {
        uint32_t now = millis();
        float dt = (now - last_time) / 1000.0f;
        last_time = now;
        
        float yaw = imu.update(dt);
        Serial.print("Yaw_Rad: "); Serial.print(yaw);
        Serial.print("\tYaw_Deg: "); Serial.println(yaw * 180.0f / M_PI);
        delay(10);
    }
}

// ────────────────────────────────────────────────────────
// 5. ToF Test
// ────────────────────────────────────────────────────────
void run_test_tof() {
    Serial.println("\n[Test 5] Starting ToF (VL53L0X) Test...");
    TwoWire I2C0 = TwoWire(0);
    I2C0.begin(19, 18, 400000);
    ToFSensors tof(&I2C0, TOF_FRONT_XSHUT, TOF_REAR_XSHUT);

    if (!tof.begin()) {
        Serial.println("ToF Init Failed! Check XSHUT pins and I2C.");
        while(true) delay(10);
    }
    Serial.println("ToF Init OK. Move hands in front of sensors.");
    
    while (true) {
        float front_m, rear_m;
        if (tof.read(front_m, rear_m)) {
            Serial.print("Front: "); Serial.print(front_m);
            Serial.print(" m \tRear: "); Serial.print(rear_m); Serial.println(" m");
        } else {
            Serial.println("ToF Read Error or Timeout!");
        }
        delay(50);
    }
}

// ────────────────────────────────────────────────────────
// Menu Logic
// ────────────────────────────────────────────────────────
void check_and_run_hw_tests() {
    Serial.println("========================================");
    Serial.println("  MoveLite Hardware Test Menu");
    Serial.println("========================================");
    Serial.println(" [1] Test Encoders (AS5600)");
    Serial.println(" [2] Test Motor PWM (Open Loop)");
    Serial.println(" [3] Test Motor Velocity (Closed Loop)");
    Serial.println(" [4] Test IMU (MPU6050)");
    Serial.println(" [5] Test ToF (VL53L0X)");
    Serial.println("----------------------------------------");
    Serial.println("Press 1-5 within 5 seconds to run a test...");
    
    unsigned long start_time = millis();
    while (millis() - start_time < 5000) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '1') run_test_encoder();
            if (c == '2') run_test_motor_pwm();
            if (c == '3') run_test_motor_velocity();
            if (c == '4') run_test_imu();
            if (c == '5') run_test_tof();
        }
        delay(1); // feed watchdog
    }
    Serial.println("No test selected. Booting normal MoveLite application...\n");
}
