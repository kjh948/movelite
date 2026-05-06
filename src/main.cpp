#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "motor/foc_motor.h"
#include "motor/differential_drive.h"
#include "sensor/imu.h"
#include "sensor/tof.h"
#include "mapping/occupancy_grid.h"
#include "navigation/astar.h"
#include "navigation/path_follower.h"
#include "web/web_server.h"
#include "hw_test.h"

// ─── Global State & Synchronization ──────────────────────────────────────────
RobotState        g_state;
SemaphoreHandle_t g_state_mutex;

// ─── Hardware Instances ──────────────────────────────────────────────────────
TwoWire I2C0 = TwoWire(0); // SDA=19, SCL=18
TwoWire I2C1 = TwoWire(1); // SDA=23, SCL=5

SimpleFOCWrapper motor_l(M0_UH, M0_VH, M0_WH, M0_EN, &I2C0, 7);
SimpleFOCWrapper motor_r(M1_UH, M1_VH, M1_WH, M1_EN, &I2C1, 7);

DifferentialDrive drive(ROBOT_WHEEL_RADIUS, ROBOT_WHEEL_BASE);
IMU               imu(&I2C0);
ToFSensors        tof(&I2C0, TOF_FRONT_XSHUT, TOF_REAR_XSHUT);

OccupancyGrid     grid;
PathFollower      follower;

WebServer         web_server(80);

// For navigation
uint8_t               inflated_map[OccupancyGrid::W * OccupancyGrid::H];
std::vector<GridCell> current_path;

// Task handles
TaskHandle_t hCore1Task;

// WiFi credentials (replace with yours)
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASS";

// ─── Core 1: Hard Real-Time Control Loop ─────────────────────────────────────
void core1_task(void* pvParameters) {
    TickType_t last_100hz = xTaskGetTickCount();
    TickType_t last_20hz  = xTaskGetTickCount();
    
    float manual_v = 0.0f;
    float manual_w = 0.0f;

    while (true) {
        // 1. Run FOC Loop as fast as possible (no delays here)
        motor_l.loopFOC();
        motor_r.loopFOC();

        TickType_t now = xTaskGetTickCount();

        // 2. 100Hz updates (Odometry, IMU)
        if (now - last_100hz >= pdMS_TO_TICKS(10)) {
            float dt = (now - last_100hz) / 1000.0f;
            last_100hz = now;

            float yaw = imu.update(dt);
            
            // Get measured velocities
            float v_l = motor_l.getVelocity();
            float v_r = motor_r.getVelocity(); // Need to handle right motor direction inversion if needed based on mounting
            
            drive.updateOdometry(v_l, -v_r, dt); // Assuming right motor is mounted mirrored
            drive.setTheta(yaw); // Override Odometry theta with IMU yaw

            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_state.x = drive.getX();
            g_state.y = drive.getY();
            g_state.theta = drive.getTheta();
            g_state.vel_l = v_l;
            g_state.vel_r = v_r;
            xSemaphoreGive(g_state_mutex);
        }

        // 3. 20Hz updates (Sensors, Mapping, Navigation, Web Commands)
        if (now - last_20hz >= pdMS_TO_TICKS(50)) {
            last_20hz = now;

            // Process commands from Core 0
            RobotCommand cmd;
            while (web_server.popCommand(cmd)) {
                if (cmd.type == CmdType::JOYSTICK) {
                    manual_v = cmd.param1;
                    manual_w = cmd.param2;
                } else if (cmd.type == CmdType::START_MAPPING) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state.mode = RobotMode::MAPPING;
                    xSemaphoreGive(g_state_mutex);
                } else if (cmd.type == CmdType::STOP_MAPPING) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state.mode = RobotMode::MAP_READY;
                    xSemaphoreGive(g_state_mutex);
                } else if (cmd.type == CmdType::SET_GOAL && g_state.mode == RobotMode::MAP_READY) {
                    int gx = (int)cmd.param1;
                    int gy = (int)cmd.param2;
                    int sx, sy;
                    if (grid.worldToGrid(drive.getX(), drive.getY(), sx, sy)) {
                        grid.buildInflated(inflated_map);
                        if (AStar::plan(inflated_map, sx, sy, gx, gy, current_path)) {
                            follower.setPath(current_path, grid);
                            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                            g_state.mode = RobotMode::NAVIGATING;
                            xSemaphoreGive(g_state_mutex);
                        }
                    }
                }
            }

            // Read ToF Sensors
            float front_m, rear_m;
            if (tof.read(front_m, rear_m)) {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_state.tof_front = front_m;
                g_state.tof_rear = rear_m;
                xSemaphoreGive(g_state_mutex);

                // Update Map if in MAPPING mode
                if (g_state.mode == RobotMode::MAPPING) {
                    grid.updateRay(drive.getX(), drive.getY(), drive.getTheta(), 0.0f, front_m);
                    grid.updateRay(drive.getX(), drive.getY(), drive.getTheta(), M_PI, rear_m);
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state.map_dirty = true;
                    xSemaphoreGive(g_state_mutex);
                }
            }

            // Calculate Target Velocities based on Mode
            float target_v = 0.0f, target_w = 0.0f;
            
            if (g_state.mode == RobotMode::MAPPING || g_state.mode == RobotMode::IDLE || g_state.mode == RobotMode::MAP_READY) {
                target_v = manual_v;
                target_w = manual_w;
            } else if (g_state.mode == RobotMode::NAVIGATING) {
                if (!follower.compute(drive.getX(), drive.getY(), drive.getTheta(), target_v, target_w)) {
                    // Reached goal
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state.mode = RobotMode::MAP_READY;
                    xSemaphoreGive(g_state_mutex);
                }
                
                // Very basic obstacle avoidance during navigation
                if (front_m < 0.2f && target_v > 0) {
                     target_v = 0; target_w = 0; // Stop
                     // Trigger replanning logic here in a full implementation
                }
            }

            // Convert (v,w) to wheel velocities
            float l_rads, r_rads;
            drive.setVelocity(target_v, target_w, l_rads, r_rads);
            
            motor_l.setVelocity(l_rads);
            motor_r.setVelocity(-r_rads); // Apply mirrored inverse
        }
        
        // Yield to let watchdog breathe slightly if needed, but SimpleFOC loop needs to be tight.
        // vTaskDelay(1) would drop loop to 1kHz, which might be too slow for FOC depending on motor.
        // We rely on FreeRTOS preemption if other tasks need time on Core 1 (none do).
    }
}


// ─── Core 0: Setup and Web Task ──────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000); // Give serial time to attach
    
    // Check for hardware tests before starting normal system
    check_and_run_hw_tests();

    g_state_mutex = xSemaphoreCreateMutex();

    // 1. Initialize I2C Buses
    I2C0.begin(19, 18, 400000);
    I2C1.begin(23, 5, 400000);

    // 2. Initialize Sensors (on I2C0)
    if (!imu.begin()) Serial.println("IMU Init Failed!");
    if (!tof.begin()) Serial.println("ToF Init Failed!");

    // 3. Initialize Motors (12V assumed, adjust if using 3S LiPo ~11.1V-12.6V)
    if (!motor_l.begin(12.0f)) Serial.println("Motor L Init Failed!");
    if (!motor_r.begin(12.0f)) Serial.println("Motor R Init Failed!");

    // 4. Start Core 1 Control Task (Priority 1 - High)
    xTaskCreatePinnedToCore(
        core1_task, "Core1_Ctrl", 8192, NULL, 1, &hCore1Task, 1
    );

    // 5. Initialize Web Server on Core 0
    web_server.begin(WIFI_SSID, WIFI_PASS);
}

void loop() {
    // Core 0 loop: 20Hz telemetry broadcast
    delay(50);
    
    RobotState current_state;
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    current_state = g_state;
    g_state.map_dirty = false; // Clear dirty flag locally after reading
    xSemaphoreGive(g_state_mutex);

    web_server.broadcastState(current_state, grid, g_state_mutex);
}
