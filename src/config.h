#pragma once

// ─────────────────────────────────────────────
//  Robot state machine
// ─────────────────────────────────────────────
enum class RobotMode : uint8_t {
    IDLE       = 0,
    MAPPING    = 1,
    MAP_READY  = 2,
    NAVIGATING = 3,
    REPLANNING = 4,
    ARRIVED    = 5,
    ERROR      = 6,
};

// ─────────────────────────────────────────────
//  Shared robot state (written by Core 1, read by Core 0)
//  Access guarded by state_mutex
// ─────────────────────────────────────────────
struct RobotState {
    float x     = 0.0f;   // meters
    float y     = 0.0f;
    float theta = 0.0f;   // radians
    float vel_l = 0.0f;   // left wheel rad/s
    float vel_r = 0.0f;
    float tof_front = 0.0f;  // meters
    float tof_rear  = 0.0f;
    RobotMode mode  = RobotMode::IDLE;
    bool  map_dirty = false;  // set true when grid updated
};

// ─────────────────────────────────────────────
//  Command from web → Core 1
// ─────────────────────────────────────────────
enum class CmdType : uint8_t {
    NONE          = 0,
    JOYSTICK      = 1,   // manual drive
    SET_GOAL      = 2,   // autonomous nav goal
    START_MAPPING = 3,
    STOP_MAPPING  = 4,
    RESET         = 5,
};

struct RobotCommand {
    CmdType type  = CmdType::NONE;
    float   param1 = 0.0f;  // v  (m/s)  or goal_x (grid)
    float   param2 = 0.0f;  // ω (rad/s) or goal_y (grid)
};
