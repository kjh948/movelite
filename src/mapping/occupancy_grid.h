#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "../sensor/tof.h"

// ─────────────────────────────────────────────
//  OccupancyGrid
//  100×100 cells, 10cm/cell → 10m×10m area
//  Cell values: 0=unknown, 127=free, 255=occupied
//  Origin (0,0) = center of grid
// ─────────────────────────────────────────────
class OccupancyGrid {
public:
    static constexpr int   W    = MAP_WIDTH;       // 100
    static constexpr int   H    = MAP_HEIGHT;      // 100
    static constexpr float RES  = MAP_RESOLUTION;  // 0.10 m/cell

    static constexpr uint8_t UNKNOWN  = 0;
    static constexpr uint8_t FREE     = 127;
    static constexpr uint8_t OCCUPIED = 255;

    // Inflation radius in cells (for A* planning)
    static constexpr int INFLATE_R = 2;

    OccupancyGrid();

    void reset();

    // Update map from a single ToF ray
    // robot_x, robot_y, robot_theta : world pose (meters, radians)
    // ray_angle_offset : angle of sensor relative to robot heading (radians)
    //                    front=0, rear=M_PI
    // distance : measured distance in meters (0 = no obstacle / max range)
    void updateRay(float robot_x, float robot_y, float robot_theta,
                   float ray_angle_offset, float distance);

    // Access raw cell value (grid coordinates)
    uint8_t get(int gx, int gy) const;
    void    set(int gx, int gy, uint8_t val);

    // Convert world ↔ grid coordinates
    bool worldToGrid(float wx, float wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, float& wx, float& wy) const;

    // Build inflated copy for path planning (obstacles expanded by INFLATE_R)
    // out must be W*H bytes (caller allocates on stack or static buffer)
    void buildInflated(uint8_t* out) const;

    // Raw data pointer for WebSocket transmission
    const uint8_t* data() const { return _cells; }

private:
    uint8_t _cells[W * H];

    // Bresenham ray-cast: mark cells free up to hit, occupied at hit
    void raycast(int x0, int y0, int x1, int y1, bool obstacle);

    bool inBounds(int gx, int gy) const {
        return gx >= 0 && gx < W && gy >= 0 && gy < H;
    }
};
