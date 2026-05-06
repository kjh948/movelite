#include "occupancy_grid.h"
#include <stdlib.h>

OccupancyGrid::OccupancyGrid() {
    reset();
}

void OccupancyGrid::reset() {
    memset(_cells, UNKNOWN, sizeof(_cells));
}

// ─── World ↔ Grid ─────────────────────────────────────────────────────────────

bool OccupancyGrid::worldToGrid(float wx, float wy, int& gx, int& gy) const {
    // Grid origin is at center (W/2, H/2)
    gx = (int)floorf(wx / RES) + W / 2;
    gy = (int)floorf(wy / RES) + H / 2;
    return inBounds(gx, gy);
}

void OccupancyGrid::gridToWorld(int gx, int gy, float& wx, float& wy) const {
    wx = ((float)(gx - W / 2) + 0.5f) * RES;
    wy = ((float)(gy - H / 2) + 0.5f) * RES;
}

// ─── Cell access ──────────────────────────────────────────────────────────────

uint8_t OccupancyGrid::get(int gx, int gy) const {
    if (!inBounds(gx, gy)) return OCCUPIED;  // treat out-of-bounds as obstacle
    return _cells[gy * W + gx];
}

void OccupancyGrid::set(int gx, int gy, uint8_t val) {
    if (inBounds(gx, gy)) _cells[gy * W + gx] = val;
}

// ─── Ray update ───────────────────────────────────────────────────────────────

void OccupancyGrid::updateRay(float robot_x, float robot_y, float robot_theta,
                               float ray_angle_offset, float distance) {
    float angle = robot_theta + ray_angle_offset;

    int rx, ry;
    if (!worldToGrid(robot_x, robot_y, rx, ry)) return;

    bool obstacle = (distance < ToFSensors::MAX_RANGE - 0.05f);

    float hit_x = robot_x + cosf(angle) * distance;
    float hit_y = robot_y + sinf(angle) * distance;
    int hx, hy;
    if (!worldToGrid(hit_x, hit_y, hx, hy)) {
        // Ray exits map — only mark free cells within map
        float max_d = fminf(distance, ToFSensors::MAX_RANGE);
        hit_x = robot_x + cosf(angle) * max_d;
        hit_y = robot_y + sinf(angle) * max_d;
        worldToGrid(hit_x, hit_y, hx, hy);
        obstacle = false;
    }
    raycast(rx, ry, hx, hy, obstacle);
}

// ─── Bresenham ray-cast ───────────────────────────────────────────────────────

void OccupancyGrid::raycast(int x0, int y0, int x1, int y1, bool obstacle) {
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        bool at_end = (x0 == x1 && y0 == y1);
        if (at_end && obstacle) {
            // Only mark occupied if not already free
            if (get(x0, y0) != FREE) set(x0, y0, OCCUPIED);
        } else if (!at_end) {
            // Mark intermediate cells as free (don't overwrite occupied)
            if (get(x0, y0) != OCCUPIED) set(x0, y0, FREE);
        }
        if (at_end) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

// ─── Inflation ────────────────────────────────────────────────────────────────

void OccupancyGrid::buildInflated(uint8_t* out) const {
    memcpy(out, _cells, W * H);

    for (int gy = 0; gy < H; gy++) {
        for (int gx = 0; gx < W; gx++) {
            if (_cells[gy * W + gx] == OCCUPIED) {
                // Inflate: mark surrounding cells as OCCUPIED in output
                for (int dy = -INFLATE_R; dy <= INFLATE_R; dy++) {
                    for (int dx = -INFLATE_R; dx <= INFLATE_R; dx++) {
                        if (dx*dx + dy*dy <= INFLATE_R*INFLATE_R) {
                            int nx = gx + dx, ny = gy + dy;
                            if (inBounds(nx, ny))
                                out[ny * W + nx] = OCCUPIED;
                        }
                    }
                }
            }
        }
    }
}
