#pragma once
#include <stdint.h>
#include <vector>
#include "../mapping/occupancy_grid.h"

// ─────────────────────────────────────────────
//  A* path planner on OccupancyGrid
//  Returns path as list of grid cells (gx, gy)
// ─────────────────────────────────────────────

struct GridCell {
    int x, y;
};

class AStar {
public:
    // max_nodes: upper bound on open-set size (SRAM guard)
    static constexpr int MAX_NODES = 2000;

    // Plan path from (sx,sy) to (gx,gy) on inflated map.
    // inflated: W*H byte array where OCCUPIED=255 is impassable.
    // Returns true and fills 'path' with grid cells (start→goal).
    // Returns false if no path found or map full.
    struct Node {
        int16_t x, y;
        int16_t parent_x, parent_y;
        float   g, f;
    };

    static bool plan(const uint8_t* inflated,
                     int sx, int sy, int gx, int gy,
                     std::vector<GridCell>& path);

    static float heuristic(int x, int y, int gx, int gy);
};
