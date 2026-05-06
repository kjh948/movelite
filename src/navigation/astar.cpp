#include "astar.h"
#include <math.h>
#include <string.h>

// ─── Static node pool (no heap alloc) ────────────────────────────────────────
static AStar::Node s_pool[AStar::MAX_NODES];
// Closed set: 1 byte per cell (bool)
static uint8_t     s_closed[OccupancyGrid::W * OccupancyGrid::H];

static constexpr int W = OccupancyGrid::W;
static constexpr int H = OccupancyGrid::H;

float AStar::heuristic(int x, int y, int gx, int gy) {
    // Octile distance (allows diagonal movement)
    int dx = abs(x - gx), dy = abs(y - gy);
    return (dx + dy) + (1.41421f - 2.0f) * (float)fmin(dx, dy);
}

// 8-directional neighbours
static const int DX[8] = {1,-1,0,0, 1,-1, 1,-1};
static const int DY[8] = {0,0,1,-1, 1,-1,-1, 1};
static const float DC[8] = {1,1,1,1, 1.41421f,1.41421f,1.41421f,1.41421f};

bool AStar::plan(const uint8_t* inflated,
                 int sx, int sy, int gx, int gy,
                 std::vector<GridCell>& path) {
    path.clear();
    if (sx == gx && sy == gy) { path.push_back({gx, gy}); return true; }

    memset(s_closed, 0, sizeof(s_closed));

    // open list as a simple sorted array (small grid → acceptable)
    int open_count = 0;

    // parent tracking: store parent index in pool
    // Use s_pool as both open list and parent map
    // For 100x100 grid, MAX_NODES=2000 is sufficient in practice
    int16_t parent_x[W * H];
    int16_t parent_y[W * H];
    float   g_cost[W * H];
    memset(parent_x, -1, sizeof(parent_x));
    memset(parent_y, -1, sizeof(parent_y));
    for (int i = 0; i < W*H; i++) g_cost[i] = 1e9f;

    g_cost[sy * W + sx] = 0;
    s_pool[open_count++] = {(int16_t)sx, (int16_t)sy, -1, -1,
                             0, heuristic(sx, sy, gx, gy)};

    while (open_count > 0) {
        // Find node with lowest f
        int best = 0;
        for (int i = 1; i < open_count; i++)
            if (s_pool[i].f < s_pool[best].f) best = i;

        Node cur = s_pool[best];
        // Remove from open
        s_pool[best] = s_pool[--open_count];

        int ci = cur.y * W + cur.x;
        if (s_closed[ci]) continue;
        s_closed[ci] = 1;

        if (cur.x == gx && cur.y == gy) {
            // Reconstruct path
            int rx = gx, ry = gy;
            while (!(rx == sx && ry == sy)) {
                path.push_back({rx, ry});
                int pi = ry * W + rx;
                int nx = parent_x[pi], ny = parent_y[pi];
                rx = nx; ry = ny;
            }
            path.push_back({sx, sy});
            // Reverse: path is goal→start, flip to start→goal
            for (int l=0, r=(int)path.size()-1; l<r; l++,r--)
                { auto t=path[l]; path[l]=path[r]; path[r]=t; }
            return true;
        }

        for (int d = 0; d < 8; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (inflated[ny * W + nx] == OccupancyGrid::OCCUPIED) continue;
            if (s_closed[ny * W + nx]) continue;

            float ng = cur.g + DC[d];
            int ni = ny * W + nx;
            if (ng < g_cost[ni]) {
                g_cost[ni]  = ng;
                parent_x[ni] = cur.x;
                parent_y[ni] = cur.y;
                if (open_count < MAX_NODES) {
                    s_pool[open_count++] = {
                        (int16_t)nx, (int16_t)ny,
                        cur.x, cur.y,
                        ng, ng + heuristic(nx, ny, gx, gy)
                    };
                }
            }
        }
    }
    return false;  // no path
}
