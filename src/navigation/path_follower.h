#pragma once
#include <Arduino.h>
#include <vector>
#include "../navigation/astar.h"
#include "../mapping/occupancy_grid.h"

// ─────────────────────────────────────────────
//  Pure Pursuit path follower
//  Inputs : current pose (x, y, theta), path
//  Outputs: (v, omega) to DifferentialDrive
// ─────────────────────────────────────────────
class PathFollower {
public:
    // look_ahead : look-ahead distance (meters)
    // max_v      : max linear speed (m/s)
    // goal_tol   : distance to goal that counts as "arrived" (meters)
    PathFollower(float look_ahead = 0.30f,
                 float max_v     = 0.30f,
                 float goal_tol  = 0.10f);

    // Set a new path (replaces existing)
    void setPath(const std::vector<GridCell>& path,
                 const OccupancyGrid& grid);

    // Compute (v, omega) given current pose
    // Returns false when path is complete (goal reached)
    bool compute(float x, float y, float theta,
                 float& v, float& omega);

    bool hasPath() const { return !_waypoints.empty(); }
    void clearPath() { _waypoints.clear(); _wp_idx = 0; }

private:
    struct Waypoint { float x, y; };

    float _look_ahead;
    float _max_v;
    float _goal_tol;

    std::vector<Waypoint> _waypoints;
    size_t                _wp_idx = 0;

    // Find the furthest waypoint within look-ahead circle
    bool findLookAheadPoint(float x, float y,
                             float& lx, float& ly);

    static float wrapAngle(float a);
};
