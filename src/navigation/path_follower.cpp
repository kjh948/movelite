#include "path_follower.h"
#include <math.h>

PathFollower::PathFollower(float look_ahead, float max_v, float goal_tol)
    : _look_ahead(look_ahead), _max_v(max_v), _goal_tol(goal_tol) {}

void PathFollower::setPath(const std::vector<GridCell>& path,
                            const OccupancyGrid& grid) {
    _waypoints.clear();
    _wp_idx = 0;
    for (const auto& cell : path) {
        float wx, wy;
        grid.gridToWorld(cell.x, cell.y, wx, wy);
        _waypoints.push_back({wx, wy});
    }
}

bool PathFollower::compute(float x, float y, float theta,
                            float& v, float& omega) {
    v = 0; omega = 0;
    if (_waypoints.empty()) return false;

    // Check if goal reached
    const Waypoint& goal = _waypoints.back();
    float dx = goal.x - x, dy = goal.y - y;
    if (sqrtf(dx*dx + dy*dy) < _goal_tol) {
        clearPath();
        return false;  // arrived
    }

    // Advance _wp_idx past any already-passed waypoints
    while (_wp_idx < _waypoints.size() - 1) {
        float dwx = _waypoints[_wp_idx].x - x;
        float dwy = _waypoints[_wp_idx].y - y;
        if (sqrtf(dwx*dwx + dwy*dwy) < _look_ahead * 0.5f)
            _wp_idx++;
        else
            break;
    }

    // Find look-ahead point
    float lx, ly;
    if (!findLookAheadPoint(x, y, lx, ly)) {
        // Fallback: head directly to next waypoint
        lx = _waypoints[_wp_idx].x;
        ly = _waypoints[_wp_idx].y;
    }

    // Pure Pursuit curvature
    float alpha = wrapAngle(atan2f(ly - y, lx - x) - theta);
    float ld    = sqrtf((lx-x)*(lx-x) + (ly-y)*(ly-y));

    if (ld < 1e-4f) { v = 0; omega = 0; return true; }

    float curvature = 2.0f * sinf(alpha) / ld;

    // Scale speed based on curvature (slow down on tight turns)
    float speed = _max_v / (1.0f + fabsf(curvature) * 0.5f);
    speed = fminf(speed, _max_v);

    v     = speed;
    omega = speed * curvature;
    return true;
}

bool PathFollower::findLookAheadPoint(float x, float y,
                                       float& lx, float& ly) {
    // Walk waypoints from _wp_idx to end; find last one inside look-ahead circle
    bool found = false;
    for (size_t i = _wp_idx; i < _waypoints.size(); i++) {
        float dx = _waypoints[i].x - x;
        float dy = _waypoints[i].y - y;
        float d  = sqrtf(dx*dx + dy*dy);
        if (d <= _look_ahead) {
            lx = _waypoints[i].x;
            ly = _waypoints[i].y;
            found = true;
        } else if (found) {
            break;  // stepped beyond look-ahead
        }
    }
    if (!found && _wp_idx < _waypoints.size()) {
        lx = _waypoints[_wp_idx].x;
        ly = _waypoints[_wp_idx].y;
        found = true;
    }
    return found;
}

float PathFollower::wrapAngle(float a) {
    while (a >  M_PI) a -= 2.0f * M_PI;
    while (a < -M_PI) a += 2.0f * M_PI;
    return a;
}
