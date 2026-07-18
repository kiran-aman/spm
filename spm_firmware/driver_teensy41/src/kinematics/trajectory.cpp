#include "trajectory.h"

float TrajectoryInterpolator::_cubic(float q_start, float q_end, float s) const {
    // clamp s to [0, 1]
    if(s < 0.0f) { s = 0.0f; }
    if(s > 1.0f) { s = 1.0f; }

    float blend = 3.0f*s*s - 2.0f*s*s*s;
    return q_start + (q_end - q_start) * blend;
}

void TrajectoryInterpolator::set_target(RPY start, RPY end, float duration) {
    _start = start;
    _end = end;
    _duration = duration;
    _elapsed = 0.0f;
}

RPY TrajectoryInterpolator::update(float dt) {
    _elapsed += dt;
    float s = (_duration > 0.0f) ? (_elapsed / _duration) : 1.0f;

    RPY out;
    out.roll = _cubic(_start.roll, _end.roll, s);
    out.pitch = _cubic(_start.pitch, _end.pitch, s);
    out.yaw = _cubic(_start.yaw, _end.yaw, s);
    return out;
}