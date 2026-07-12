#pragma once

#include <Arduino.h>

struct RPY {
    float roll;
    float pitch;
    float yaw;
};

class TrajectoryInterpolator {
public:
    // set target orientation (rpy_final) over time duration
    void set_target(RPY start, RPY end, float duration);

    // get interpolated orientation at current time
    RPY update(float dt);

    bool is_done() const { return _elapsed >= _duration; }
    float elapsed() const { return _elapsed; }

private:
    RPY _start;
    RPY _end;
    float _duration = 0.0f;
    float _elapsed = 0.0f;

    // cubic polynomial trajectory generation
    float _cubic(float q_start, float q_end, float s) const;

    // ----> ADD TRAPEZOIDAL VELOCITY TRAJECTORY GEN <-----
};