#pragma once

#include <array>

struct Inputs {
    double xTarget;
    double yTarget;
    double zTarget;
};

struct State {
double posf[3];
double* velf;
double vel_ref[3];
double pos_ref[3];
};

struct Params {
    double position_maxAngle;
    double position_maxVel;
    double position_Kp_pos;
    double position_Kp_vel;
    double position_Ki_vel;
    double position_intLim;
    double freq;
    double g;
};

class URPosControl {
public:
    URPosControl() = default;

    double* control(const Inputs &inputs, State &state, const Params &par);

private:
    double errorInt[3] = {0.0, 0.0, 0.0};
};
