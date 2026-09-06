#include "runtime/control_adapter.hpp"

#include <cmath>

#include "global/utils.hpp"

CtrlCmd MakeStopCommand() {
    CtrlCmd command;
    command.ctrl_mode = 2;
    command.gear = 4;
    command.cmd_type = 1;
    command.accel = 0.0f;
    command.brake = 0.3f;
    command.steer = 0.0f;
    return command;
}

CtrlCmd ToControlCommand(const MPCControl& control, const MPCParams& params) {
    const double steer_radians =
        clip(control.delta, -params.steering_max, params.steering_max);
    const double steer =
        clip(steer_radians / params.steering_max, -1.0, 1.0);

    double accel = 0.0;
    double brake = 0.0;
    if (control.accel >= 0.0) {
        accel = clip(control.accel / params.accel_max, 0.0, 1.0);
    } else {
        brake = clip(-control.accel / params.brake_full_decel, 0.0, 1.0);
    }

    CtrlCmd command;
    command.ctrl_mode = 2;
    command.gear = 4;
    command.cmd_type = 1;
    command.accel = static_cast<float>(accel);
    command.brake = static_cast<float>(brake);
    command.steer = static_cast<float>(steer);
    return command;
}
