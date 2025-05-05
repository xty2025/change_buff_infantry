#pragma once
#include <cstdint>

namespace controller
{
    struct ControlResult
    {
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
        float pitch_actual_want;
        float yaw_actual_want;
        bool valid=true;
    };
}