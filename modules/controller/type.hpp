#pragma once
#include <cstdint>

namespace controller
{
    struct ControlResult
    {
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
        bool valid=true;
    };
}