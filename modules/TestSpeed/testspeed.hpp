#pragma once
#include "modules/modules.hpp"
#include "Log/log.hpp"

class testspeed
{
public:
    ControlResult MoveTo(float pitch_want, float yaw_want)
    {
        reachAim = false;
        aim_pitch = pitch_want;
        aim_yaw = yaw_want;
        ControlResult result;
        result.shoot_flag = 0;
        result.pitch_setpoint = pitch_want;
        result.yaw_setpoint = yaw_want;
        INFO("Go for pitch: {}, yaw: {}", aim_pitch, aim_yaw);
        return result;
    }
    std::function<void(const ParsedSerialData&)> RecordFunc()
    {
        return [this](const ParsedSerialData& parsedData) {
            if (fabs(parsedData.pitch_now - aim_pitch) < 0.2 && fabs(parsedData.yaw_now - aim_yaw) < 0.2)
            {
                reachAim = true;
                INFO("reach aim");
            }
            else
            {
                reachAim = false;
                INFO("pitch: {}, yaw: {}", parsedData.pitch_now, parsedData.yaw_now);
            }
        };
    }
        
    bool reachAim=true;

private:
    float aim_pitch=0.0;
    float aim_yaw=0.0;
};