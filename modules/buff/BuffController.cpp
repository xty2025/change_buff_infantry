#include "BuffController.hpp"
#include "driver/driver.hpp"

namespace power_rune {
    BuffController::BuffController() {   }

    BuffController::~BuffController() {   }
    BuffControlResult BuffController::buff_control
    (const ParsedSerialData& parsedData,
     float target_pitch,
     float target_yaw) const
    {
        BuffControlResult result;
        result.yaw_setpoint = parsedData.yaw_now;
        result.pitch_setpoint = parsedData.pitch_now;
        result.pitch_actual_want = parsedData.pitch_now;
        result.yaw_actual_want = parsedData.yaw_now;
        result.valid = false;
        result.shoot_flag = false; 

        if (target_pitch > 89.0f && target_yaw > 89.0f)
        {
            result.yaw_setpoint = parsedData.yaw_now;
            result.pitch_setpoint = parsedData.pitch_now;
            result.pitch_actual_want = parsedData.pitch_now;
            result.yaw_actual_want = parsedData.yaw_now;
            result.valid = false;
            result.shoot_flag = false; 
        }
        else
        {
            result.yaw_setpoint = target_yaw;
            //转换成最近角。yaw会但pitch不用
            //actual->上游想要的角度。
            //set->发送给下位机的
            result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
            result.yaw_actual_want = target_yaw;
            
            result.pitch_setpoint = target_pitch;          
            result.pitch_actual_want = target_pitch;
   
            result.valid = true;
            result.shoot_flag = true; 
        }
        return result;
    }
}