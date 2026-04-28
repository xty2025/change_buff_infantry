#pragma once
#include <cstdint>
#include <functional>
#include "driver/type.hpp"
#include "predictor/type.hpp"
#include "solver/type.hpp"

namespace power_rune {
    using driver::ParsedSerialData;
    struct BuffControlResult {
        uint8_t shoot_flag;
        float pitch_setpoint;
        float yaw_setpoint;
        float pitch_actual_want;
        float yaw_actual_want;
        bool valid = true;
    };

    class BuffController {
        public:
            BuffController();
            ~BuffController();

            BuffControlResult buff_control(const ParsedSerialData& parsedData,
                                           float target_pitch,
                                           float target_yaw) const;
        };

}