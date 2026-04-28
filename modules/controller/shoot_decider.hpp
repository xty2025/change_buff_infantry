#pragma once
#include <chrono>
#include <functional>
#include "predictor/type.hpp"
#include "driver/type.hpp"

// 前向声明，避免循环依赖
namespace controller {
    class BallisticCalculator;
}

namespace controller {

using predictor::Armor;
using driver::ParsedSerialData;

struct ShootConditions {
    double angle_tolerance_x = 0.2;
    double angle_tolerance_y = 0.1;
    double min_confidence = 0.8;
    bool require_stable_tracking = true;
};

class ShootDecider {
private:
    ShootConditions conditions_;
    
public:
    void configure(const ShootConditions& conditions) { 
        conditions_ = conditions; 
    }
    
    bool shouldShoot(const Armor& armor,
                    const ParsedSerialData& serial_data,
                    double calculated_pitch,
                    double calculated_yaw) const;
                    
    // 为瞄准中心模式提供一个简化的接口，直接传入BallisticCalculator
    bool shouldShootAtCenter(const std::array<Armor, 4>& armors,
                           const ParsedSerialData& serial_data,
                           const BallisticCalculator* ballistic_calc) const;
};

} // namespace controller
