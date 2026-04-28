#include "shoot_decider.hpp"
#include "ballistic_calculator.hpp"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace controller {

bool ShootDecider::shouldShoot(const Armor& armor,
                              const ParsedSerialData& serial_data,
                              double calculated_pitch,
                              double calculated_yaw) const {
    // 计算角度差异
    double delta_yaw = std::remainder(calculated_pitch - serial_data.pitch_now * M_PI / 180, 2 * M_PI);
    double delta_pitch = std::remainder(calculated_yaw - serial_data.yaw_now * M_PI / 180, 2 * M_PI);
    double dist = std::sqrt(armor.center.x * armor.center.x + armor.center.y * armor.center.y + armor.center.z * armor.center.z);
    
    // 检查角度容差
    return (std::tan(delta_yaw) * dist < conditions_.angle_tolerance_x) && 
           (std::tan(delta_pitch) * dist < conditions_.angle_tolerance_y);
}

bool ShootDecider::shouldShootAtCenter(const std::array<Armor, 4>& armors,
                                     const ParsedSerialData& serial_data,
                                     const BallisticCalculator* ballistic_calc) const {
    if (!ballistic_calc) return false;
    
    // 在瞄准中心模式下，检查是否有任何装甲板可以命中
    for (const auto& armor : armors) {
        if (armor.status == predictor::Armor::AVAILABLE) {
            // 为每个装甲板计算弹道（包含射击表补偿）
            auto ballistic_result = ballistic_calc->calculate(armor.center.x, armor.center.y, armor.center.z);
            if (ballistic_result.success) {
                // 检查是否满足射击条件
                if (shouldShoot(armor, serial_data, ballistic_result.pitch, ballistic_result.yaw)) {
                    return true; // 只要有一个装甲板可以命中就射击
                }
            }
        }
    }
    return false;
}

} // namespace controller
