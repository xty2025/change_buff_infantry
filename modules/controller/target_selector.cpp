#include "target_selector.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include "Location/location.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace controller {

using location::Location;

DistanceBasedSelector::DistanceBasedSelector(bool mouse_require, double cam_x, double cam_y, double response_speed)
    : mouse_require_(mouse_require), pic_camera_x_(cam_x), pic_camera_y_(cam_y), response_speed_(response_speed) {
}

TargetInfo DistanceBasedSelector::selectBestCar(const Predictions& predictions, const ParsedSerialData& serial_data) {
    TargetInfo best_target;
    double min_distance = std::numeric_limits<double>::max();
    
    for (const auto& prediction : predictions) {
        double distance = calculateDistance(prediction, serial_data);
        if (distance < min_distance) {
            min_distance = distance;
            best_target.car_id = prediction.id;
            best_target.armor_id = -1; // 将在selectBestArmor中选择
        }
    }
    
    return best_target;
}

TargetInfo DistanceBasedSelector::selectBestArmor(const Predictions& predictions,
                                                const TargetInfo& current_target,
                                                const ParsedSerialData& serial_data) {
    auto car_it = std::find_if(predictions.begin(), predictions.end(),
        [&](const auto& prediction) { return prediction.id == current_target.car_id; });
    
    if (car_it == predictions.end()) {
        return current_target;
    }
    
    // 首先检查当前装甲板是否仍然有效
    bool is_valid_armor_id = std::any_of(car_it->armors.begin(), car_it->armors.end(),
        [&](const auto& armor) { return (armor.id == current_target.armor_id) && (armor.status == predictor::Armor::AVAILABLE); });
    
    TargetInfo result = current_target;
    
    if (is_valid_armor_id && current_target.armor_id >= 0) {
        // 应用反小陀螺逻辑
        double distance = std::sqrt(car_it->center.x * car_it->center.x + car_it->center.y * car_it->center.y);
        double radius = (current_target.armor_id % 2 == 0) ? car_it->r1 : car_it->r2;
        double jump_angle = M_PI / 4 - ((std::abs(car_it->omega) > 0.01) ? 
            (1 / (1 + 1.414 * distance * response_speed_ / radius / std::abs(car_it->omega))) : 0);
        
        if (jump_angle < 0) jump_angle = M_PI / 6;
        
        if (std::abs(car_it->omega) > 0.6) { // 反小陀螺
            if (car_it->omega > 0 && car_it->armors[current_target.armor_id].yaw > jump_angle) {
                result.armor_id = (current_target.armor_id + 3) % 4;
            }
            else if (car_it->omega < 0 && car_it->armors[current_target.armor_id].yaw < -jump_angle) {
                result.armor_id = (current_target.armor_id + 1) % 4;
            }
            
            // 重新检查新装甲板是否有效
            is_valid_armor_id = std::any_of(car_it->armors.begin(), car_it->armors.end(),
                [&](const auto& armor) { return (armor.id == result.armor_id) && (armor.status == predictor::Armor::AVAILABLE); });
        }
    }
    
    if (!is_valid_armor_id) {
        // 重新选择距离最近的装甲板
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;
        
        for (const auto& armor : car_it->armors) {
            if (armor.status == predictor::Armor::AVAILABLE) {
                double distance = calculateDistance(armor, serial_data);
                if (distance < min_distance) {
                    min_distance = distance;
                    result.armor_id = armor.id;
                    found = true;
                }
            }
        }
        
        if (!found) {
            result.armor_id = -1;
        }
    }
    
    return result;
}

double DistanceBasedSelector::calculateDistance(const predictor::Armor& armor, const ParsedSerialData& serial_data) {
    if (!mouse_require_) {
        return armor.center.x * armor.center.x + armor.center.y * armor.center.y;
    } else {
        Location tmp;
        tmp.imu = ImuData(serial_data);
        tmp.xyz_imu = armor.center;
        CXYD tmp_cxy = tmp.cxy;
        return (tmp_cxy.cx - pic_camera_x_) * (tmp_cxy.cx - pic_camera_x_) +
               (tmp_cxy.cy - pic_camera_y_) * (tmp_cxy.cy - pic_camera_y_);
    }
}

double DistanceBasedSelector::calculateDistance(const predictor::Prediction& prediction, const ParsedSerialData& serial_data) {
    if (!mouse_require_) {
        return prediction.center.x * prediction.center.x + prediction.center.y * prediction.center.y;
    } else {
        Location tmp;
        tmp.imu = ImuData(serial_data);
        tmp.xyz_imu = prediction.center;
        CXYD tmp_cxy = tmp.cxy;
        return (tmp_cxy.cx - pic_camera_x_) * (tmp_cxy.cx - pic_camera_x_) +
               (tmp_cxy.cy - pic_camera_y_) * (tmp_cxy.cy - pic_camera_y_);
    }
}

} // namespace controller
