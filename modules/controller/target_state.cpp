#include "target_state.hpp"
#include <algorithm>
#include <cmath>

namespace controller {

void TargetStateManager::configure(int max_lost_frames, int min_stable_frames, int wait_frame) {
    max_lost_frames_ = max_lost_frames;
    min_stable_frames_ = min_stable_frames;
    wait_frame_ = wait_frame;
}

bool TargetStateManager::judgeAimNew(bool request) {
    aim_new_ = false;
    if(((!aiming_) && request) || (aiming_ && (!request))) {
        accumulate_aim_request_++;
    }
    else {
        accumulate_aim_request_ = 0;
    }
    if(accumulate_aim_request_ > wait_frame_) {
        accumulate_aim_request_ = 0;
        aiming_ = request;
        if(aiming_)
            aim_new_ = true;
    }
    return aim_new_;
}

TargetState TargetStateManager::updateState(const Predictions& predictions, bool aim_request) {
    // 如果不需要瞄准，重置状态
    if (!aim_request) {
        transitionTo(TargetState::SEARCHING);
        current_target_.reset();
        return current_state_;
    }
    
    bool target_available = isTargetAvailable(predictions);
    
    switch (current_state_) {
        case TargetState::SEARCHING:
            if (target_available) {
                current_target_.stable_frames++;
                if (current_target_.stable_frames >= min_stable_frames_) {
                    transitionTo(TargetState::TRACKING);
                }
            } else {
                current_target_.reset();
            }
            break;
            
        case TargetState::TRACKING:
            if (target_available) {
                current_target_.lost_frames = 0;
                current_target_.last_seen = std::chrono::steady_clock::now();
            } else {
                current_target_.lost_frames++;
                if (current_target_.lost_frames >= max_lost_frames_) {
                    transitionTo(TargetState::LOST);
                }
            }
            break;
            
        case TargetState::LOST:
            if (target_available) {
                current_target_.lost_frames = 0;
                transitionTo(TargetState::TRACKING);
            } else {
                // 可以设置最大丢失时间后重新搜索
                //auto now = std::chrono::steady_clock::now();
                //auto lost_duration = now - current_target_.last_seen;
                //if (lost_duration > std::chrono::seconds(2)) {
                    transitionTo(TargetState::SEARCHING);
                    current_target_.reset();
                //}
            }
            break;
            
        case TargetState::SWITCHING:
            if (target_available) {
                transitionTo(TargetState::TRACKING);
            }
            break;
    }
    
    return current_state_;
}

bool TargetStateManager::isTargetAvailable(const Predictions& predictions) const {
    if (!current_target_.isValid()) {
        return false;
    }
    
    // 检查car_id是否存在
    auto car_it = std::find_if(predictions.begin(), predictions.end(),
        [&](const auto& prediction) { return prediction.id == current_target_.car_id; });
    
    if (car_it == predictions.end()) {
        return false;
    }
    
    // 如果armor_id有效，检查armor是否可用
    if (current_target_.armor_id >= 0) {
        auto armor_it = std::find_if(car_it->armors.begin(), car_it->armors.end(),
            [&](const auto& armor) { 
                return armor.id == current_target_.armor_id && 
                       armor.status == predictor::Armor::AVAILABLE; 
            });
        return armor_it != car_it->armors.end();
    }
    
    return true;
}

void TargetStateManager::transitionTo(TargetState new_state) {
    if (current_state_ != new_state) {
        current_state_ = new_state;
        // 可以在这里添加状态转换的日志
    }
}

} // namespace controller
