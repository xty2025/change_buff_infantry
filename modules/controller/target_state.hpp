#pragma once
#include <chrono>
#include "predictor/type.hpp"

namespace controller {

using predictor::Predictions;

enum class TargetState {
    SEARCHING,      // 搜索目标
    TRACKING,       // 跟踪目标
    LOST,          // 目标丢失
    SWITCHING      // 切换目标
};

struct TargetInfo {
    int car_id = -1;
    int armor_id = -1;
    int lost_frames = 0;
    int stable_frames = 0;
    std::chrono::steady_clock::time_point last_seen;
    
    bool isValid() const { return car_id >= 0; }
    void reset() { 
        car_id = -1; 
        armor_id = -1; 
        lost_frames = 0; 
        stable_frames = 0; 
    }
};

class TargetStateManager {
private:
    TargetState current_state_ = TargetState::SEARCHING;
    TargetInfo current_target_;
    
    // 配置参数
    int max_lost_frames_ = 10;
    int min_stable_frames_ = 3;
    int wait_frame_ = 5;
    
public:
    void configure(int max_lost_frames, int min_stable_frames, int wait_frame);
    TargetState updateState(const Predictions& predictions, bool aim_request);
    TargetInfo getCurrentTarget() const { return current_target_; }
    void setCurrentTarget(const TargetInfo& target) { current_target_ = target; }
    TargetState getCurrentState() const { return current_state_; }
    
    // 兼容原代码的接口
    bool judgeAimNew(bool request);
    std::pair<int, int> getAimArmorId() const { 
        return std::make_pair(current_target_.car_id, current_target_.armor_id); 
    }
    void setAimArmorId(const std::pair<int, int>& id) {
        current_target_.car_id = id.first;
        current_target_.armor_id = id.second;
    }
    
private:
    bool isTargetAvailable(const Predictions& predictions) const;
    void transitionTo(TargetState new_state);
    
    // 兼容原代码的状态变量
    bool aim_new_ = false;
    bool aiming_ = false;
    int accumulate_aim_request_ = 0;
};

} // namespace controller
