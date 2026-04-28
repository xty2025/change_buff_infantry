#pragma once
#include <memory>
#include <limits>
#include "target_state.hpp"
#include "driver/type.hpp"
#include "solver/type.hpp"
#include "Location/location.hpp"

namespace controller {

using driver::ParsedSerialData;
using solver::ImuData;
using location::Location;

class TargetSelector {
public:
    virtual ~TargetSelector() = default;
    virtual TargetInfo selectBestCar(const Predictions& predictions, 
                                   const ParsedSerialData& serial_data) = 0;
    virtual TargetInfo selectBestArmor(const Predictions& predictions,
                                     const TargetInfo& current_target,
                                     const ParsedSerialData& serial_data) = 0;
};

class DistanceBasedSelector : public TargetSelector {
private:
    bool mouse_require_;
    double pic_camera_x_;
    double pic_camera_y_;
    double response_speed_;
    
public:
    DistanceBasedSelector(bool mouse_require, double cam_x, double cam_y, double response_speed);
    
    TargetInfo selectBestCar(const Predictions& predictions, 
                           const ParsedSerialData& serial_data) override;
    
    TargetInfo selectBestArmor(const Predictions& predictions,
                             const TargetInfo& current_target,
                             const ParsedSerialData& serial_data) override;
                             
private:
    double calculateDistance(const predictor::Armor& armor, const ParsedSerialData& serial_data);
    double calculateDistance(const predictor::Prediction& prediction, const ParsedSerialData& serial_data);
};

} // namespace controller
