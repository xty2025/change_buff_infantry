#include "controller.hpp"
#include <algorithm>
#include <cmath>
#include <Log/log.hpp>
#include <Udpsend/udpsend.hpp>

const double PI = 3.1415926;

auto controller::createController(param::Param json_param) -> std::shared_ptr<controller::Controller>
{
    return std::make_shared<controller::Controller>(json_param);
}

using namespace controller;
using namespace aimlog;
using param::Param;

Controller::Controller(const param::Param& json_param) : json_param_(json_param) {
    readJsonParam();
    initializeComponents();
}

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}

void Controller::readJsonParam()
{
    mouse_require_ = json_param_["mouse_require"].Bool();
    pic_camera_x_ = json_param_["pic_camera_x"].Double();
    pic_camera_y_ = json_param_["pic_camera_y"].Double();
    response_speed_ = json_param_["response_speed"].Double();
    if (json_param_.exists("yaw_smoothing")) {
        yaw_smoothing_enable_ = json_param_["yaw_smoothing"]["enable"].Bool();
        yaw_smoothing_alpha_ = json_param_["yaw_smoothing"]["alpha"].Double();
        yaw_smoothing_alpha_ = std::clamp(yaw_smoothing_alpha_, 0.0, 1.0);
    }
    shoot_delay_ = std::chrono::duration<double>(json_param_["shoot_delay"].Double());
    shoot_table_adjust_ = json_param_["shoot_table_adjust"]["enable"].Bool();

    pitch_param_.resize(6, 0.0);
    yaw_param_.resize(6, 0.0);

    if (shoot_table_adjust_) {
        pitch_param_[0] = json_param_["shoot_table_adjust"]["pitch"]["intercept"].Double();
        pitch_param_[1] = json_param_["shoot_table_adjust"]["pitch"]["coef_z"].Double();
        pitch_param_[2] = json_param_["shoot_table_adjust"]["pitch"]["coef_d"].Double();
        pitch_param_[3] = json_param_["shoot_table_adjust"]["pitch"]["coef_z2"].Double();
        pitch_param_[4] = json_param_["shoot_table_adjust"]["pitch"]["coef_zd"].Double();
        pitch_param_[5] = json_param_["shoot_table_adjust"]["pitch"]["coef_d2"].Double();

        yaw_param_[0] = json_param_["shoot_table_adjust"]["yaw"]["intercept"].Double();
        yaw_param_[1] = json_param_["shoot_table_adjust"]["yaw"]["coef_z"].Double();
        yaw_param_[2] = json_param_["shoot_table_adjust"]["yaw"]["coef_d"].Double();
        yaw_param_[3] = json_param_["shoot_table_adjust"]["yaw"]["coef_z2"].Double();
        yaw_param_[4] = json_param_["shoot_table_adjust"]["yaw"]["coef_zd"].Double();
        yaw_param_[5] = json_param_["shoot_table_adjust"]["yaw"]["coef_d2"].Double();
    }
}

void Controller::initializeComponents()
{
    state_manager_ = std::make_unique<TargetStateManager>();
    state_manager_->configure(10, 3, 5);

    target_selector_ = std::make_unique<DistanceBasedSelector>(
        mouse_require_, pic_camera_x_, pic_camera_y_, response_speed_);

    ballistic_calculator_ = std::make_unique<BallisticCalculator>();
    ballistic_calculator_->configure(max_iter_, tol_, bullet_mass_, bullet_diameter_,
                                     bullet_speed_, shoot_table_adjust_, pitch_param_, yaw_param_);

    shoot_decider_ = std::make_unique<ShootDecider>();
    ShootConditions conditions;
    conditions.angle_tolerance_x = tol_deltax_;
    conditions.angle_tolerance_y = tol_deltay_;
    shoot_decider_->configure(conditions);
}

double Controller::applyYawSmoothing(double target_yaw, double current_yaw, bool reset)
{
    target_yaw = current_yaw + std::remainder(target_yaw - current_yaw, 360.0);

    if (!yaw_smoothing_enable_) {
        return target_yaw;
    }

    if (reset || !yaw_filter_initialized_) {
        last_yaw_setpoint_ = target_yaw;
        yaw_filter_initialized_ = true;
        return last_yaw_setpoint_;
    }

    double yaw_delta = std::remainder(target_yaw - last_yaw_setpoint_, 360.0);
    last_yaw_setpoint_ += yaw_smoothing_alpha_ * yaw_delta;
    last_yaw_setpoint_ = current_yaw + std::remainder(last_yaw_setpoint_ - current_yaw, 360.0);
    return last_yaw_setpoint_;
}

ControlResult Controller::control(const ParsedSerialData& parsedData)
{
    if (judgeAimNew(parsedData.aim_request)) {
        aim_armor_id = std::make_pair(-1, -1);
        TargetInfo reset_target;
        state_manager_->setCurrentTarget(reset_target);
    }

    bool aim_center_request = parsedData.aim_request == 3;

    auto predictions = getPredictions(parsedData);
    if (predictions.empty()) {
        WARN("No prediction");
        return createEmptyResult(parsedData);
    }

    auto target = handleTargetSelection(predictions, parsedData);
    if (!target.isValid()) {
        WARN("No valid target");
        return createEmptyResult(parsedData);
    }

    aim_armor_id = std::make_pair(target.car_id, target.armor_id);

    if (aim_center_request) {
        return handleCenterAimRequest(predictions, parsedData, target);
    }

    auto pred_it = std::find_if(predictions.begin(), predictions.end(),
        [&](const auto& pred) { return pred.id == target.car_id; });
    if (pred_it == predictions.end()) {
        WARN("Prediction not found");
        return createEmptyResult(parsedData);
    }

    if (target.armor_id < 0) {
        auto ballistic_result = ballistic_calculator_->calculate(
            pred_it->center.x, pred_it->center.y, pred_it->center.z);
        if (!ballistic_result.success) {
            WARN("calcPitchYaw failed for center");
            return createEmptyResult(parsedData);
        }

        cached_fly_time_ = std::chrono::duration<double>(ballistic_result.time);
        return buildControlResult(parsedData, ballistic_result, nullptr, false);
    }

    auto armor_it = std::find_if(pred_it->armors.begin(), pred_it->armors.end(),
        [&](const auto& armor) { return armor.id == target.armor_id; });
    if (armor_it == pred_it->armors.end()) {
        WARN("Armor not found");
        return createEmptyResult(parsedData);
    }

    auto ballistic_result = ballistic_calculator_->calculate(
        armor_it->center.x, armor_it->center.y, armor_it->center.z);
    if (!ballistic_result.success) {
        WARN("calcPitchYaw failed");
        return createEmptyResult(parsedData);
    }

    bool should_shoot = shoot_decider_->shouldShoot(
        *armor_it, parsedData, ballistic_result.pitch, ballistic_result.yaw);

    cached_fly_time_ = std::chrono::duration<double>(ballistic_result.time);

    return buildControlResult(parsedData, ballistic_result, &(*armor_it), should_shoot);
}

Predictions Controller::getPredictions(const ParsedSerialData& parsedData)
{
    auto predictions = predictFunc(Time::TimeStamp::now() + cached_fly_time_);

    if (!predictions.empty()) {
        updateFlyTime(predictions);
        predictions = predictFunc(Time::TimeStamp::now() + cached_fly_time_);
    }

    return predictions;
}

void Controller::updateFlyTime(const Predictions& predictions)
{
    auto current_target = state_manager_->getCurrentTarget();
    if (!current_target.isValid()) return;

    auto it = std::find_if(predictions.begin(), predictions.end(),
        [&](const auto& prediction) { return prediction.id == current_target.car_id; });
    if (it != predictions.end()) {
        double distance = sqrt(it->center.x * it->center.x + it->center.y * it->center.y);
        if (distance > 0.0) {
            cached_fly_time_ = std::chrono::duration<double>(distance / ballistic_calculator_->getBulletSpeed());
        }
    }
}

TargetInfo Controller::handleTargetSelection(const Predictions& predictions, const ParsedSerialData& parsedData)
{
    state_manager_->updateState(predictions, parsedData.aim_request);
    auto current_target = state_manager_->getCurrentTarget();

    if (!current_target.isValid()) {
        current_target = target_selector_->selectBestCar(predictions, parsedData);
        if (current_target.isValid()) {
            state_manager_->setCurrentTarget(current_target);
        }
    }

    if (current_target.isValid()) {
        current_target = target_selector_->selectBestArmor(predictions, current_target, parsedData);
        state_manager_->setCurrentTarget(current_target);
    }

    return current_target;
}

ControlResult Controller::handleCenterAimRequest(const Predictions& predictions,
                                                 const ParsedSerialData& parsedData,
                                                 const TargetInfo& target)
{
    auto pred_it = std::find_if(predictions.begin(), predictions.end(),
        [&](const auto& pred) { return pred.id == target.car_id; });
    if (pred_it == predictions.end()) {
        return createEmptyResult(parsedData);
    }

    auto ballistic_result = ballistic_calculator_->calculate(
        pred_it->center.x, pred_it->center.y, pred_it->center.z);
    if (!ballistic_result.success) {
        return createEmptyResult(parsedData);
    }

    auto result = buildControlResult(parsedData, ballistic_result, nullptr, false);
    cached_fly_time_ = std::chrono::duration<double>(ballistic_result.time);

    auto future_predictions = predictFunc(Time::TimeStamp::now() + cached_fly_time_ + shoot_delay_);
    auto future_pred_it = std::find_if(future_predictions.begin(), future_predictions.end(),
        [&](const auto& pred) { return pred.id == target.car_id; });

    if (future_pred_it != future_predictions.end()) {
        result.shoot_flag = shoot_decider_->shouldShootAtCenter(
            future_pred_it->armors, parsedData, ballistic_calculator_.get());
    }

    return result;
}

ControlResult Controller::buildControlResult(const ParsedSerialData& parsedData,
                                             const BallisticResult& ballistic,
                                             const Armor* target_armor,
                                             bool should_shoot)
{
    (void)target_armor;
    ControlResult result;
    result.valid = ballistic.success;

    if (ballistic.success) {
        result.pitch_setpoint = ballistic.pitch * 180 / PI;
        result.yaw_setpoint = ballistic.yaw * 180 / PI;
        result.yaw_setpoint = applyYawSmoothing(result.yaw_setpoint, parsedData.yaw_now);
    } else {
        result.pitch_setpoint = parsedData.pitch_now;
        result.yaw_setpoint = parsedData.yaw_now;
        yaw_filter_initialized_ = false;
    }

    result.pitch_actual_want = result.pitch_setpoint;
    result.yaw_actual_want = result.yaw_setpoint;
    result.shoot_flag = should_shoot;

    INFO("aim_pitch:{},aim_yaw:{}", result.pitch_setpoint, result.yaw_setpoint);

    return result;
}

ControlResult Controller::createEmptyResult(const ParsedSerialData& parsedData)
{
    ControlResult result;
    yaw_filter_initialized_ = false;
    result.yaw_setpoint = parsedData.yaw_now;
    result.pitch_setpoint = parsedData.pitch_now;
    result.pitch_actual_want = parsedData.pitch_now;
    result.yaw_actual_want = parsedData.yaw_now;
    result.valid = false;
    result.shoot_flag = false;
    return result;
}
