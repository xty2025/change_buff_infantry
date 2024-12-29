#include "controller.hpp"
#include <cmath>
#include <Log/log.hpp>

auto modules::createController() -> std::unique_ptr<modules::Controller>
{
    return std::make_unique<controller::Controller>();
}

using namespace controller;
using namespace aimlog;

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}

ControlResult Controller::control(const ParsedSerialData& parsedData)
{
    Time::TimeStamp now;
    Predictions predictions_for_time = predictFunc(now + flyTime);
    if(predictions_for_time.empty())
    {
        WARN("No prediction");
        return ControlResult();
    }
    bool is_valid_car_id = std::any_of(predictions_for_time.begin(), predictions_for_time.end(), 
                                  [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if(!is_valid_car_id)
    {
        WARN("Invalid car id");
        //重新选择一个距离最近的
        auto min_distance = std::numeric_limits<double>::max();
        for(const auto& prediction : predictions_for_time)
        {
            double distance = sqrt(prediction.x * prediction.x + prediction.y * prediction.y);
            if(distance < min_distance)
            {
                min_distance = distance;
                aim_armor_id.first = prediction.id;
            }
        }
        //需要重新计算flyTime
        flyTime = std::chrono::duration<double>(min_distance / bullet_speed);//粗略计算
        predictions_for_time = predictFunc(now + flyTime);
    }
    
    Prediction aim_prediction;
    for(const auto& prediction : predictions_for_time)
        if(prediction.id == aim_armor_id.first)
        {
            is_valid_car_id = true;
            aim_prediction = prediction;
            break;
        }
    
    if(!is_valid_car_id)
    {
        WARN("New car id invalid");
        return ControlResult();
    }
    bool is_valid_armor_id = std::any_of(aim_prediction.armors.begin(), aim_prediction.armors.end(), 
                            [&](const auto& armor) { return armor.id == aim_armor_id.second; });
    if(!is_valid_armor_id)
    {
        WARN("Invalid armor id");
        //重新选择一个距离最近的
        auto min_distance = std::numeric_limits<double>::max();
        for(const auto& armor : aim_prediction.armors)
        {
            if(armor.status == Armor::AVAILABLE)
            {
                double distance = sqrt(armor.x * armor.x + armor.y * armor.y);
                if(distance < min_distance)
                {
                    min_distance = distance;
                    aim_armor_id.second = armor.id;
                }
            }
        }
        if(min_distance == std::numeric_limits<double>::max())
        {
            WARN("No available armor");
            return ControlResult();
        }
    }
    //计算pitch和yaw
    double pitch = 0.0;
    double yaw = 0.0;
    double time = 0.0;
    if(!calcPitchYaw(pitch, yaw, time, aim_prediction.armors[aim_armor_id.second].x, aim_prediction.armors[aim_armor_id.second].y, aim_prediction.armors[aim_armor_id.second].z))
        return ControlResult();
    flyTime = std::chrono::duration<double>(time);
    ControlResult result;
    result.pitch_setpoint = pitch;
    result.yaw_setpoint = yaw;
    return result;
    //下面是火控逻辑
    // URGENT!!!!!!!!!!!!!!!!
    
}

bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z)
{
    double distance = sqrt(target_x * target_x + target_y * target_y);
    //double theta = atan2(target_z, distance);
    double theta = pitch;
    double delta_z = 0.0;
    // 首先计算空气阻力系数 K
    double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
    for (int i = 0; i < max_iter; i++)
    {
        // 计算炮弹的飞行时间
        double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));

        delta_z = target_z - bullet_speed * sin(theta) * t / cos(theta) + 0.5 * GRAVITY * t * t / cos(theta) / cos(theta);
        // DLOG(INFO) << "delta_y: " << delta_y;

        // 不断更新theta，直到小于某一个阈值
        if (fabs(delta_z) < tol)
        {
            time = t;
            break;
        }

        // 更新角度
        theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + GRAVITY * t * t / (bullet_speed * bullet_speed) * sin(theta) / pow(cos(theta), 3));
    }
    if(fabs(delta_z) > tol)
    {
        //不更新pitch和yaw
        WARN("calcPitchYaw failed");
        return false;//计算失败
    }
    else
    {
        pitch = theta;
        yaw = atan2(target_y, target_x);
        return true;
    }
}
