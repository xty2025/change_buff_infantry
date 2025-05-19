#include "controller.hpp"
#include <cmath>
#include <Log/log.hpp>
#include <Udpsend/udpsend.hpp>

auto controller::createController(param::Param json_param) -> std::shared_ptr<controller::Controller>
{
    return std::make_unique<controller::Controller>(json_param);
}

using namespace controller;
using namespace aimlog;
using param::Param;

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}
void Controller::readJsonParam()
{
    mouse_require = json_param["mouse_require"].Bool();
    pic_camera_x = json_param["pic_camera_x"].Double();
    pic_camera_y = json_param["pic_camera_y"].Double();
    shootDelay = std::chrono::duration<double>(json_param["shoot_delay"].Double());
}

static bool thetaInRange(double theta_deg, double range_deg)
{
    theta_deg = std::remainder(theta_deg, 360.0);
    if(std::abs(theta_deg) < range_deg)
        return true;
    else
        return false;
} 

bool Controller::judgeAimNew(bool request)
{
    aim_new = false;
    if(((!aiming) && request) || (aiming && (!request)))
    {
        accumulate_aim_request++;
    }
    else
    {
        accumulate_aim_request = 0;
    }
    if(accumulate_aim_request > waitFrame)
    {
        accumulate_aim_request = 0;
        aiming = request;
        if(aiming)
            aim_new = true;
    }
    return aim_new;
}

ControlResult Controller::control(const ParsedSerialData& parsedData)
{
    if(judgeAimNew(parsedData.aim_request))
    {
        aim_armor_id = std::make_pair(-1, -1);
    }

    if(parsedData.actual_bullet_speed > min_bullet_speed)
    {
        bullet_speed = parsedData.actual_bullet_speed * bullet_speed_alpha + (1 - bullet_speed_alpha) * bullet_speed;
    }


    ControlResult result;
    result.yaw_setpoint = parsedData.yaw_now;
    result.pitch_setpoint = parsedData.pitch_now;
    result.pitch_actual_want = parsedData.pitch_now;
    result.yaw_actual_want = parsedData.yaw_now;
    result.valid = false;
    result.shoot_flag = false;

    Predictions predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime + shootDelay);
    if (predictions_for_time.empty())
    {
        WARN("No prediction");
        return result;
    }
    bool is_valid_car_id = std::any_of(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (!is_valid_car_id)
    {
        WARN("Invalid car id");
        //重新选择一个相机距离最近的
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;
        location::Location tmp;
        tmp.imu = ImuData(parsedData);
        for (const auto& prediction : predictions_for_time)
        {
            double distance;
            if(!mouse_require)
                distance = prediction.center.x * prediction.center.x + prediction.center.y * prediction.center.y;
            else
            {
                tmp.xyz_imu = prediction.center;
                CXYD tmp_cxy = tmp.cxy;
                distance = (tmp_cxy.cx - pic_camera_x) * (tmp_cxy.cx - pic_camera_x) + (tmp_cxy.cy - pic_camera_y) * (tmp_cxy.cy - pic_camera_y);
            }
            if (distance < min_distance)
            {
                min_distance = distance;
                aim_armor_id.first = prediction.id;
                aim_armor_id.second = -1;
                found = true;
            }
        }
        if(!found)
        {
            WARN("No available car");
            return result;
        }
    }
    auto it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        WARN("New car id invalid(shouldn't happen)");
        return result;
    }
    //calc new flytime
    double distance = sqrt(it->center.x * it->center.x + it->center.y * it->center.y);
    if (distance > 0.0)
    {
        flyTime = std::chrono::duration<double>(distance / bullet_speed);//粗略计算
        predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime + shootDelay);
    }
    it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    if (it == predictions_for_time.end())
    {
        WARN("New car id invalid after flytime update");
        return result;
    }
    bool is_valid_armor_id = std::any_of(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return (armor.id == aim_armor_id.second) && (armor.status == Armor::AVAILABLE); });
    if (!is_valid_armor_id)
    {
        WARN("Invalid armor id");
        //重新选择一个距离最近的
        double min_distance = std::numeric_limits<double>::max();
        bool found = false;
        location::Location tmp;
        tmp.imu = ImuData(parsedData);
        for (const auto& armor : it->armors)
        {
            if (armor.status == Armor::AVAILABLE)
            {
                double distance;
                if(!mouse_require)
                    distance = armor.center.x * armor.center.x + armor.center.y * armor.center.y;
                else
                {
                    tmp.xyz_imu = armor.center;
                    CXYD tmp_cxy = tmp.cxy;
                    distance = (tmp_cxy.cx - pic_camera_x) * (tmp_cxy.cx - pic_camera_x) + (tmp_cxy.cy - pic_camera_y) * (tmp_cxy.cy - pic_camera_y);
                }
                if (distance < min_distance)
                {
                    min_distance = distance;
                    aim_armor_id.second = armor.id;
                    found = true;
                }
            }
        }
        if (!found)
        {
            WARN("No available armor");
            WARN("Now choose to aim at the car");
            double pitch = 0.0;
            double yaw = 0.0;
            double time = 0.0;
            bool success = calcPitchYaw(pitch, yaw, time, it->center.x + x_offset, it->center.y + y_offset, it->center.z + z_offset);
            if (!success)
            {
                WARN("calcPitchYaw failed");
                return result;
            }
            result.pitch_setpoint = pitch * 180 / PI;
            result.yaw_setpoint = yaw * 180 / PI;
            result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
            result.pitch_actual_want = result.pitch_setpoint;
            result.yaw_actual_want = result.yaw_setpoint;
            result.valid = true;
            result.shoot_flag = false;
            flyTime = std::chrono::duration<double>(time);
            INFO("aim_pitch:{},aim_yaw:{}", result.pitch_setpoint, result.yaw_setpoint);
            return result;
        }
    }
    auto armor_it = std::find_if(it->armors.begin(), it->armors.end(),
        [&](const auto& armor) { return armor.id == aim_armor_id.second; });
    if (armor_it == it->armors.end())
    {
        WARN("Invalid armor id (shoudln't happen)");
        return result;
    }
    //should calc new time
    //but now we just use the old time
    double pitch = 0.0;
    double yaw = 0.0;
    double time = 0.0;
    if (!calcPitchYaw(pitch, yaw, time, armor_it->center.x + x_offset, armor_it->center.y + y_offset, armor_it->center.z + z_offset))
    {
        WARN("calcPitchYaw failed");
        return result;
    }
    result.pitch_setpoint = pitch * 180 / PI + pitch_offset;
    result.yaw_setpoint = yaw * 180 / PI + yaw_offset;
    result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
    result.valid = true;

    double armor_yaw_now = armor_it->yaw * 180 / PI - parsedData.yaw_now;
    if(thetaInRange(armor_yaw_now, armor_yaw_allow)&&it->stable)
    {
        result.shoot_flag = true;
    }
    else
    {
        WARN("armor yaw not in allow range");
        result.shoot_flag = false;
    }

    //calc actual want with no shootDelay
    //i.e. ignore motion time delay
    predictions_for_time = predictFunc(Time::TimeStamp::now() + flyTime);
    auto actual_car_it = std::find_if(predictions_for_time.begin(), predictions_for_time.end(),
        [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    auto actual_armor_it = std::find_if(actual_car_it->armors.begin(), actual_car_it->armors.end(),
        [&](const auto& armor) { return armor.id == aim_armor_id.second; });
    if(!calcPitchYaw(pitch, yaw, time, actual_armor_it->center.x + x_offset, actual_armor_it->center.y + y_offset, actual_armor_it->center.z + z_offset))
    {
        WARN("calcPitchYaw failed");
        result.shoot_flag = false;
        return result;
    }
    else
    {
        result.yaw_actual_want = yaw * 180 / PI + yaw_offset;
        result.pitch_actual_want = pitch * 180 / PI + pitch_offset;
        result.yaw_actual_want = parsedData.yaw_now + std::remainder(result.yaw_actual_want - parsedData.yaw_now, 360.0);
        if(abs(result.yaw_actual_want - parsedData.yaw_now) > tol_yaw||
           abs(result.pitch_actual_want - parsedData.pitch_now) > tol_pitch)
        {
            WARN("not in tolerance, cancel shoot");
            result.shoot_flag = false;
        }
    }
    return result;
}

bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z)
{

// Param param("../config.json");

//     param = param[param["car_name"].String()];
//     double twohigh = param["2m_highoffset"].Double();
//     double fivehigh = param["5m_highoffset"].Double();
//     double finalhigh = (std::sqrt(target_x * target_x + target_y * target_y) - 2.0)/3.0 * (fivehigh - twohigh) + twohigh;
//     target_z += finalhigh;
    INFO("targetx,y,z:{},{},{}",target_x,target_y,target_z);
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
