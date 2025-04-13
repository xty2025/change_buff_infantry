#include "controller.hpp"
#include <cmath>
#include <Log/log.hpp>
#include <Param/param.hpp>
#include <Udpsend/udpsend.hpp>

auto controller::createController() -> std::shared_ptr<controller::Controller>
{
    return std::make_unique<controller::Controller>();
}

using namespace controller;
using namespace aimlog;
using param::Param;

void Controller::registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc)
{
    this->predictFunc = predictFunc;
}

const int camera_halfwidth=640;
const int camera_halfheight=512;

// 1. 目标请求处理函数
bool Controller::processAimRequest(const ParsedSerialData& parsedData)
{
    if(parsedData.aim_request == 1)
        aim_want++;
    else
        aim_want--;
        
    if(aim_want < -waitFrame)
        aim_want = -waitFrame;
    if(aim_want > waitFrame)
        aim_want = waitFrame;
        
    bool aim_new_request = false;
    if(old_aim_want < 0 && aim_want >= 0)
        aim_new_request = true;
    else
        aim_new_request = false;
        
    old_aim_want = aim_want;
    return aim_new_request;
}

// 2. 目标选择函数
void Controller::selectTarget(Predictions& predictions_for_time, std::pair<int,int>&aim_armor_id )
{
    WARN("Invalid car id");
    //重新选择一个距离最近的
    auto min_camdistance = std::numeric_limits<double>::max();
    double distance = -1;
    for(const auto& prediction : predictions_for_time)
    {
//            //double distance = sqrt(prediction.x * prediction.x + prediction.y * prediction.y);
//            double distance = prediction.dist;
//            if(distance < min_distance)
//            {
//                min_distance = distance;
//                aim_armor_id.first = prediction.id;
//            }
        //find in armos
        for(const auto& armor:prediction.armors)
        {
            if(armor.status == Armor::AVAILABLE)
            {
                //对armor做判断，确定它的yaw在一定范围内
                if(armor.yaw <10 || armor.yaw > 350)
                {
                    continue;
                }
                double camdistance = (armor.cam_x - camera_halfwidth) * (armor.cam_x - camera_halfwidth) + (armor.cam_y - camera_halfheight) * (armor.cam_y - camera_halfheight);
                if(camdistance < min_camdistance)
                {
                    distance = armor.gen_cam_dist;
                    min_camdistance = camdistance;
                    aim_armor_id.first = prediction.id;
                    aim_armor_id.second = armor.id;
                }
            }
        }
    }
    //需要重新计算flyTime
    if(distance > 0.0)
    {
        flyTime = std::chrono::duration<double>(distance / bullet_speed);//粗略计算
        predictions_for_time = predictFunc(now + flyTime);
    }

}

// 3. 获取目标预测信息
void Controller::getTargetPrediction(const Predictions& predictions_for_time, Prediction& aim_prediction, bool& is_valid_car_id)
{
    for(const auto& prediction : predictions_for_time)
    if(prediction.id == aim_armor_id.first)
    {
        is_valid_car_id = true;
        aim_prediction = prediction;
        break;
    }
}



// 重构后的主控制函数
ControlResult Controller::control(const ParsedSerialData& parsedData)
{
    Param param("../config.json");
    param = param[param["car_name"].String()];
    
    // 处理瞄准请求
    bool aim_new_request = processAimRequest(parsedData);
    
    INFO("now yaw:{},now pitch:{}", parsedData.yaw_now, parsedData.pitch_now);
    ControlResult result;
    result.yaw_setpoint = parsedData.yaw_now;
    result.pitch_setpoint = parsedData.pitch_now;
    result.valid = false;
    result.shoot_flag = false;
    
    if(aim_new_request)
        aim_armor_id = std::make_pair(-1, -1);
    else if(param["mouse_require"].Bool() && (aim_want < 0))
    {
        WARN("MOUSE REQUIRE");
        return result;
    }
    
    // 获取目标预测
    Time::TimeStamp now;
    Predictions predictions_for_time = predictFunc(now + flyTime);
    if(predictions_for_time.empty())
    {
        WARN("No prediction");
        return result;
    }
    bool is_valid_car_id = std::any_of(predictions_for_time.begin(), predictions_for_time.end(), 
                                  [&](const auto& prediction) { return prediction.id == aim_armor_id.first; });
    
    // 选择目标
    if(!is_valid_car_id)
    {
        selectTarget(predictions_for_time, aim_armor_id);
    }
    
    // 获取目标预测信息
    Prediction aim_prediction;
    getTargetPrediction(predictions_for_time, aim_prediction, is_valid_car_id);

    if(!is_valid_car_id)
    {
        WARN("New car id invalid");
        return result;
    }
    
    double high_offset = 0.0;
    high_offset = param["high_offset"].Double();
            result.shoot_flag = false;
    if(aim_prediction.status == Prediction::UNKNOW)
    {
        WARN("No armor");
        return result;
    }
    else if (aim_prediction.status == Prediction::ROTATE)
    {
        double pitch = 0.0;
        double yaw = 0.0;
        double time = 0.0;
        if(!calcPitchYaw(pitch, yaw, time, aim_prediction.x, aim_prediction.y, aim_prediction.target_z + high_offset))
            return result;
        flyTime = std::chrono::duration<double>(time);
        //rad to degree
        result.pitch_setpoint = pitch * 180 / PI + param["pitch_offset"].Double();
        result.yaw_setpoint = yaw * 180 / PI + param["yaw_offset"].Double();
        result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
        result.valid = true;
        INFO("aim_pitch:{},aim_yaw:{}",result.pitch_setpoint,result.yaw_setpoint);
        double tol_pitch = param["shoot_pitch_tol"].Double();
        double tol_yaw = param["shoot_yaw_tol"].Double();

        if((std::abs(result.pitch_setpoint - parsedData.pitch_now) > tol_pitch) ||( std::abs(result.yaw_setpoint - parsedData.yaw_now) > tol_yaw))
        {
            WARN("pitch or yaw not in tolerance");
        }
        else
        {
            double r_tol_pitch = param["rotate_shoot_pitch_tol"].Double();
            double r_tol_yaw = param["rotate_shoot_yaw_tol"].Double();
            //judge whether to shoot
            //check all armors which are available
            //and if one available and p&y in tolerance, then shoot
            for(const auto& armor : aim_prediction.armors)
            {
                INFO("statusnew:{}",(int)armor.status);
                if(armor.status == Armor::AVAILABLE)
                {
                    double pitch = 0.0;
                    double yaw = 0.0;
                    double time = 0.0;
                    //exit(-1);
                    if(!calcPitchYaw(pitch, yaw, time, armor.x, armor.y, armor.z + high_offset))
                        continue;
                    //rad to degree
                    pitch = pitch * 180 / PI + param["pitch_offset"].Double();
                    yaw = yaw * 180 / PI + param["yaw_offset"].Double();
                    yaw = parsedData.yaw_now + std::remainder(yaw - parsedData.yaw_now, 360.0);
                    INFO(" asbyyy:{},yyy:{}",yaw,parsedData.yaw_now);
                    //exit(-1);
                    if((std::abs(pitch - parsedData.pitch_now) < r_tol_pitch) &&( std::abs(yaw - parsedData.yaw_now) < r_tol_yaw))
                    {
                        result.shoot_flag = true;
                        break;
                    }
                }
            }
            if(!result.shoot_flag)ERROR("cancel shoot");
            else INFO("not cancel");
            INFO("a");
        }
    }
    else
    {
        bool is_valid_armor_id = std::any_of(aim_prediction.armors.begin(), aim_prediction.armors.end(), 
        [&](const auto& armor) { return (armor.id == aim_armor_id.second) && (armor.status == Armor::AVAILABLE); });
        if(!is_valid_armor_id)
        {
            WARN("Invalid armor id");
            //重新选择一个距离最近的
            auto min_camdistance = std::numeric_limits<double>::max();
            for(const auto& armor : aim_prediction.armors)
            {
                if(armor.status == Armor::AVAILABLE)
                {
                    //double distance = sqrt(armor.x * armor.x + armor.y * armor.y);
                    double camdistance = (armor.cam_x - camera_halfwidth) * (armor.cam_x - camera_halfwidth) + (armor.cam_y - camera_halfheight) * (armor.cam_y - camera_halfheight);
                    if(camdistance < min_camdistance)
                    {
                        min_camdistance = camdistance;
                        aim_armor_id.second = armor.id;
                    }
                }
            }
            if(min_camdistance == std::numeric_limits<double>::max())
            {
                WARN("No available armor");
                return result;
            }
        }
        //计算pitch和yaw
        double pitch = 0.0;
        double yaw = 0.0;
        double time = 0.0;
        if(!calcPitchYaw(pitch, yaw, time, aim_prediction.armors[aim_armor_id.second].x, aim_prediction.armors[aim_armor_id.second].y, aim_prediction.armors[aim_armor_id.second].z + high_offset))
            return result;
        // if(!calcPitchYaw(pitch, yaw, time, aim_prediction.armors[aim_armor_id.second].x, aim_prediction.armors[aim_armor_id.second].y, aim_prediction.armors[aim_armor_id.second].z ))
        //     return result;
        flyTime = std::chrono::duration<double>(time);

        //rad to degree
        result.pitch_setpoint = pitch * 180 / PI + param["pitch_offset"].Double();
        result.yaw_setpoint = yaw * 180 / PI + param["yaw_offset"].Double();
        result.yaw_setpoint = parsedData.yaw_now + std::remainder(result.yaw_setpoint - parsedData.yaw_now, 360.0);
        result.valid = true;
        INFO("aim_pitch:{},aim_yaw:{}",result.pitch_setpoint,result.yaw_setpoint);
        double tol_pitch = param["shoot_pitch_tol"].Double();
        double tol_yaw = param["shoot_yaw_tol"].Double();
        if(std::abs(result.pitch_setpoint - parsedData.pitch_now) > tol_pitch || std::abs(result.yaw_setpoint - parsedData.yaw_now) > tol_yaw)
        {
            WARN("pitch or yaw not in tolerance");
        }
        else
        {
            result.shoot_flag = true;
        }
        //UdpSend::sendData((float)result.yaw_setpoint);
    }
    return result;
}



bool Controller::calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z)
{

Param param("../config.json");

    param = param[param["car_name"].String()];
    double twohigh = param["2m_highoffset"].Double();
    double fivehigh = param["5m_highoffset"].Double();
    double finalhigh = (std::sqrt(target_x * target_x + target_y * target_y) - 2.0)/3.0 * (fivehigh - twohigh) + twohigh;
    target_z += finalhigh;
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
// 损失函数
  
