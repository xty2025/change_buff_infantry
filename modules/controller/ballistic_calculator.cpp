#include "ballistic_calculator.hpp"
#include <cmath>
#include <Log/log.hpp>

using namespace aimlog;

namespace controller {

BallisticCalculator::BallisticCalculator() 
    : max_iter_(100), tol_(1e-6), bullet_mass_(3.2e-3), bullet_diameter_(16.8e-3),
      bullet_speed_(23.0), shoot_table_adjust_(false),
      pitch_param_(6, 0.0), yaw_param_(6, 0.0) {
}

void BallisticCalculator::configure(int max_iter, double tol, double bullet_mass, 
                                  double bullet_diameter, double bullet_speed, 
                                  bool shoot_table_adjust,
                                  const std::vector<double>& pitch_param,
                                  const std::vector<double>& yaw_param) {
    max_iter_ = max_iter;
    tol_ = tol;
    bullet_mass_ = bullet_mass;
    bullet_diameter_ = bullet_diameter;
    bullet_speed_ = bullet_speed;
    shoot_table_adjust_ = shoot_table_adjust;
    pitch_param_ = pitch_param;
    yaw_param_ = yaw_param;
}

BallisticResult BallisticCalculator::calculate(double target_x, double target_y, double target_z) const {
    BallisticResult result;
    result.success = calcPitchYaw(result.pitch, result.yaw, result.time, target_x, target_y, target_z);
    
    // 如果基础弹道计算成功，应用射击表补偿
    if (result.success && shoot_table_adjust_) {
        double horizontal_distance = std::sqrt(target_x * target_x + target_y * target_y);
        
        // 应用射击表补偿（以弧度为单位）
        result.pitch += fitPitch(target_z, horizontal_distance) * PI / 180.0;
        result.yaw += fitYaw(target_z, horizontal_distance) * PI / 180.0;
    }
    
    return result;
}

double BallisticCalculator::fitPitch(double z_height, double horizontal_distance) const {
    if (!shoot_table_adjust_ || pitch_param_.size() < 6) {
        return 0.0;
    }
    
    // Model 3: Full 2nd Order 参数 (PITCH)
    double intercept = pitch_param_[0];
    double coef_z = pitch_param_[1];
    double coef_d = pitch_param_[2];
    double coef_z2 = pitch_param_[3];
    double coef_zd = pitch_param_[4];
    double coef_d2 = pitch_param_[5];

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;
    
    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

double BallisticCalculator::fitYaw(double z_height, double horizontal_distance) const {
    if (!shoot_table_adjust_ || yaw_param_.size() < 6) {
        return 0.0;
    }
    
    // Model 3: Full 2nd Order 参数 (YAW)
    double intercept = yaw_param_[0];
    double coef_z = yaw_param_[1];
    double coef_d = yaw_param_[2];
    double coef_z2 = yaw_param_[3];
    double coef_zd = yaw_param_[4];
    double coef_d2 = yaw_param_[5];

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;

    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

bool BallisticCalculator::calcPitchYaw(double& pitch, double& yaw, double& time, 
                                     double target_x, double target_y, double target_z) const {
    INFO("targetx,y,z: {}, {}, {}", target_x, target_y, target_z);
    double distance = sqrt(target_x * target_x + target_y * target_y);
    double theta = pitch;
    double delta_z = 0.0;
    
    // 首先计算空气阻力系数 K
    double k1 = C_D * RHO * (PI * bullet_diameter_ * bullet_diameter_) / 8 / bullet_mass_;
    
    for (int i = 0; i < max_iter_; i++) {
        // 计算炮弹的飞行时间
        double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed_ * cos(theta));

        delta_z = target_z - bullet_speed_ * sin(theta) * t / cos(theta) + 
                  0.5 * GRAVITY * t * t / cos(theta) / cos(theta);

        // 不断更新theta，直到小于某一个阈值
        if (fabs(delta_z) < tol_) {
            time = t;
            break;
        }

        // 更新角度
        theta -= delta_z / (-(bullet_speed_ * t) / pow(cos(theta), 2) + 
                           GRAVITY * t * t / (bullet_speed_ * bullet_speed_) * 
                           sin(theta) / pow(cos(theta), 3));
    }
    
    if (fabs(delta_z) > tol_) {
        WARN("calcPitchYaw failed");
        return false;
    } else {
        pitch = theta;
        yaw = atan2(target_y, target_x);
        return true;
    }
}

} // namespace controller
