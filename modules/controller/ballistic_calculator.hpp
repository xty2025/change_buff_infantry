#pragma once
#include <vector>
#include <chrono>

namespace controller {

const double PI = 3.1415926;
const double GRAVITY = 9.794;
const double C_D = 0.42;
const double RHO = 1.169;

struct BallisticResult {
    bool success = false;
    double pitch = 0.0;
    double yaw = 0.0;
    double time = 0.0;
};

class BallisticCalculator {
private:
    int max_iter_;
    double tol_;
    double bullet_mass_;
    double bullet_diameter_;
    double bullet_speed_;
    bool shoot_table_adjust_;
    std::vector<double> pitch_param_;
    std::vector<double> yaw_param_;
    
public:
    BallisticCalculator();
    
    void configure(int max_iter, double tol, double bullet_mass, double bullet_diameter,
                  double bullet_speed, bool shoot_table_adjust,
                  const std::vector<double>& pitch_param,
                  const std::vector<double>& yaw_param);
    
    BallisticResult calculate(double target_x, double target_y, double target_z) const;
    
    double fitPitch(double z_height, double horizontal_distance) const;
    double fitYaw(double z_height, double horizontal_distance) const;
    
    void setBulletSpeed(double speed) { bullet_speed_ = speed; }
    double getBulletSpeed() const { return bullet_speed_; }
    
private:
    bool calcPitchYaw(double& pitch, double& yaw, double& time, 
                     double target_x, double target_y, double target_z) const;
};

} // namespace controller
