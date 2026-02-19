#include "MotionModel.hpp"
#include <algorithm>
//预测状态12维度
//观测7维度
//计算卡尔曼滤波器的流程
namespace predictor{
    //inline主要减小在不同类之间反复调用的开销
    inline Eigen::Matrix3d build_Q_PVA(double q_jerk_base, double dt) {
        Eigen::Matrix3d Q_sub;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        Q_sub << q_jerk_base * dt5 / 20.0, q_jerk_base * dt4 / 8.0,  q_jerk_base * dt3 / 6.0,
                q_jerk_base * dt4 / 8.0,  q_jerk_base * dt3 / 3.0,  q_jerk_base * dt2 / 2.0,
                q_jerk_base * dt3 / 6.0,  q_jerk_base * dt2 / 2.0,  q_jerk_base * dt;
        return Q_sub;
    }

// Helper to build a 2x2 Q sub-matrix for [angle, angular_velocity]
// based on continuous white noise in angular acceleration (spectral density q_ang_accel)
    inline Eigen::Matrix2d build_Q_AngleVel(double q_ang_accel_base, double dt) {
        Eigen::Matrix2d Q_sub;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;

        Q_sub << q_ang_accel_base * dt3 / 3.0, q_ang_accel_base * dt2 / 2.0,
                q_ang_accel_base * dt2 / 2.0, q_ang_accel_base * dt;
        return Q_sub;
    }

void MotionModel::initMotionModel()
{
//    MatrixXX P = MatrixXX::Zero();
//    P << 0.1,  0.02, 0,    0,    0,    0,    0,    0,    0,    0,
//            0.02, 0.2,  0,    0,    0,    0,    0,    0,    0,    0,
//            0,    0,    0.1,  0.02, 0,    0,    0,    0,    0,    0,
//            0,    0,    0.02, 0.2,  0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    0.1,  0.02, 0,    0,    0,    0,
//            0,    0,    0,    0,    0.02, 0.2,  0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    0.02, 0.005,0,    0,
//            0,    0,    0,    0,    0,    0,    0.005,0.02, 0,    0,
//            0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,
//            0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1;
//
//    MatrixXX Q = MatrixXX::Zero();
//    Q << 5e-4, 1e-4, 0,    0,    0,    0,    0,    0,    0,    0,
//            1e-4, 1e-2, 0,    0,    0,    0,    0,    0,    0,    0,
//            0,    0,    5e-4, 1e-4, 0,    0,    0,    0,    0,    0,
//            0,    0,    1e-4, 1e-2, 0,    0,    0,    0,    0,    0,
//            0,    0,    0,    0,    1e-3, 2e-4, 0,    0,    0,    0,
//            0,    0,    0,    0,    2e-4, 5e-2, 0,    0,    0,    0,
//            0,    0,    0,    0,    0,    0,    5e-6, 1e-8, 0,    0,
//            0,    0,    0,    0,    0,    0,    1e-8, 5e-6, 0,    0,
//            0,    0,    0,    0,    0,    0,    0,    0,    1e-5, 0,
//            0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-5;
//    // 测量变量顺序: ax, ay, az, tangent, angle_left, angle_right
//    MatrixYY R = MatrixYY::Zero();
//    R << 0.03,  0.01,  0,     0,     0,     0,
//            0.01,  0.03,  0,     0,     0,     0,
//            0,     0,     0.03,  0,     0,     0,
//            0,     0,     0,     0.1,   0,     0,
//            0,     0,     0,     0,     0.0004,  0.00015,
//            0,     0,     0,     0,     0.00015, 0.0004;
//    //R *= 10;
//    R*=0.01;
//    Q*=10;
//    ekf.init(P, Q, R);
    // 状态变量顺序: x, vx, y, vy, theta, omega, r1, r2, z1, z2, ax, ay
    MatrixXX P = MatrixXX::Zero();
    P << 0.1,   0.02,  0,     0,     0,    0,     0,     0,     0,    0,   0.004, 0,
            0.02,  0.2,   0,     0,     0,    0,     0,     0,     0,    0,   0.02,  0,
            0,     0,     0.1,   0.02,  0,    0,     0,     0,     0,    0,   0,     0.004,
            0,     0,     0.02,  0.2,   0,    0,     0,     0,     0,    0,   0,     0.02,
            0,     0,     0,     0,     0.1,  0.02,  0,     0,     0,    0,   0,     0,
            0,     0,     0,     0,     0.02, 0.2,   0,     0,     0,    0,   0,     0,
            0,     0,     0,     0,     0,    0,     0.02,  0.005, 0,    0,   0,     0,
            0,     0,     0,     0,     0,    0,     0.005, 0.02,  0,    0,   0,     0,
            0,     0,     0,     0,     0,    0,     0,     0,     0.1,  0,   0,     0,
            0,     0,     0,     0,     0,    0,     0,     0,     0,    0.1, 0,     0,
            0.004, 0.02,  0,     0,     0,    0,     0,     0,     0,    0,   0.4,   0,
            0,     0,     0.004, 0.02,  0,    0,     0,     0,     0,    0,   0,     0.4;

    MatrixXX Q_val = MatrixXX::Zero();
//    Q << 5e-4, 1e-4, 0,    0,    0,    0,    0,    0,    0,    0,
//         1e-4, 1e-2, 0,    0,    0,    0,    0,    0,    0,    0,
//         0,    0,    5e-4, 1e-4, 0,    0,    0,    0,    0,    0,
//         0,    0,    1e-4, 1e-2, 0,    0,    0,    0,    0,    0,
//         0,    0,    0,    0,    1e-4, 2e-5, 0,    0,    0,    0,
//         0,    0,    0,    0,    2e-5, 5e-3, 0,    0,    0,    0,
//         0,    0,    0,    0,    0,    0,    5e-6, 1e-8, 0,    0,
//         0,    0,    0,    0,    0,    0,    1e-8, 5e-6, 0,    0,
//         0,    0,    0,    0,    0,    0,    0,    0,    1e-4, 0,
//         0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-4;
    //      Ideal Q for one-dimension
    //      Q = σ_a_drift^2 * [[dt^4/20, dt^3/8, dt^2/6 ], [dt^3/8, dt^2/3, dt/2 ], [dt^2/6, dt/2, 1 ]]
    //      x,     vx,    y,     vy,    theta, omega, r1,    r2,    z1,    z2,    ax,    ay
//    const double sigma_x = 0.05; // 5cm
//    const double sigma_y = sigma_x;
//    double sx = sigma_x * sigma_x; // 0.0025
//    double sy = sigma_y * sigma_y; // 0.0025
//    const double est_dt = 0.015;
//
//    Q << 5e-4,  1e-4,  0,     0,     0,     0,     0,     0,     0,     0,     0,     0,      // x
//            1e-4,  1e-2,  0,     0,     0,     0,     0,     0,     0,     0,     2e-3,  0,      // vx
//            0,     0,     5e-4,  1e-4,  0,     0,     0,     0,     0,     0,     0,     0,      // y
//            0,     0,     1e-4,  1e-2,  0,     0,     0,     0,     0,     0,     0,     2e-3,   // vy
//            0,     0,     0,     0,     1e-3,  2e-4,  0,     0,     0,     0,     0,     0,      // theta
//            0,     0,     0,     0,     2e-4,  5e-2,  0,     0,     0,     0,     0,     0,      // omega
//            0,     0,     0,     0,     0,     0,     5e-6,  1e-8,  0,     0,     0,     0,      // r1
//            0,     0,     0,     0,     0,     0,     1e-8,  5e-6,  0,     0,     0,     0,      // r2
//            0,     0,     0,     0,     0,     0,     0,     0,     1e-5,  0,     0,     0,      // z1
//            0,     0,     0,     0,     0,     0,     0,     0,     0,     1e-5,  0,     0,      // z2
//            0,     2e-3,  0,     0,     0,     0,     0,     0,     0,     0,     2e-1,  0,      // ax
//            0,     0,     0,     2e-3,  0,     0,     0,     0,     0,     0,     0,     2e-1;   // ay
    double q_ax_jerk_base_ = 3;     // m^2/s^5
    double q_ay_jerk_base_ = 3;     // m^2/s^5
    double q_omega_accel_base_ = 0.2; // rad^2/s^3
    double q_r1_drift_base_ = 1e-5;   // m^2/s
    double q_r2_drift_base_ = 1e-5;   // m^2/s
    double q_z1_drift_base_ = 1e-4;   // m^2/s
    double q_z2_drift_base_ = 1e-4;   // m^2/s
    double dt = 0.015; // time step in seconds
    // X, VX, AX (indices 0, 1, 10)
    Eigen::Matrix3d Q_xva = build_Q_PVA(q_ax_jerk_base_, dt);
    Q_val(0,0)   = Q_xva(0,0); Q_val(0,1)   = Q_xva(0,1); Q_val(0,10)  = Q_xva(0,2);
    Q_val(1,0)   = Q_xva(1,0); Q_val(1,1)   = Q_xva(1,1); Q_val(1,10)  = Q_xva(1,2);
    Q_val(10,0)  = Q_xva(2,0); Q_val(10,1)  = Q_xva(2,1); Q_val(10,10) = Q_xva(2,2);

    // Y, VY, AY (indices 2, 3, 11)
    Eigen::Matrix3d Q_yva = build_Q_PVA(q_ay_jerk_base_, dt);
    Q_val(2,2)   = Q_yva(0,0); Q_val(2,3)   = Q_yva(0,1); Q_val(2,11)  = Q_yva(0,2);
    Q_val(3,2)   = Q_yva(1,0); Q_val(3,3)   = Q_yva(1,1); Q_val(3,11)  = Q_yva(1,2);
    Q_val(11,2)  = Q_yva(2,0); Q_val(11,3)  = Q_yva(2,1); Q_val(11,11) = Q_yva(2,2);

    // THETA, OMEGA (indices 4, 5)
    Eigen::Matrix2d Q_to = build_Q_AngleVel(q_omega_accel_base_, dt);
    Q_val(4,4) = Q_to(0,0); Q_val(4,5) = Q_to(0,1);
    Q_val(5,4) = Q_to(1,0); Q_val(5,5) = Q_to(1,1);

    // Parameters: r1, r2, z1, z2 (indices 6, 7, 8, 9)
    Q_val(6,6) = q_r1_drift_base_ * dt;
    Q_val(7,7) = q_r2_drift_base_ * dt; 
    Q_val(8,8) = q_z1_drift_base_ * dt;
    Q_val(9,9) = q_z2_drift_base_ * dt;
                // 测量变量顺序: ax, ay, az, tangent, angle_left, angle_right
            // MatrixYY R = MatrixYY::Zero();
            // R << 0.01,  0,     0,     0,     0,     0, 0,
            //      0.,    0.01,  0,     0,     0,     0, 0,
            //      0,     0,     0.1,   0,     0,     0, 0,
            //      0,     0,     0,     0.04,   0,     0, 0,
            //      0,     0,     0,     0,     0.04,  0.015,0,
            //      0,     0,     0,     0,     0.015, 0.04,0,
            //      0,     0,     0,     0,     0,     0,   0.01;

            //xty:


            // 扩展为 10 维观测：
            // armor_pitch, armor_yaw, dist, tangent,
            // yaw_left, yaw_right, yaw_center,
            // pitch_top, pitch_bottom, pitch_center
            MatrixYY R = MatrixYY::Zero();
            R << 0.01,  0,     0,     0,      0,      0,      0,      0,      0,      0,
             0,     0.01,  0,     0,      0,      0,      0,      0,      0,      0,
             0,     0,     0.1,   0,      0,      0,      0,      0,      0,      0,
             0,     0,     0,     0.04,   0,      0,      0,      0,      0,      0,
             0,     0,     0,     0,      0.04,   0.015,  0.01,   0,      0,      0,
             0,     0,     0,     0,      0.015,  0.04,   0.01,   0,      0,      0,
             0,     0,     0,     0,      0.01,   0.01,   0.02,   0,      0,      0,
             0,     0,     0,     0,      0,      0,      0,      0.03,   0.01,   0.015,
             0,     0,     0,     0,      0,      0,      0,      0.01,   0.03,   0.015,
             0,     0,     0,     0,      0,      0,      0,      0.015,  0.015,  0.03;
    //R *= 10;
    R *= 0.1;

    //xty:


    // 保存基线测量噪声，后续误差拟合只做比例缩放，保持结构稳定。
    base_R_ = R;
    yaw_boundary_var_ema_ = std::max(1e-6, (R(4,4) + R(5,5) + R(6,6)) / 3.0);
    pitch_boundary_var_ema_ = std::max(1e-6, (R(7,7) + R(8,8) + R(9,9)) / 3.0);

    ekf.init(P, Q_val, R);
}


//xty:


void MotionModel::fitBoundaryErrorAndUpdateR(const VectorY& residual)
{
    // yaw 边界观测残差能量（left/right/center）
    const double yaw_var_now = (
        residual[4] * residual[4] +
        residual[5] * residual[5] +
        residual[6] * residual[6]
    ) / 3.0;

    // pitch 边界观测残差能量（top/bottom/center）
    const double pitch_var_now = (
        residual[7] * residual[7] +
        residual[8] * residual[8] +
        residual[9] * residual[9]
    ) / 3.0;

    // EMA 拟合（抑制抖动，保留趋势）
    yaw_boundary_var_ema_ = (1.0 - boundary_fit_alpha_) * yaw_boundary_var_ema_ + boundary_fit_alpha_ * yaw_var_now;
    pitch_boundary_var_ema_ = (1.0 - boundary_fit_alpha_) * pitch_boundary_var_ema_ + boundary_fit_alpha_ * pitch_var_now;

    const double yaw_base_var = std::max(1e-6, (base_R_(4,4) + base_R_(5,5) + base_R_(6,6)) / 3.0);
    const double pitch_base_var = std::max(1e-6, (base_R_(7,7) + base_R_(8,8) + base_R_(9,9)) / 3.0);

    const double yaw_scale = std::clamp(yaw_boundary_var_ema_ / yaw_base_var, boundary_scale_min_, boundary_scale_max_);
    const double pitch_scale = std::clamp(pitch_boundary_var_ema_ / pitch_base_var, boundary_scale_min_, boundary_scale_max_);

    for (int i = 4; i <= 6; ++i) {
        for (int j = 4; j <= 6; ++j) {
            ekf.ekf.R(i, j) = base_R_(i, j) * yaw_scale;
        }
    }
    for (int i = 7; i <= 9; ++i) {
        for (int j = 7; j <= 9; ++j) {
            ekf.ekf.R(i, j) = base_R_(i, j) * pitch_scale;
        }
    }

    INFO("boundary fit scale yaw={}, pitch={}", yaw_scale, pitch_scale);
}

VectorX MotionModel::getPredictResult(const Time::TimeStamp& timestamp)
{
    // INFO("predictResult: {}", ekf.predict(timestamp)[0]);
    // INFO("predictResult: {}", ekf.predict(timestamp)[1]);
    // INFO("predictResult: {}", ekf.predict(timestamp)[2]);
    // INFO("predictResult: {}", ekf.predict(timestamp)[3]);
    return ekf.predict(timestamp);
}

void MotionModel::Update(const VectorY& measure_vec, const Time::TimeStamp& timestamp, int armor_id)
{
    if(firstUpdate)
    {
        initMotionModel();
        firstUpdate = false;
        VectorX first_state = first_state_estimate(measure_vec, armor_id);
        ekf.setX(first_state);
        ekf.setTimeStamp(timestamp);
    }
    else
    {   
        
        VectorY measure_pred;
        measure.setMode(false);
        measure.setId(armor_id);
        measure(ekf.predict(timestamp).data(), measure_pred.data());
        VectorY measure_adjust = measure_vec;

        measure_adjust[0] = std::remainder(measure_vec[0] - measure_pred[0], 2 * M_PI) + measure_pred[0];
        measure_adjust[1] = std::remainder(measure_vec[1] - measure_pred[1], 2 * M_PI) + measure_pred[1];

        measure_adjust[3] = std::remainder(measure_vec[3] - measure_pred[3], 2 * M_PI) + measure_pred[3];
        measure_adjust[4] = std::remainder(measure_vec[4] - measure_pred[4], 2 * M_PI) + measure_pred[4];
        measure_adjust[5] = std::remainder(measure_vec[5] - measure_pred[5], 2 * M_PI) + measure_pred[5];
        measure_adjust[6] = std::remainder(measure_vec[6] - measure_pred[6], 2 * M_PI) + measure_pred[6];

        //xty:


        // 上下边界 pitch 观测角同样做 2π 展开，避免跨 ±π 跳变。
        measure_adjust[7] = std::remainder(measure_vec[7] - measure_pred[7], 2 * M_PI) + measure_pred[7];
        measure_adjust[8] = std::remainder(measure_vec[8] - measure_pred[8], 2 * M_PI) + measure_pred[8];
        measure_adjust[9] = std::remainder(measure_vec[9] - measure_pred[9], 2 * M_PI) + measure_pred[9];


        // auto x = ekf.getX();
        // auto theta = x[4]+ M_PI / 2. * armor_id;
        // auto should_armor_yaw = M_PI/2 - theta;
        //  std::cout<<"should"<<should_armor_yaw<<std::endl;
        //  std::cout<<"measure_should"<<measure_vec[2]<<std::endl;
        // //measure_adjust[2] = std::fmod(measure[2] - should_armor_yaw + M_PI, 2 * M_PI) + should_armor_yaw - M_PI;
        // measure_adjust[2] = std::remainder(measure_vec[2] - should_armor_yaw, 2 * M_PI) + should_armor_yaw;
        // std::cout<<"measure_adjust"<<measure_adjust[2]<<std::endl;
        // std::cout<<"id"<<armor_id<<std::endl;
        //std::cout<<ekf.getP()<<std::endl;
        //std::cout<<"P trace:"<<ekf.getX().trace()<<std::endl;
        //calc residual
        VectorY residual = measure_adjust - measure_pred;
        std::cout<<"residual0:"<<residual[0]<<std::endl;
        std::cout<<"residual1:"<<residual[1]<<std::endl;
        std::cout<<"residual2:"<<residual[2]<<std::endl;
        std::cout<<"residual3:"<<residual[3]<<std::endl;
        std::cout<<"residual4:"<<residual[4]<<std::endl;
        std::cout<<"residual5:"<<residual[5]<<std::endl;

        //xty:


        std::cout<<"residual7:"<<residual[7]<<std::endl;
        std::cout<<"residual8:"<<residual[8]<<std::endl;
        std::cout<<"residual9:"<<residual[9]<<std::endl;

        // yaw + pitch 双轴边界误差拟合，动态调整 R。
        fitBoundaryErrorAndUpdateR(residual);

        // according to residual
        // determine whole car stable & armor stable
//        if(residual[0] > 0.5 || residual[1] > 0.5 || residual[2] > 0.5)
//        {
//            whole_car_stable = false;
//            armor_stable = false;
//        }
//        else if(residual[0] > 0.2 || residual[1] > 0.2 || residual[2] > 0.2)
//        {
//            whole_car_stable = true;
//            armor_stable = false;
//        }
//        else
//        {
//            whole_car_stable = true;
//            armor_stable = true;
//        }
        whole_car_stable = true;
        armor_stable = true;
        
        ekf.setTotalId(id1, id2);
        INFO("set total id: {}, {}", id1, id2);
        VectorX state = ekf.update(measure_adjust, timestamp, armor_id);
        ekf.resetVisibleId();
        if(state[6] < 0.2) state[6] = 0.2;
        if(state[7] < 0.2) state[7] = 0.2;
        ekf.setX(state);
    }
}
}