#include "MotionModel.hpp"

namespace predictor{

void MotionModel::initMotionModel()
{

    // 状态变量顺序: x, vx, y, vy, theta, omega, r1, r2, z1, z2
    MatrixXX P = MatrixXX::Zero();
    P << 0.1,  0.02, 0,    0,    0,    0,    0,    0,    0,    0,
         0.02, 0.2,  0,    0,    0,    0,    0,    0,    0,    0,
         0,    0,    0.1,  0.02, 0,    0,    0,    0,    0,    0,
         0,    0,    0.02, 0.2,  0,    0,    0,    0,    0,    0,
         0,    0,    0,    0,    0.1,  0.02, 0,    0,    0,    0,
         0,    0,    0,    0,    0.02, 0.2,  0,    0,    0,    0,
         0,    0,    0,    0,    0,    0,    0.02, 0.005,0,    0,
         0,    0,    0,    0,    0,    0,    0.005,0.02, 0,    0,
         0,    0,    0,    0,    0,    0,    0,    0,    0.1,  0,
         0,    0,    0,    0,    0,    0,    0,    0,    0,    0.1;
    
    MatrixXX Q = MatrixXX::Zero();
    Q << 5e-4, 1e-4, 0,    0,    0,    0,    0,    0,    0,    0,
         1e-4, 1e-2, 0,    0,    0,    0,    0,    0,    0,    0,
         0,    0,    5e-4, 1e-4, 0,    0,    0,    0,    0,    0,
         0,    0,    1e-4, 1e-2, 0,    0,    0,    0,    0,    0,
         0,    0,    0,    0,    1e-4, 2e-5, 0,    0,    0,    0,
         0,    0,    0,    0,    2e-5, 5e-3, 0,    0,    0,    0,
         0,    0,    0,    0,    0,    0,    5e-6, 1e-8, 0,    0,
         0,    0,    0,    0,    0,    0,    1e-8, 5e-6, 0,    0,
         0,    0,    0,    0,    0,    0,    0,    0,    1e-4, 0,
         0,    0,    0,    0,    0,    0,    0,    0,    0,    1e-4;
    // 测量变量顺序: ax, ay, az, tangent, angle_left, angle_right
    MatrixYY R = MatrixYY::Zero();
    R << 0.03,  0.01,  0,     0,     0,     0,
         0.01,  0.03,  0,     0,     0,     0,
         0,     0,     0.03,  0,     0,     0,
         0,     0,     0,     0.03,  0,     0,
         0,     0,     0,     0,     0.04,  0.015,
         0,     0,     0,     0,     0.015, 0.04;
    //R *= 10; 
    ekf.init(P, Q, R);
}

VectorX MotionModel::getPredictResult(const Time::TimeStamp& timestamp)
{
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
        measure_adjust[3] = std::remainder(measure_vec[3] - measure_pred[3], 2 * M_PI) + measure_pred[3];
        measure_adjust[4] = std::remainder(measure_vec[4] - measure_pred[4], 2 * M_PI) + measure_pred[4];
        measure_adjust[5] = std::remainder(measure_vec[5] - measure_pred[5], 2 * M_PI) + measure_pred[5];
        

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