/*
#pragma once 
#include "timeEKF.hpp"
#include "ceres/ceres.h"
#include "type.hpp"
#include "Log/log.hpp"
#include "MotionModel.hpp"
using namespace predictor;
predictor::VectorX first_state(const predictor::VectorY &measure,const int armor_id){
    VectorX state ;
    state.setZero();
    double dist=measure[2];
    double z_obs=dist*std::sin(measure[0]);
    double x_obs=dist*std::cos(measure[0])*std::cos(measure[1]);
    double y_obs=dist*std::cos(measure[0])*std::sin(measure[1]);
    double tangent=measure[3];
    double theta=M_PI/2-M_PI/2*armor_id-tangent;
    double r=0.3;
    double ang=theta+M_PI/2*armor_id;
       state[0] = x_obs - r * std::cos(ang);  // cx
        state[2] = y_obs + r * std::sin(ang);  // cy
        // 设置朝向和半径
        state[4] = theta;
        state[6] = r;  // r1
        state[7] = r;  // r2
        state[8] = z_obs;  // z1
        state[9] = z_obs;  // z2
        state[10] = state[11] = 0;
        return state;
    }
    ekf::timeEKF::TimeEKF<stateTransFunc,measureFunc,N_X,N_Y>ekf;
    stateTransFunc stateTrans;
    measureFunc mearure;
    bool firstUpdate=true; 
    */
   