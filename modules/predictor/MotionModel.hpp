#pragma once
#include "optimizeSolve.hpp"
#include "interfaceType.hpp"

using namespace StateFitting;
class MotionModel : public StateFitting::OptimizeSolve
{
public:
    //x,y,omega,vx,vy,alpha,r1,r2,h1,h2
    ADvector ideal_stateUpdate(const ADvector &state, const double delta_time) override
    {
        ADvector state_update(10);
        state_update[0] = state[0] + state[3] * delta_time;
        state_update[1] = state[1] + state[4] * delta_time;
        state_update[2] = state[2] + state[5] * delta_time;
        state_update[3] = state[3];
        state_update[4] = state[4];
        state_update[5] = state[5];
        state_update[6] = state[6];
        state_update[7] = state[7];
        state_update[8] = state[8];
        state_update[9] = state[9];
        return state_update;
    }
    //p,y,distance,visible
    //measure_id allow 0,1,2,3
    //0,2 ~ r1,h1 1,3 ~ r2,h2
    const double PI = 3.1415926;
    typedef CppAD::AD<double> ADdouble;
    ADvector ideal_measure(const ADvector &state, const int measure_id) override
    {
        ADvector measure(4);
        //1. calc xyz
        ADdouble x = state[0], y = state[1], z = (measure_id % 2 == 0) ? state[8] : state[9];
        ADdouble r = (measure_id % 2 == 0) ? state[6] : state[7];
        ADdouble angle = state[2] + PI / 2 * measure_id;
        ADdouble armor_x = x + r * CppAD::cos(angle);
        ADdouble armor_y = y + r * CppAD::sin(angle);
        //2. change xyz to pyd and calc visible
        measure[2] = CppAD::sqrt(armor_x*armor_x + armor_y*armor_y + z*z);
        measure[0] = CppAD::asin(z/measure[2]);  // pitch
        measure[1] = CppAD::atan2(armor_y, armor_x);   // yaw
        ADdouble dot_product = x * CppAD::cos(angle) + y * CppAD::sin(angle);
        measure[3] = CppAD::CondExpLt(dot_product, ADdouble(0.0), ADdouble(1.0), ADdouble(0.0));
        return measure;
    }

    Prediction getPredictResult(Time::TimeStamp timestamp, int carid)
    {
        Prediction result;
        Dvector nowstate;
        Dvector armormeasure;
        double dt = timestamp2dt(timestamp);
        getCurrentState(nowstate);
        result.id = carid;
        result.x = nowstate[0];
        result.y = nowstate[1];
        result.theta = nowstate[2];
        result.r1 = nowstate[6];
        result.r2 = nowstate[7];
        result.z0 = nowstate[8];
        result.z1 = nowstate[9];
        for (int i = 0; i < 4; i++)
        {
            getPredictMeasure(armormeasure, dt, i);
            double p = armormeasure[0], y = armormeasure[1], d = armormeasure[2];
            result.armors[i].id = i;
            result.armors[i].x = d * cos(p) * cos(y);
            result.armors[i].y = d * cos(p) * sin(y);
            result.armors[i].z = d * sin(p);
            result.armors[i].yaw = nowstate[2] + PI / 2 * i - atan2(result.y, result.x);
            result.armors[i].status = armormeasure[3] == 1 ? Armor::AVAILABLE : Armor::UNSEEN;
        }


        return result;
    }
};