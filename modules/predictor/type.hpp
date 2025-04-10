#pragma once
#include <Eigen/Core>
#include <vector>
#include <array>
#include "Location/location.hpp"

namespace predictor
{
    struct Armor {
        enum armor_status {
            NONEXIST,
            UNSEEN,
            AVAILABLE
        };
        location::Location location;
        double gen_cam_x, gen_cam_y, gen_cam_dist;
        double cam_x, cam_y;
        double x, y, z;
        double yaw;
        int id;
        armor_status status = NONEXIST;
    };

    struct Prediction {
        enum state_type {
            STILL,
            LINERAL,
            ROTATE,
            UNKNOW
        };
        location::Location location;
        state_type status = UNKNOW;
        int id;
        double gen_cam_x, gen_cam_y;
        double target_z;
        double x, y, z0, z1, theta;
        double dist;
        double r1, r2;
        std::array<Armor, 4> armors;
    };

    typedef std::vector<Prediction> Predictions;
};