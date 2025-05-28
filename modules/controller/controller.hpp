#include "type.hpp"
#include "driver/type.hpp"
#include "predictor/type.hpp"
#include "solver/type.hpp"
#include "TimeStamp/TimeStamp.hpp"
#include "Param/param.hpp"


namespace controller
{
    using predictor::Armor;
    using predictor::Prediction;
    using predictor::Predictions;
    using driver::ParsedSerialData;
    using solver::ImuData;
    const double PI = 3.1415926;
    const double GRAVITY = 9.794;
    const double C_D = 0.42;
    const double RHO = 1.169;
    class Controller 
    {
    public:
        Controller(param::Param& json_param) : json_param(json_param) {readJsonParam();}
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc);
        ControlResult control(const ParsedSerialData& parsedData);
    private:
        void readJsonParam(void);
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        param::Param json_param;
        bool aim_new = false;
        bool aiming = false;
        const int waitFrame = 5;
        int accumulate_aim_request = 0;
        std::pair<int, int> aim_armor_id = {-1, -1};//(car, armor)
        int max_iter = 100;
        double tol = 1e-6;
        double bullet_mass = 3.2e-3;
        double bullet_diameter = 16.8e-3;
        bool judgeAimNew(bool request);
        bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
        std::chrono::duration<double> flyTime = 0.0s;

        double bullet_speed = 23.0;
        const double min_bullet_speed = 20.0;
        const double bullet_speed_alpha = 0.5;

        std::chrono::duration<double> shootDelay = 0.1s;

        double tol_deltax = 0.2;
        double tol_deltay = 0.1;
        double response_speed = 0.5; // 0.5 rad/s
        double armor_yaw_allow = 45.0;
        bool mouse_require = false;
        double pic_camera_x = 640.0; //图传模块中心点在相机坐标系中的坐标
        double pic_camera_y = 512.0; //图传模块中心点在相机坐标系中的坐标
    };

    std::shared_ptr<Controller> createController(param::Param json_param);
} // namespace controller