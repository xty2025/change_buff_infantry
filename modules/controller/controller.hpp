#include "modules.hpp"
#include "cppad/cppad.hpp"

namespace controller
{
    const double PI = 3.1415926;
    const double GRAVITY = 9.794;
    const double C_D = 0.42;
    const double RHO = 1.169;
    class Controller : public modules::Controller
    {
    public:
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc) override;
        ControlResult control(const ParsedSerialData& parsedData) override;
    private:
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        std::pair<int, int> aim_armor_id = {-1, -1};//(car, armor)
        int max_iter = 100;
        double tol = 1e-6;
        double bullet_speed = 15.0;
        double bullet_mass = 3.2e-3;
        double bullet_diameter = 16.8e-3;
        bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
        std::chrono::duration<double> shootDelay = 0.2s;
        std::chrono::duration<double> flyTime = 0.0s;
    };

    //A noneed statement only for reminding you.
    using modules::createController;
} // namespace controller