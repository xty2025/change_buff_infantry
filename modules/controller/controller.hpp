#include "type.hpp"
#include "driver/type.hpp"
#include "predictor/type.hpp"
#include "TimeStamp/TimeStamp.hpp"


namespace controller
{
    using predictor::Armor;
    using predictor::Prediction;
    using predictor::Predictions;
    using driver::ParsedSerialData;
    const double PI = 3.1415926;
    const double GRAVITY = 9.794;
    const double C_D = 0.42;
    const double RHO = 1.169;
    class Controller 
    {
    public:
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc);
        ControlResult control(const ParsedSerialData& parsedData);
        chosen_results choose_target(const Predictions& predictions);
        void selectTarget(Predictions& predictions_for_time,std:: pair<int,int>&aim_armor_id, Time::TimeStamp now);
        bool processAimRequest(const ParsedSerialData& parsedData);
        void getTargetPrediction(const Predictions& predictions_for_time, Prediction& aim_prediction, bool& is_valid_car_id);

    private:
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        bool aim_new = false;
        const int waitFrame = 5;
        int aim_want = - waitFrame;
        int old_aim_want = - waitFrame;
        std::pair<int, int> aim_armor_id = {-1, -1};//(car, armor)
        int max_iter = 100;
        double tol = 1e-6;
        double bullet_speed = 24.5;
        double bullet_mass = 3.2e-3;
        double bullet_diameter = 16.8e-3;
        //bool calcPitchYaw(double& pitch, double& yaw, double& time, double target_x, double target_y, double target_z);
        std::chrono::duration<double> shootDelay = 0.1s;
        std::chrono::duration<double> flyTime = 0.0s;
    };

    std::shared_ptr<Controller> createController();    
} // namespace controller
