#pragma once
#include "type.hpp"
#include "driver/type.hpp"
#include "predictor/type.hpp"
#include "solver/type.hpp"
#include "TimeStamp/TimeStamp.hpp"
#include "Param/param.hpp"
#include "target_state.hpp"
#include "target_selector.hpp"
#include "ballistic_calculator.hpp"
#include "shoot_decider.hpp"
#include <memory>
#include <functional>

namespace controller {
    using predictor::Armor;
    using predictor::Prediction;
    using predictor::Predictions;
    using driver::ParsedSerialData;
    using solver::ImuData;

    class Controller {
    public:
        Controller(const param::Param& json_param);
        ~Controller() = default;
        void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc);
        ControlResult control(const ParsedSerialData& parsedData);

    private:
        void readJsonParam();
        void initializeComponents();
        double applyYawSmoothing(double target_yaw, double current_yaw, bool reset = false);

        Predictions getPredictions(const ParsedSerialData& parsedData);
        TargetInfo handleTargetSelection(const Predictions& predictions,
                                         const ParsedSerialData& parsedData);
        ControlResult buildControlResult(const ParsedSerialData& parsedData,
                                         const BallisticResult& ballistic,
                                         const Armor* target_armor,
                                         bool should_shoot);
        ControlResult createEmptyResult(const ParsedSerialData& parsedData);
        void updateFlyTime(const Predictions& predictions);

        ControlResult handleCenterAimRequest(const Predictions& predictions,
                                             const ParsedSerialData& parsedData,
                                             const TargetInfo& target);

        bool judgeAimNew(bool request) { return state_manager_->judgeAimNew(request); }

        std::unique_ptr<TargetStateManager> state_manager_;
        std::unique_ptr<TargetSelector> target_selector_;
        std::unique_ptr<BallisticCalculator> ballistic_calculator_;
        std::unique_ptr<ShootDecider> shoot_decider_;

        param::Param json_param_;
        std::function<Predictions(Time::TimeStamp)> predictFunc;
        std::chrono::duration<double> cached_fly_time_{0.1};

        bool mouse_require_ = false;
        double pic_camera_x_ = 640.0;
        double pic_camera_y_ = 512.0;
        double response_speed_ = 0.5;
        bool yaw_smoothing_enable_ = false;
        double yaw_smoothing_alpha_ = 1.0;
        std::chrono::duration<double> shoot_delay_{0.1};
        bool shoot_table_adjust_ = false;
        std::vector<double> pitch_param_;
        std::vector<double> yaw_param_;
        double bullet_speed_ = 23.0;
        const double min_bullet_speed_ = 20.0;
        const double bullet_speed_alpha_ = 0.5;
        double tol_deltax_ = 0.2;
        double tol_deltay_ = 0.1;
        double armor_yaw_allow_ = 45.0;

        int max_iter_ = 100;
        double tol_ = 1e-6;
        double bullet_mass_ = 3.2e-3;
        double bullet_diameter_ = 16.8e-3;

        std::pair<int, int> aim_armor_id = {-1, -1};
        bool yaw_filter_initialized_ = false;
        double last_yaw_setpoint_ = 0.0;
    };

    std::shared_ptr<Controller> createController(param::Param json_param);
} // namespace controller