#pragma once
#include "CarPredictorNew.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include "TimeStamp/TimeStamp.hpp"
#include "type.hpp"
#include "tracker/type.hpp"

namespace predictor {
    using tracker::TrackResults;

class Predictor{
private:
    std::mutex car_mutex;
    std::map<int, std::unique_ptr<MotionModel>> cars;
    std::map<int, int> detect_count;//detected then set to 0, not detected then add 1, upper than 10 then delete.
    const int MaxMissFrame = 10;
public:
    std::function<Predictions(Time::TimeStamp)> predictFunc();
    Predictions predict(Time::TimeStamp timestamp);
    void update(const TrackResults& trackResults, const Time::TimeStamp& timestamp);
    bool Stable() const{return false;};
    
};
std::unique_ptr<Predictor> createPredictor();

} // namespace predictor
