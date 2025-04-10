#include "predictor.hpp"
#include "Udpsend/udpsend.hpp"
#include <Log/log.hpp>
#include "Param/param.hpp"

auto predictor::createPredictor() -> std::unique_ptr<Predictor> {
    return std::make_unique<predictor::Predictor>();
}

namespace predictor {
    std::function<Predictions(Time::TimeStamp)> Predictor::predictFunc()
    {
        return std::function<Predictions(Time::TimeStamp)>([&](Time::TimeStamp timestamp) { return predict(timestamp); });
    }
    Predictions Predictor::predict(Time::TimeStamp timestamp)
    {
        Predictions predictions;
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto& [carid, car] : cars)
        {
            if(car->Stable())
                predictions.push_back(car->getPredictResult(timestamp, carid));
            else
                WARN("Car {} is not stable", carid);
        }
        return predictions;
    }
    void Predictor::update(const TrackResults& trackResults, const Time::TimeStamp& timestamp)
    {
        std::map<int, std::vector<std::pair<Eigen::VectorXd, int>>> measures;
        for(const auto& trackResult : trackResults)
        {
            Eigen::VectorXd measure(5);
            // measure[0] = trackResult.extra_center.x;
            // measure[1] = trackResult.extra_center.y;
            // measure[2] = trackResult.location.distance;
            // measure[3] = trackResult.theta;
            // measure[4] = trackResult.ratio;
            measure[0] = 0;
            measure[1] = 0;
            measure[2] = 0;
            measure[3] = 0;
            measure[4] = 0;
            measures[trackResult.car_id].push_back(std::make_pair(measure, trackResult.armor_id));
            //INFO("track: car_id: {}, armor_id: {}, x: {}, y: {}, distance: {}", trackResult.car_id, trackResult.armor_id, measure[0], measure[1], measure[2]);
            detect_count[trackResult.car_id] = 0;
        }
        for(auto& [carid, measure] : measures)
        {
            std::lock_guard<std::mutex> lock(car_mutex);
            if(cars.find(carid) == cars.end())
            {
                cars[carid] = std::make_unique<MotionModel>();
                cars[carid]->initMotionModel();
            }
            cars[carid]->Update(measure, timestamp);
        }
        // for(const auto& trackResult : trackResults)
        // {
        //     if(cars.find(trackResult.car_id) == cars.end())
        //     {
        //         std::lock_guard<std::mutex> lock(car_mutex);
        //         cars[trackResult.car_id] = std::make_unique<MotionModel>();
        //         cars[trackResult.car_id]->initMotionModel();
        //     }
        //     Eigen::VectorXd measure(3);
        //     measure[0] = trackResult.extra_center.x;
        //     measure[1] = trackResult.extra_center.y;
        //     measure[2] = trackResult.location.distance;
        //     cars[trackResult.car_id]->Update(measure, timestamp, trackResult.armor_id);

        //     detect_count[trackResult.car_id] = 0;
        // }
        std::lock_guard<std::mutex> lock(car_mutex);
        for(auto it = detect_count.begin(); it != detect_count.end();)
        {
            if(it->second > MaxMissFrame)
            {
                cars.erase(it->first);
                it = detect_count.erase(it);
            }
            else
            {
                it->second++;
                ++it;
            }
        }
    }

} // namespace predictor
