#pragma once
#include "interfaceType.hpp"
#include "modules.hpp"
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

namespace tracker {

class Tracker : public modules::Tracker {
private:
    std::vector<std::pair<ArmorXYV, int>> armors;
public:
    void merge(const Detections& detections, double threshold) override;// according to NMS, reserve the last one
    std::vector<cv::Rect2i> calcROI(const XYVs& projects, int width, int height, int camera_width, int camera_height) override;
    TrackResults getTrackResult(const std::vector<std::tuple<XYV,int,int>> &old_prediction) override;
    bool isDetected() override;
};;;

//A noneed statement only for reminding you.
using modules::createTracker;

} // namespace tracker
