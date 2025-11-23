#pragma once
#include "TimeStamp/TimeStamp.hpp"
#include <vector>
#include <array>
#include <opencv2/opencv.hpp>
#include "Location/location.hpp"

namespace tracker
{
    typedef std::array<XYV, 4> ArmorXYV;
    struct TrackResult
    {
        ArmorXYV armor;//装甲板的4个点的坐标和速度。
        cv::Rect2f rect;//装甲板的矩形框。
        location::Location location;//装甲板的位置信息。
        double yaw;//装甲板的yaw角度。
        bool visible = true;//装甲板是否可见。
        int armor_id;//装甲板的id。
        int car_id;//车的id。
        TrackResult(const ArmorXYV& armor, int car_id, int armor_id) : armor(armor), armor_id(armor_id), car_id(car_id) {};
        TrackResult() : armor{XYV(0,0),XYV(0,0),XYV(0,0),XYV(0,0)}, visible(false), armor_id(0), car_id(0) {};
    };
    typedef std::vector<TrackResult> TrackResults;
    struct CarTrackResult
    {
        int car_id;
        int car_type;
        cv::Rect2f bounding_rect;
        CarTrackResult(): car_id(-1), car_type(-1), bounding_rect(0, 0, 0, 0) {};
    };
    typedef std::vector<CarTrackResult> CarTrackResults;
    typedef std::pair<TrackResults, CarTrackResults> TrackResultPairs;
}
