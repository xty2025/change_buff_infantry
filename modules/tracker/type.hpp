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
        ArmorXYV armor;
        location::Location location;
        bool visible = true;
        int armor_id;
        int car_id;
        TrackResult(const ArmorXYV& armor, int car_id, int armor_id) : armor(armor), armor_id(armor_id), car_id(car_id) {};
        TrackResult() : armor{XYV(0,0),XYV(0,0),XYV(0,0),XYV(0,0)}, visible(false), armor_id(0), car_id(0) {};
    };
    typedef std::vector<TrackResult> TrackResults;
}
