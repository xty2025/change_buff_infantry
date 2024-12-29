#pragma once
#include <iostream>
#include <string>
#include <array>
#include <memory>
#include <chrono>
#include <functional>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "TimeStamp/TimeStamp.hpp"

# pragma region driver
struct SerialConfig
{
    std::string portName;
    int baudRate;
};
struct CameraConfig
{
    enum LightSource
    {
        GX_AWB_LAMP_HOUSE_ADAPTIVE = 0, // Auto adaptive mode
        GX_AWB_LAMP_HOUSE_FLUORESCENCE = 1, // Fluorescent lamp mode
        GX_AWB_LAMP_HOUSE_INCANDESCENT = 2, // Incandescent lamp mode
        GX_AWB_LAMP_HOUSE_U30 = 3, //3000K lamp mode
        GX_AWB_LAMP_HOUSE_D50 = 4, //5000K lamp mode
        GX_AWB_LAMP_HOUSE_D65 = 5, //6500K lamp mode
        GX_AWB_LAMP_HOUSE_D75 = 6 //7000K lamp mode
    };
    std::string cameraSN;
    bool autoExposure = false;
    bool autoGain = false;
    bool autoWhiteBalance = false;
    double exposureTime = 5000;
    double autoExposureTimeMin = 3000;
    double autoExposureTimeMax = 8000;
    double gain = 12;
    double autoGainMin = 6;
    double autoGainMax = 18;
    int64_t grayValueMin = 100;
    int64_t grayValueMax = 200;
    float balanceRatioRed = 1.0;
    float balanceRatioBlue = 1.0;
    float balanceRatioGreen = 1.0;
    bool triggerSourceLine2 = true;
    LightSource lightSource = GX_AWB_LAMP_HOUSE_ADAPTIVE;
};
struct RawSerialData
{
    int8_t start_flag;
    float pitch_now;
    float yaw_now;
    float roll_now;
    float actual_bullet_speed;
    uint8_t aim_request;
    uint8_t mode_want;
    uint8_t number_want;
    uint8_t enemy_color; // 0:red 1:blue
    //uint8_t aim_want;
    uint16_t crc16;
    }__attribute__((packed));
struct ParsedSerialData
{
    int8_t start_flag;
    float pitch_now;
    float yaw_now;
    float roll_now;
    float actual_bullet_speed;
    uint8_t aim_request;
    uint8_t mode_want;
    uint8_t number_want;
    uint8_t enemy_color; // 0:red 1:blue

    Time::TimeStamp timestamp;

    ParsedSerialData(const RawSerialData& x) : start_flag(x.start_flag), pitch_now(x.pitch_now), yaw_now(x.yaw_now), roll_now(x.roll_now), 
        actual_bullet_speed(x.actual_bullet_speed), aim_request(x.aim_request), mode_want(x.mode_want), 
        number_want(x.number_want), enemy_color(x.enemy_color), timestamp(Time::TimeStamp()) {};
    ParsedSerialData() {};
};
struct ControlResult
{
    uint8_t shoot_flag;
    float pitch_setpoint;
    float yaw_setpoint;
    bool valid=true;
};
struct RawSerialWriteData
{
    int8_t start_flag;
    uint8_t detect_number;
    uint8_t shoot_flag;
    float pitch_setpoint;
    float yaw_setpoint;
    uint16_t crc16;
    RawSerialWriteData(const ControlResult& x) : start_flag('!'), detect_number(static_cast<int>(x.valid)), shoot_flag(x.shoot_flag), 
        pitch_setpoint(x.pitch_setpoint), yaw_setpoint(x.yaw_setpoint) {}
}__attribute__((packed));
struct TimeImageData
{
    cv::Mat image;
    Time::TimeStamp timestamp;
};
#pragma endregion
#pragma region detector
struct Detection
{
    cv::Rect2f bounding_rect;
    cv::Point2f center;
    int tag_id;
    float score;
    std::vector<cv::Point2f> corners;
};
typedef std::vector<Detection> Detections;
class BBox
{
public:
    BBox() : center(0, 0), rect(0, 0, 0, 0), area(0.0f), confidence(0.0f), color_id(0), tag_id(0)
    {
        for(int i = 0; i < 4; i++)
        {
            corners[i] = cv::Point2f(0, 0); // 初始化 corners 数组
        }
        points.clear(); // 清空 points 向量
    }

    cv::Point2f corners[4];
    std::vector<cv::Point2f> points;
    cv::Point2f center;
    cv::Rect rect;
    float area;
    float confidence;
    int color_id;
    int tag_id;
};
typedef std::vector<BBox> BBoxes;
#pragma endregion

struct ImuData
{
    float pitch;
    float yaw;
    float roll;
    ImuData() {};
    ImuData(const ParsedSerialData& x) : pitch(x.pitch_now), yaw(x.yaw_now), roll(x.roll_now) {};
};

namespace Eigen{
//define Eigen5d
typedef Eigen::Matrix<double, 5, 1> Vector5d;
}

struct XYV
{// the point in camera coordinate
    double x;
    double y;
    bool visible;
    XYV(double x, double y) : x(x), y(y), visible(true) {};
};
typedef std::array<XYV,4> ArmorXYV;
typedef std::vector<ArmorXYV> ArmorXYVs;
typedef std::vector<XYV> XYVs;

struct PYD
{// the point in world coordinate
    double pitch;
    double yaw;
    double distance;
    PYD(double pitch, double yaw, double distance) : pitch(pitch), yaw(yaw), distance(distance) {};
    static inline PYD XYZ2PYD(double x, double y, double z)
    {
        double distance = sqrt(x*x + y*y + z*z);
        double pitch = asin(z/distance);
        double yaw = atan2(y, x);
        return PYD(pitch, yaw, distance);
    }
};
typedef std::vector<PYD> PYDs;

struct Armor
{
    enum armor_status
    {
        NONEXIST,
        UNSEEN,
        AVAILABLE
    };
    double x,y,z;
    double yaw;
    int id;//only permit 4 value: 0,1,2,3.
    armor_status status=NONEXIST;
};
struct Prediction
{
    int id;//a random arranged number
    double x,y,z0,z1,theta;
    double r1,r2;
    //double vx,vy,omega;
    std::array<Armor,4> armors;
};
typedef std::vector<Prediction> Predictions;


struct TrackResult
{
    ArmorXYV armor;
    PYD location;//need to use solver to get the real location
    bool visible=true;
    int armor_id;
    int car_id;
    TrackResult() : armor{XYV(0,0),XYV(0,0),XYV(0,0),XYV(0,0)}, location{0,0,0}, visible(true), armor_id(0), car_id(0) {};
};
typedef std::vector<TrackResult> TrackResults;


