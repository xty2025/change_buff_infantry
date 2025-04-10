#pragma once
#include "ArmorOneStage.hpp"
#include "YoloDetector.hpp"
#include "type.hpp"
#include <future>


namespace detector
{
    class Detector
    {
    private:
        std::string model_path;
        std::string car_model_path;
        std::unique_ptr<ArmorOneStage> armor_one_stage_inferer;
        std::unique_ptr<YoloDetector> yolo_detector;
    public:
        explicit Detector(const std::string &armor_model_path, const std::string &car_model_path, bool allowGray = true) : 
            model_path(armor_model_path),
            car_model_path(car_model_path)
        {
            armor_one_stage_inferer = std::make_unique<ArmorOneStage>(armor_model_path, allowGray);
            yolo_detector = std::make_unique<YoloDetector>(car_model_path);
        }
        ~Detector() {}

        std::pair<std::vector<Detection>, std::vector<CarDetection>> detect(const cv::Mat &image, const cv::Rect &roi = cv::Rect());
        void setEnemyColor(int flag)//0: red, 1: blue
        {
            armor_one_stage_inferer->setColorFlag(flag);
        }
    };
    std::unique_ptr<Detector> createDetector(const std::string &armor_model_path, const std::string &car_model_path, bool allowGray = true);
} // namespace detector