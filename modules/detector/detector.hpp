#pragma once
#include "modules.hpp"
#include "ArmorOneStage.hpp"

namespace detector
{
    class Detector : public modules::Detector
    {
    private:
        std::string model_path;
        std::unique_ptr<ArmorOneStage> armor_one_stage_inferer;
    public:
        explicit Detector(const std::string &model_path) : model_path(model_path)
        {
            armor_one_stage_inferer = std::make_unique<ArmorOneStage>(model_path);
        }
        ~Detector() {}

        std::vector<Detection> detect(const cv::Mat &image, const cv::Rect &roi) override;
        void setEnemyColor(int flag) override
        {
            armor_one_stage_inferer->setColorFlag(flag);
        }
    };
    //A noneed statement only for reminding you.
    using modules::createDetector;
} // namespace detector