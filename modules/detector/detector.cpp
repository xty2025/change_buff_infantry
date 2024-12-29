#include "detector.hpp"

auto modules::createDetector(const std::string &model_path) -> std::unique_ptr<modules::Detector>
{
    return std::make_unique<detector::Detector>(model_path);
}
namespace detector
{
    std::vector<Detection> Detector::detect(const cv::Mat &image, const cv::Rect &roi)
    {
        BBoxes detection;
        bool use_roi = false;
        if(roi.size() == cv::Size(0,0))
        {
            use_roi = false;
            detection = (*armor_one_stage_inferer)(image);
        }
        else
        {
            use_roi = true;
            cv::Mat roi_image = image(roi);
            detection = (*armor_one_stage_inferer)(roi_image);
        }
        //according to roi, transform the pos in the original image
        std::vector<Detection> detections;
        for(auto &det : detection)
        {
            Detection detection;
            if(use_roi)
            {
                detection.bounding_rect = cv::Rect2f(det.rect.x + roi.x, det.rect.y + roi.y, det.rect.width, det.rect.height);
                detection.center = cv::Point2f(det.center.x + roi.x, det.center.y + roi.y);
                for(auto &corner : det.corners)
                {
                    detection.corners.push_back(cv::Point2f(corner.x + roi.x, corner.y + roi.y));
                }
            }
            else
            {
                detection.bounding_rect = cv::Rect2f(det.rect.x, det.rect.y, det.rect.width, det.rect.height);
                detection.center = cv::Point2f(det.center.x, det.center.y);
                for(auto &corner : det.corners)
                {
                    detection.corners.push_back(cv::Point2f(corner.x, corner.y));
                }
            }
            detection.tag_id = det.tag_id;
            detection.score = det.confidence;
            detections.push_back(detection);
        }
        return detections;
    }
}