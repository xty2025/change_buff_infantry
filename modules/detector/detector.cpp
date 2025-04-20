#include "detector.hpp"
#include "Log/log.hpp"

auto detector::createDetector(const std::string &armor_model_path, const std::string &car_model_path, bool allowGray) -> std::unique_ptr<Detector>
{
    return std::make_unique<detector::Detector>(armor_model_path, car_model_path, allowGray);
}

namespace detector
{
    std::pair<std::vector<Detection>, std::vector<CarDetection>> Detector::detect(const cv::Mat &image, const cv::Rect &roi) {
        if(image.empty()) {
            ERROR("Image is empty");
            return {};
        }
        // if(image.cols != 1280 || image.rows != 720) {
        //     ERROR("Image size is not 1280x720");
        //     INFO("Image size: {}x{}", image.cols, image.rows);
        //     return {};
        // }
        // 使用异步方式启动ArmorOneStage检测
        auto armor_future = std::async(std::launch::async, [&]() {
            bool use_roi = (roi.size() != cv::Size(0,0));
            return (*armor_one_stage_inferer)(use_roi ? image(roi) : image);
        });
        
        // 使用异步方式启动车辆检测
        auto car_future = std::async(std::launch::async, [&]() {
            return yolo_detector->infer(image);
        });
        
        // 等待ArmorOneStage检测完成并处理结果
        BBoxes detection = armor_future.get();
        //BBoxes detection = BBoxes();
        std::vector<Detection> detections;
        detections.reserve(detection.size());
        
        const int offset_x = (roi.size() != cv::Size(0,0)) ? roi.x : 0;
        const int offset_y = (roi.size() != cv::Size(0,0)) ? roi.y : 0;
        
        for(auto &det : detection) {
            detections.emplace_back();
            auto &result = detections.back();
            
            // 设置边界框和中心
            result.bounding_rect = cv::Rect2f(
                det.rect.x + offset_x,
                det.rect.y + offset_y,
                det.rect.width,
                det.rect.height
            );
            result.center = cv::Point2f(
                det.center.x + offset_x,
                det.center.y + offset_y
            );
            
            // 高效处理角点
            result.corners.reserve(4);
            for(auto &corner : det.corners) {
                result.corners.push_back(cv::Point2f(
                    corner.x + offset_x,
                    corner.y + offset_y
                ));
            }
            
            // 复制其他属性
            result.tag_id = det.tag_id;
            result.score = det.confidence;
            result.isGray = det.color_id/2 == 2;
        }
        
        // 等待车辆检测完成并处理结果
        std::vector<YoloDetection> yolo_detections = car_future.get();
        std::vector<CarDetection> car_detections;
        car_detections.reserve(yolo_detections.size());
        
        for(auto &det : yolo_detections) {
            car_detections.emplace_back();
            auto &car_detection = car_detections.back();
            
            car_detection.bounding_rect = cv::Rect2f(det.rect);
            car_detection.center = det.center;
            car_detection.tag_id = det.class_id;
            car_detection.score = det.confidence;
            INFO("yolo car detection: {} {} {} {}", det.rect.x, det.rect.y, det.rect.width, det.rect.height);
            INFO("yolo confidence: {}", det.confidence);

            // 在image上绘制检测框
            cv::rectangle(image, det.rect, cv::Scalar(0, 255, 0), 2);
            std::string label = "Class: " + std::to_string(det.class_id) + " " + std::to_string(det.confidence).substr(0, 4);
            int baseline = 0;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
        }
        INFO("find {} yolo detections", car_detections.size());
        INFO("find {} armor detections", detections.size());
        return {detections, car_detections};
    }
}