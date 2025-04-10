#include "tracker.hpp"
#include "TrackerMatcher.hpp"
#include "Log/log.hpp"

auto tracker::createTracker() -> std::unique_ptr<Tracker> {
    return std::make_unique<tracker::Tracker>();
}

namespace tracker {

// This func will regard the last one as the final result.
void Tracker::merge(const Detections& detections, double threshold) 
{
    //clear method will put into GetTrackResult method
    //armors.clear();
    for(const auto& detection : detections) {
        ArmorXYV new_armor = {
            XYV(detection.corners[0].x, detection.corners[0].y),
            XYV(detection.corners[1].x, detection.corners[1].y),
            XYV(detection.corners[2].x, detection.corners[2].y),
            XYV(detection.corners[3].x, detection.corners[3].y)
        };
        
        auto dist = [](const XYV& p1, const XYV& p2) {
            return std::hypot(p1.x - p2.x, p1.y - p2.y);
        };
        
        auto it = std::remove_if(armors.begin(), armors.end(), 
            [&](const auto& existing_armor) {
                float avg_dist = 0;
                for(int i = 0; i < 4; i++) {
                    avg_dist += dist(existing_armor.first[i], new_armor[i]);
                }
                return avg_dist/4 < threshold;
            });
        auto it2 = std::remove_if(armors_gray.begin(), armors_gray.end(), 
            [&](const auto& existing_armor) {
                float avg_dist = 0;
                for(int i = 0; i < 4; i++) {
                    avg_dist += dist(existing_armor.first[i], new_armor[i]);
                }
                return avg_dist/4 < threshold;
            });
        armors.erase(it, armors.end());
        armors_gray.erase(it2, armors_gray.end());
        if(detection.isGray)
            armors_gray.push_back(std::make_pair(new_armor, detection.tag_id));
        else
        {
            armors.push_back(std::make_pair(new_armor, detection.tag_id));
            dead[detection.tag_id] = 0;
        }
    }
}

std::vector<cv::Rect2i> Tracker::calcROI(const std::vector<XYV>& projects, int width, int height, int camera_width, int camera_height) 
{
    std::vector<cv::Rect2i> rois;
    std::vector<bool> used(projects.size(), false);
    
    for(size_t i = 0; i < projects.size(); i++) {
        if(used[i] || !projects[i].visible) {
            continue;
        }
        
        // 计算ROI中心
        int center_x = static_cast<int>(projects[i].x);
        int center_y = static_cast<int>(projects[i].y);
        
        // 计算ROI区域，确保不超出边界
        int x = std::clamp(center_x - width/2, 0, camera_width - width);
        int y = std::clamp(center_y - height/2, 0, camera_height - height);
        
        cv::Rect2i roi(x, y, width, height);
        rois.push_back(roi);
        
        // 标记所有在当前ROI中的点
        for(size_t j = i + 1; j < projects.size(); j++) {
            if(roi.contains(cv::Point(static_cast<int>(projects[j].x), 
                                    static_cast<int>(projects[j].y)))) {
                used[j] = true;
            }
        }
    }
    
    return rois;
}


std::pair<double, double> calculateRotatedRect(const ArmorXYV& points_) {
    // 检查输入点数是否为4
    if (points_.size() != 4) {
        return {0.0, 1.0}; // 返回默认值
    }
    std::vector<cv::Point2f> points;
    for (const auto& point : points_) {
        points.push_back(cv::Point2f(point.x, point.y));
    }
    // 使用 OpenCV 的 minAreaRect 函数计算外接旋转矩形
    cv::RotatedRect rotatedRect = cv::minAreaRect(points);
    
    // 获取矩形的角度（OpenCV 中角度范围为 [-90, 0)）
    double angle = rotatedRect.angle;
    
    // 将角度转换为弧度制，并调整到 [0, π) 范围
    double theta = angle * CV_PI / 180.0;
    if (theta < 0) {
        theta += CV_PI; // 转换到 [0, π) 范围
    }
    
    // 获取矩形的宽和高
    double width = rotatedRect.size.width;
    double height = rotatedRect.size.height;
    
    if(width < height) {
        theta += CV_PI / 2;
        if(theta >= CV_PI) {
            theta -= CV_PI;
        }
    }
    double aspectRatio = width / height;
    
    return {theta, aspectRatio};
}



std::map<int, Matcher> matchers;

// 此算法依赖carid的准确性
//old_prediction: (XYV, car_id, armor_id)
TrackResults Tracker::getTrackResult(Time::TimeStamp time, ImuData imu) 
{
    for(const auto& armor : armors_gray) {
        if(dead.find(armor.second) == dead.end()) {
            dead[armor.second] = maxDeadFrames + 1;
        }
    }
    for(auto& [car_id, dead_frames] : dead) {
        if(dead_frames <= maxDeadFrames) {
            dead_frames++;
        }
    }
    // A very strict condition for gray armor
    // Just for avoid situation: red_4 sentry & blue_4 sentry
    // are all hit and turn to gray
    // only satisfy conditions 1. dead <= maxDeadFrames
    // 2. deadArmor num == 1 
    // 3. no armor in armors
    // 's gray armor will be added to armors
    std::map<int, std::vector<std::pair<ArmorXYV, int>>> gray_armors_by_car;  // (car_id, [(armor, armor_id)])

    // 统计灰色装甲板
    for (const auto& armor : armors_gray) {
        gray_armors_by_car[armor.second].push_back(armor);
    }
    for (const auto& armor : armors) {
        gray_armors_by_car[armor.second].clear();  
    }
    for (auto& [car_id, gray_armors] : gray_armors_by_car) {
        if (gray_armors.size() == 1 && dead[car_id] <= maxDeadFrames) {
            armors.push_back(gray_armors[0]);
        }
    }



    TrackResults results;
    if(armors.empty()) return results;


    std::map<int,std::vector<std::pair<Point2f,const ArmorXYV*>>> params;//(car_id, centers)
    for(const auto& armor : armors) {
        location::Location center_location;
        center_location.cxy = {(armor.first[0].x + armor.first[1].x + armor.first[2].x + armor.first[3].x) / 4,
            (armor.first[0].y + armor.first[1].y + armor.first[2].y + armor.first[3].y) / 4};
        //center_location.imu = imu;
        //CXYD coord = center_location.cxy_imu;
        CXYD coord = center_location.getImuCXY(imu);
        cv::Point2f center(coord.cx/10.0, coord.cy/10.0);
        params[armor.second].push_back(std::make_pair(center, &armor.first));
    }

    for(auto& [car_id, param] : params) {
        if(matchers.find(car_id) == matchers.end()) {
            matchers.emplace(car_id, Matcher());
        }
        auto result = matchers[car_id].track(param, time, timeRatio);
        if(result.size() != param.size()) {
            ERROR("TrackerMatcher error: result size not equal to center size");
            continue;
        }
        for(auto& [id, armor_ptr] : result) {
            TrackResult track_result;
            track_result.armor = *armor_ptr;
            track_result.armor_id = id;
            //INFO("Car {} Armor {} at ({}, {})", car_id, id, track_result.extra_center.x, track_result.extra_center.y);
            track_result.car_id = car_id;
            results.push_back(track_result);
        }
    }

    armors.clear();  // 清空当前帧的装甲板
    armors_gray.clear();
    return results;
}

bool Tracker::isDetected() 
{
    return !armors.empty();
}
} // namespace tracker
