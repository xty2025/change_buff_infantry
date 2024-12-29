#include "tracker.hpp"

auto modules::createTracker() -> std::unique_ptr<modules::Tracker> {
    return std::make_unique<tracker::Tracker>();
}

namespace tracker {

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
        
        armors.erase(it, armors.end());
        armors.push_back(std::make_pair(new_armor, detection.tag_id));
    }
}

std::vector<cv::Rect2i> Tracker::calcROI(const XYVs& projects, int width, int height, int camera_width, int camera_height) 
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

//ole_prediction: (xyv, car_id, armor_id)
double max_match_dist = 200;
TrackResults Tracker::getTrackResult(const std::vector<std::tuple<XYV,int,int>>& old_prediction) {
    TrackResults results;
    if(armors.empty()) return results;

    // 计算距离矩阵
    std::vector<std::vector<double>> cost_matrix(armors.size(), 
        std::vector<double>(old_prediction.size(), std::numeric_limits<double>::max()));
    
    for(size_t i = 0; i < armors.size(); i++) {
        // 计算装甲板中心点
        XYV center = {0, 0};
        for(const auto& point : armors[i].first) {
            center.x += point.x;
            center.y += point.y;
        }
        center.x /= 4;
        center.y /= 4;

        for(size_t j = 0; j < old_prediction.size(); j++) {
            const auto& [pred_xyv, car_id, armor_id] = old_prediction[j];
            double dist = std::hypot(center.x - pred_xyv.x, 
                                   center.y - pred_xyv.y);
            if(dist < max_match_dist) { // 设置最大匹配距离阈值
                cost_matrix[i][j] = dist;
            }
        }
    }

    // 匈牙利算法求解
    std::vector<int> assignment(armors.size(), -1);
    for(size_t i = 0; i < armors.size(); i++) {
        double min_dist = std::numeric_limits<double>::max();
        int best_match = -1;
        
        for(size_t j = 0; j < old_prediction.size(); j++) {
            if(cost_matrix[i][j] < min_dist) {
                bool is_used = false;
                for(size_t k = 0; k < i; k++) {
                    if(assignment[k] == j) {
                        is_used = true;
                        break;
                    }
                }
                if(!is_used) {
                    min_dist = cost_matrix[i][j];
                    best_match = j;
                }
            }
        }
        
        assignment[i] = best_match;
    }

    // 生成结果
    for(size_t i = 0; i < armors.size(); i++) {
        TrackResult result;
        result.armor = armors[i].first;
        
        if(assignment[i] != -1) {
            const auto& [_, car_id, armor_id] = old_prediction[assignment[i]];
            result.car_id = car_id;
            result.armor_id = armor_id;
        } else {
            // 对于未匹配的装甲板，使用默认tag_id
            result.car_id = armors[i].second / 4;  // 假设每车4个装甲板
            result.armor_id = armors[i].second % 4;
        }
        
        results.push_back(result);
    }

    armors.clear();  // 清空当前帧的装甲板
    return results;
}

bool Tracker::isDetected() 
{
    return !armors.empty();
}
} // namespace tracker
