#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <tuple>
#include <limits>
#include <cmath>
#include <algorithm>
#include <set>
#include "type.hpp"
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>

using namespace aimlog;
using namespace cv;
using namespace std;
namespace tracker{

class MatcherWholeCar {
public:
    MatcherWholeCar(){}

    using Point = CXYD;
    inline double calcTheta(Point armor_point,cv::Rect2f car_rect,int id)
    {
        double armor_x = armor_point.cx;
        double center_x = car_rect.x + car_rect.width / 2;
        double theta;
        if(std::abs(armor_x - center_x) > car_rect.width / 2)
            theta = (armor_x < center_x)?-M_PI/2:M_PI/2;
        else
            theta = std::asin( (armor_x - center_x) / car_rect.width * 2);
        theta -= id * M_PI / 2;
        return theta;
    }
    //装甲板与整车的匹配
    template<typename T>
    map<int,T*> track(const vector<tuple<Point,cv::Rect2f, T*>>& currentPoints, Time::TimeStamp time) {
        if(currentPoints.empty())
        {
            missFrame++;
            return {};
        }
        if(missFrame > maxMissFrame) newTrack = true;
        missFrame = 0;
        double dt = 0.0;
        //是否重新跟踪，若为true，则直接使用当前点的theta作为预测值
        if(newTrack)
        {
            newTrack = false;
            oldtheta = 0;
            omega = 0;
            oldtheta = calcTheta(get<0>(currentPoints[0]), get<1>(currentPoints[0]), 0);
        }
        else
            dt = (time - last_time).count();
        //INFO("delta_time:{}",dt);
        last_time = time;
        //按照左右排序，先左后右
        double pred_theta = oldtheta + omega * dt;
        auto sortedPoints = currentPoints;
        sort(sortedPoints.begin(), sortedPoints.end(), [](const auto& a, const auto& b) {
            return get<0>(a).cx < get<0>(b).cx;
        });
        // after sort, the first element is the leftmost
        //从0-3开始分配装甲板，起始索引为0-3中一个，使得分配的角度差总和最小。

        std::array<double,4> res_theta = {0,0,0,0};
        for(int i=0; i<4; i++)
        {
            for(int j=0; j < sortedPoints.size(); j++)
            {
                double diff = calcTheta(get<0>(sortedPoints[j]), get<1>(sortedPoints[j]), i + j)
                        - pred_theta;
                diff = std::remainder(diff,2*M_PI);
                res_theta[i] += std::abs(diff);
            }
        }
        //get<0>(sortedPoints[j])- : 获取检测点的坐标
        //get<1>(sortedPoints[j])- : 获取检测点的矩形框
        //get<2>(sortedPoints[j])- : 获取检测点关联的数据指针,将检测点按最优方案分配ID并存储到结果中
        //选取角度差总和最小的方案为最优匹配。
        int min_index = std::min_element(res_theta.begin(), res_theta.end()) - res_theta.begin();
        map<int,T*> result;
        double newtheta = 0;
        for(int j=0; j < sortedPoints.size(); j++)
        {
            result[(min_index + j)%4] = get<2>(sortedPoints[j]);
            newtheta += calcTheta(get<0>(sortedPoints[j]), get<1>(sortedPoints[j]), min_index + j);
        }
        newtheta /= sortedPoints.size();
        //INFO("newtheta,oldtheta:{},{}",newtheta,oldtheta);
        double newomega = 0;
        if(dt > 0.0001)
            newomega = std::remainder(newtheta - oldtheta, 2 * M_PI) / dt;
        else
            newomega = 0;
        while(std::abs(newomega)>maxOmega)newomega/=2;
        omega = alpha * newomega + (1 - alpha) * omega;//主要是为了平滑 omega
        //INFO("newomega:{}",omega);
        oldtheta = newtheta;
        return result;
    }

private:
    bool newTrack = true;
    Time::TimeStamp last_time;
    double oldtheta = 0;
    double omega = 0;
    const double maxOmega = 10 * 2 * M_PI / 1000.0;
    int missFrame = 0;
    const int maxMissFrame = 10;
    const double alpha = 0.9;
};

}