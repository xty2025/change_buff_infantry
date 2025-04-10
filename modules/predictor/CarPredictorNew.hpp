#pragma once 
#include "timeEKF.hpp"
#include "ceres/ceres.h"
#include "type.hpp"
#include "Udpsend/udpsend.hpp"
#include "Log/log.hpp"

namespace predictor{
    //measure : gen_x, gen_y, distance
    using VectorY = Eigen::Matrix<double, 5, 1>;
    enum CarStatus
    {
        STILL,
        LINEAR,
        ROTATE,
        UNSTABLE
    };

    struct ArmorSeries
    {
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> dist;
        std::vector<Time::TimeStamp> time;
        double begin_x = 0, end_x = 0, average_y = 0;
        double v_x = 0;
        double v_y = 0;
        double center_time;
        int missFrame = 0;
    };

    class DistKF
    {
    public:
        //initDistKF(){};
        //not use KF but just use a simple average
        void updateDistKF(double dist)
        {
            this->dist_ = alpha * dist + (1 - alpha) * this->dist_;
            //UdpSend::sendData((float)this->dist_);
        }
        double distPredict()
        {
            return dist_;
        }
        double dist_ = 3;
        double alpha = 0.8;
    };

    class MotionModel 
    {
    public:
        void initMotionModel()
        {

            //distKF.initDistKF(1, 0.1, 0.1);
        }
        double findX(ArmorSeries* series, double deltaT)
        {
            //use histrory data to linear predict
            //find x1, x2, t1, t2
            //t1<deltat<t2
            //x = x1 + (x2 - x1) / (t2 - t1) * (deltat - t1)
            if(series->time.size() < 2)
                return 0;
            int i = 0;
            while(i < series->time.size() && (series->time[i] - series->time.front()).toSeconds() < deltaT)
            {
                i++;
            }
            if(i == 0)
                return series->x.front();
            if(i == series->time.size())
                return series->x.back();
            double x1 = series->x[i - 1];
            double x2 = series->x[i];
            double t1 = (series->time[i - 1] - series->time.front()).toSeconds();
            double t2 = (series->time[i] - series->time.front()).toSeconds();
            //return x1;// + (x2 - x1) / (t2 - t1) * (deltaT - t1);
            return x1 + (x2 - x1) / (t2 - t1) * (deltaT - t1);
            //return x1 + series->v_x * (deltaT - t1);
        }
        double findY(ArmorSeries* series, double deltaT)
        {
            //use histrory data to linear predict
            //find y1, y2, t1, t2
            //t1<deltat<t2
            //y = y1 + (y2 - y1) / (t2 - t1) * (deltat - t1)
            if(series->time.size() < 2)
                return 0;
            int i = 0;
            while(i < series->time.size() && (series->time[i] - series->time.front()).toSeconds() < deltaT)
            {
                i++;
            }
            if(i == 0)
                return series->y.front();
            if(i == series->time.size())
                return series->y.back();
            double y1 = series->y[i - 1];
            double y2 = series->y[i];
            double t1 = (series->time[i - 1] - series->time.front()).toSeconds();
            double t2 = (series->time[i] - series->time.front()).toSeconds();
            //return y1 ;//+ (y2 - y1) / (t2 - t1) * (deltaT - t1);
            return y1 + (y2 - y1) / (t2 - t1) * (deltaT - t1);
        }
        bool rotateFind(int armor_id, const Time::TimeStamp& timestamp,std::pair<double,double> &result)
        {
            //find newest series with 2 | armor_id - function's armor_id
            //first find in workingSeries
            ArmorSeries* series = nullptr;
            //double delta_work_time = -1;
            for(auto& [id, s] : workingSeries)
            {
                if(((id - armor_id) % 2) == 0)
                {
                    if(series == nullptr || s.time.back() > series->time.back())
                    {
                        series = &s;
                        //delta_work_time = fmod((timestamp - s.time.back()).toSeconds(),period_T * 0.5);
                    }
                }
            }
            ArmorSeries* finish_series = nullptr;
            //INFO("ttt.......armor_id:{}", armor_id);
            double delta_finish_time = -1;
            //then find in finishSeries
            for(auto& [id, s] : finishSeries)
            {
                if(((id - armor_id) % 2) == 0)
                {
                    if(finish_series == nullptr || s.time.back() > finish_series->time.back())
                    {
                        finish_series = &s;
                        delta_finish_time = fmod((timestamp - s.time.front()).toSeconds(),period_T * 0.5);
                    }
                }
            }
            INFO("delta_finish_time:{}", delta_finish_time);
            if(finish_series == nullptr)
                return false;
            if((finish_series->time.back() - finish_series->time.front()).toSeconds() < delta_finish_time + biastime)
                return false;
            if(series == nullptr)
{                result = {findX(finish_series, delta_finish_time), findY(finish_series, delta_finish_time)};}
            else
            {
                INFO("delta1:{} delta2:{}", (series->time.back() - finish_series->time.front()).toSeconds(), delta_finish_time);
                double delta1 = fmod((series->time.back() - finish_series->time.front()).toSeconds(),period_T *0.5);
                result = {series->x.back() - findX(finish_series, delta1) + findX(finish_series, delta_finish_time),
                    series->y.back() - findY(finish_series, delta1) + findY(finish_series, delta_finish_time)};
            }
            //result = {findX(finish_series, finish_series->center_time), findY(finish_series, finish_series->center_time)};

            return true;
        }
        Prediction getPredictResult(const Time::TimeStamp& timestamp, int carid)
        {
            if(status == LINEAR)
            {
                Prediction result;
                result.id = carid;
                result.dist = distKF.distPredict();
                for (int i = 0; i < 4; i++)
                {
                    result.armors[i].id = i;
                    if(workingSeries.find(i) != workingSeries.end())
                    {
                        double deltaT = (timestamp - workingSeries[i].time.back()).toSeconds();
                        result.armors[i].gen_cam_x = workingSeries[i].x.back() + workingSeries[i].v_x * deltaT;
                        result.armors[i].gen_cam_y = workingSeries[i].y.back() + workingSeries[i].v_y * deltaT;
                        result.armors[i].gen_cam_dist = distKF.distPredict();
                        result.armors[i].status = Armor::AVAILABLE;
                    }
                    else
                    {
                        result.armors[i].id = i;
                        result.armors[i].status = Armor::UNSEEN;
                    }
                }
                result.status = Prediction::LINERAL;
                return result;
            }
            else if(status == ROTATE)
            {
                Prediction result;
                result.id = carid;
                result.dist = distKF.distPredict();
                //find newest finishSeries and use its xbegin, xend to calc centerx
                ArmorSeries* series = nullptr;
                for(auto& [id, s] : finishSeries)
                {
                    if(series == nullptr || s.time.back() > series->time.back())
                    {
                        series = &s;
                    }
                }
                if(series == nullptr)
                {
                    result.status = Prediction::UNKNOW;
                    return result;
                }else
                {
                    result.gen_cam_x = (series->end_x + series->begin_x) / 2;
                    result.gen_cam_y = series->average_y;
                }

                for (int i = 0; i < 4; i++)
                {
                    //auto [x, y] = rotateFind(i, timestamp);
                    std::pair<double,double> resultXY;
                    if(rotateFind(i, timestamp, resultXY))
                    {
                        result.armors[i].gen_cam_x = resultXY.first;
                        result.armors[i].gen_cam_y = resultXY.second;
                        result.armors[i].gen_cam_dist = distKF.distPredict();
                        result.armors[i].status = Armor::AVAILABLE;
                    }
                    else
                    {
                        result.armors[i].id = i;
                        result.armors[i].status = Armor::UNSEEN;
                        // if(workingSeries.find(i) != workingSeries.end())
                        // {
                        //     double deltaT = (timestamp - workingSeries[i].time.back()).toSeconds();
                        //     result.armors[i].gen_cam_x = workingSeries[i].x.back() + workingSeries[i].v_x * deltaT;
                        //     result.armors[i].gen_cam_y = workingSeries[i].y.back() + workingSeries[i].v_y * deltaT;
                        //     result.armors[i].gen_cam_dist = distKF.distPredict();
                        //     result.armors[i].status = Armor::AVAILABLE;
                        // }
                    }
                }
                int targetid = -1;
                if(clock_wise)
                {
                    for(int i = 0; i < 4; i++)
                    {
                        if((result.armors[i].status == Armor::AVAILABLE)
                            && ((result.armors[i].gen_cam_x - result.gen_cam_x) > 0)
                            && ((targetid == -1) 
                                || (result.armors[i].gen_cam_x < result.armors[targetid].gen_cam_x)))
                        
                        {
                            targetid = i;
                        }
                    }
                    if(targetid != -1)
                        result.gen_cam_y = result.armors[targetid].gen_cam_y;
                }
                else
                {
                    for(int i = 0; i < 4; i++)
                    {
                        if((result.armors[i].status == Armor::AVAILABLE)
                            && ((result.armors[i].gen_cam_x - result.gen_cam_x) < 0)
                            && ((targetid == -1) 
                                || (result.armors[i].gen_cam_x > result.armors[targetid].gen_cam_x)))
                        
                        {
                            targetid = i;
                        }
                    }
                    if(targetid != -1)
                        result.gen_cam_y = result.armors[targetid].gen_cam_y;
                }
                result.status = Prediction::ROTATE;
                return result;
            }
            else
            {
                Prediction result;
                result.id = carid;
                for (int i = 0; i < 4; i++)
                {
                    result.armors[i].id = i;
                    result.armors[i].status = Armor::NONEXIST;
                }
                result.status = Prediction::UNKNOW;
                return result;
            }
        }
        void calcCenterTime(ArmorSeries& series)
        {
            //average time
            double sum = 0;
            double avg_y = 0;
            for(auto& y : series.y)
            {
                avg_y += y;
            }
            series.average_y = avg_y / series.y.size();
            for(auto& time : series.time)
            {
                sum += (time - beginTime).toSeconds();
            }
            series.center_time = sum / series.time.size();
            series.begin_x = series.x.front();
            series.end_x = series.x.back();
            series.v_x = (series.end_x - series.begin_x) / series.center_time;
        }
        void updateT(void)
        {
            //check 2 newest finishSeries, calc center_time and update period_T
            if(finishSeries.size() > 1)
            {
                // 使用更安全的方式初始化两个迭代器
                auto newest = finishSeries.begin();
                auto second_newest = std::next(finishSeries.begin());
                
                // 确保newest确实是最新的
                if(second_newest->second.center_time > newest->second.center_time) {
                    std::swap(newest, second_newest);
                }
                
                // 从第三个元素开始遍历，找出最新和第二新的元素
                for(auto it = std::next(finishSeries.begin(), 2); it != finishSeries.end(); it++)
                {
                    if(it->second.center_time > newest->second.center_time)
                    {
                        second_newest = newest;
                        newest = it;
                    }
                    else if(it->second.center_time > second_newest->second.center_time)
                    {
                        second_newest = it;
                    }
                }
                
                double tmp_period_T = newest->second.center_time - second_newest->second.center_time;
                tmp_period_T *= 4;
                period_T = tmp_period_T * period_alpha + period_T * (1 - period_alpha);
                INFO("period_T:{}", period_T);
            }
        }
        void Update(std::vector<std::pair<Eigen::VectorXd,int>>& measure, const Time::TimeStamp& timestamp)
        {
            for(auto& [id,series] : finishSeries)
              std::cout<<"finish:"<<id<<"size:"<<series.x.size()<<std::endl;
            //for(auto& [id,series] : workingSeries)
              //std::cout<<"finish_work:"<<id<<"size:"<<series.x.size()<<std::endl;
            //UdpSend::sendData((float)period_T);
            if(finishSeries.size()>0)
            {
                for(auto& [id,series] : finishSeries)
                {
                    UdpSend::sendData((float)((series.begin_x+series.end_x)/2));
                    break;
                }
            }
            else{
                UdpSend::sendData((float)0);
            }
            //allocate measure to workingSeries
            for(auto& [measure, armorid] : measure)
            {
                if(workingSeries.find(armorid) == workingSeries.end())
                {
                    workingSeries[armorid] = ArmorSeries();
                }
                workingSeries[armorid].x.push_back(measure[0]);
                workingSeries[armorid].y.push_back(measure[1]);
                UdpSend::sendData((float)measure[0]);
                UdpSend::sendData((float)measure[1]);
                UdpSend::sendData((float)workingSeries[armorid].v_x);
                UdpSend::sendData((float)workingSeries[armorid].v_y);
                workingSeries[armorid].dist.push_back(measure[2]);
                workingSeries[armorid].time.push_back(timestamp);
                workingSeries[armorid].missFrame = 0;
                distKF.updateDistKF(measure[2]);
            }
            //while map's size > 2, move the oldest to finishSeries
            while(workingSeries.size() > 2)
            {
                auto oldest = workingSeries.begin();
                for(auto it = workingSeries.begin(); it != workingSeries.end(); it++)
                {
                    if(it->second.center_time < oldest->second.center_time)
                    {
                        oldest = it;
                    }
                }
                calcCenterTime(oldest->second);
                finishSeries[oldest->first] = oldest->second;
                //updateT();
                workingSeries.erase(oldest);
            }
                        while(finishSeries.size() > 2)
            {
                auto oldest = finishSeries.begin();
                for(auto it = finishSeries.begin(); it != finishSeries.end(); it++)
                {
                    if(it->second.center_time < oldest->second.center_time)
                    {
                        oldest = it;
                    }
                }
                //calcCenterTime(oldest->second);
                //finishSeries[oldest->first] = oldest->second;
                //updateT();
                finishSeries.erase(oldest);
            }
            //if any series in workingSeries is too old, move it to finishSeries
            for(auto it = workingSeries.begin(); it != workingSeries.end();)
            {
                if((Time::TimeStamp::now() - it->second.time.back()).toSeconds() > missTimeMax ||
                    it->second.missFrame > missFrameMax)
                {
                    calcCenterTime(it->second);
                    finishSeries[it->first] = it->second;
                    //updateT();
                    it = workingSeries.erase(it);
                }
                else
                {
                    it->second.missFrame++;
                    it++;
                }
            }
            //check finishSeries, if any series is too old or too short, erase it
            for(auto it = finishSeries.begin(); it != finishSeries.end();)
            {
                if((Time::TimeStamp::now() - it->second.time.back()).toSeconds() > missTimeFinishMax
                || it->second.x.size() < validFrameMin)
                {
                    it = finishSeries.erase(it);
                }
                else
                {
                    it++;
                }
            }
            updateT();
            //exam workingSeries & finishSeries' each elements' length, if too big, choose status LINEAR
            //extraly, ROTATE status need finishSeries' size > 1
            if(finishSeries.size() > 1)
            {
                status = ROTATE;
                for(auto& [armorid, series] : workingSeries)
                {
                    if(series.x.size() > rotateFrameMax)
                    {
                        status = LINEAR;
                        break;
                    }
                }
                for(auto& [armorid, series] : finishSeries)
                {
                    if(series.x.size() > rotateFrameMax)
                    {
                        status = LINEAR;
                        break;
                    }
                }
                //if(status == ROTATE)
                //{
                 //   rotate_support++;
                //}
                //else
                //{
                 //   rotate_support = 0;
                //}
                //if(rotate_support > rotate_accept)
                  //  status = ROTATE;
                //else
                 //   status = LINEAR;

            }
            else
                status = LINEAR;
            if(status == LINEAR)
                INFO("status: LINEAR");
            else if(status == ROTATE)
                INFO("status: ROTATE");
            else
                INFO("status: UNSTABLE");

            //first update linear part
            //v_x = (x_n - x_(n - frame_avg)) / (t_n - t_(n - frame_avg)) * alpha + v_x * (1 - alpha)
            //v_y = (y_n - y_(n - frame_avg)) / (t_n - t_(n - frame_avg)) * alpha + v_y * (1 - alpha)
            for(auto& [armorid, series] : workingSeries)
            {
                if(series.x.size() > frame_avg)
                {
                    series.v_x = (series.x.back() - series.x[series.x.size() - frame_avg]) / 
                        (series.time.back() - series.time[series.time.size() - frame_avg]).toSeconds() * alpha + 
                        series.v_x * (1 - alpha);
                    series.v_y = (series.y.back() - series.y[series.y.size() - frame_avg]) / 
                        (series.time.back() - series.time[series.time.size() - frame_avg]).toSeconds() * alpha + 
                        series.v_y * (1 - alpha);
                    if(series.v_x > 0)clock_wise = false;
                    else clock_wise = true;
                }
            }
            //then update rotate part
            //seems to no need do other thing.
        }
        bool Stable(){ return status != UNSTABLE; }
    private:
        const int rotate_accept = 50;
        int rotate_support = 0;
        const int frame_avg = 6;
        const double alpha = 0.2;
        const double biastime = 0.0;

        const int missFrameMax = 3;
        const double missTimeMax = 0.5;

        const double missTimeFinishMax = 2.0;
        const int validFrameMin = 3;
        const int rotateFrameMax = 200;


        std::map<int, ArmorSeries> workingSeries;
        std::map<int, ArmorSeries> finishSeries;

        bool clock_wise = true;// 顺时针
        DistKF distKF;
        Time::TimeStamp beginTime = Time::TimeStamp::now();
        double period_T = 1.0;
        double period_alpha = 0.9;

        CarStatus status = LINEAR;
    };
}