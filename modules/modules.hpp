#pragma once
#include "interfaceType.hpp"
#include <Eigen/Core>

namespace modules
{
    class Driver {
    public:
        virtual void setSerialConfig(SerialConfig config) = 0;
        virtual void setCameraConfig(CameraConfig config) = 0;
        virtual std::function<void(const ControlResult&)> sendSerialFunc() = 0;
        virtual void registReadCallback(std::function<void(const ParsedSerialData&)> callback) = 0;
        virtual void runSerialThread() = 0;
        virtual void runCameraThread() = 0;
        virtual bool isExistNewCameraData() = 0;
        virtual void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) = 0;
        virtual ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp) = 0;
        virtual void clearSerialData() = 0;
    };
    std::unique_ptr<Driver> createDriver();

    class Controller {//TODO: 开火控制还没写好
    public:
        virtual void registPredictFunc(std::function<Predictions(Time::TimeStamp)> predictFunc) = 0;
        //virtual void registSolveFunc(std::function<Projects(const Predictions&, const ParsedSerialData&)> solveFunc) = 0;
        virtual ControlResult control(const ParsedSerialData& parsedData) = 0;
    };
    std::unique_ptr<Controller> createController();

    class Detector {
    public:
        //virtual Detections detect(const ParsedCameraData& frame) = 0;
        virtual Detections detect(const cv::Mat &image, const cv::Rect &roi = cv::Rect()) = 0;
        virtual void setEnemyColor(int flag) = 0;//0: red, 1: blue
    };
    std::unique_ptr<Detector> createDetector(const std::string &model_path);

    class Tracker {//TODO: getTrackResult method has HUGE problem for index.
    public:
        virtual void merge(const Detections& detections, double threshold=20.0f) = 0;
        virtual std::vector<cv::Rect2i> calcROI(const XYVs& projects, int width=416, int height=416, int camera_width=1280, int camera_height=1024) = 0;//TODO: change to use ArmorXYVs to auto adjust the roi size.
        virtual TrackResults getTrackResult(const std::vector<std::tuple<XYV,int,int>> &old_prediction) = 0;
        virtual bool isDetected() = 0;
    };
    std::unique_ptr<Tracker> createTracker();

    class Solver {//TODO: 传入初始值并改用迭代式算法进行pnp求解可能会增加稳定性。
    public:
        virtual XYV world2camera(const PYD& point, const ImuData& imuData) = 0;
        virtual PYDs camera2world(const ArmorXYVs& trackResults, const ImuData& imuData, bool isLarge) = 0;
        virtual void setCameraIntrinsicMatrix(const Eigen::Matrix3d& cameraIntrinsicMatrix) = 0;
        virtual void setCameraOffset(const Eigen::Vector3d& cameraOffset) = 0;
        virtual void setDistorationCoefficients(const Eigen::Vector5d& distorationCoefficients) = 0;//k1,k2,p1,p2,k3
    };
    std::unique_ptr<Solver> createSolver();

    class Predictor {//TODO: 多车同时检测的记忆功能，加快收敛速度。
    public://TODO: 应当新增自动初始化解算函数  VITAL
        virtual std::function<Predictions(Time::TimeStamp)> predictFunc() = 0;
        virtual Predictions predict(Time::TimeStamp timestamp) = 0;
        virtual void update(const TrackResults& trackResults, const Time::TimeStamp& timestamp) = 0;
    };
    std::unique_ptr<Predictor> createPredictor();
}
