#include <iostream>
#include <ranges>
#include <modules/modules.hpp>
#include <opencv2/opencv.hpp>
#include <modules/TestSpeed/testspeed.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
using namespace modules;
using namespace aimlog;


// 控制指令最好不要以超高频率发送
// 控制指令间隔不能较长，须保证串口处于激活状态
// 对多线程共享变量的赋值不能极高频率，否则可能导致另一个线程无法访问。

int main() {
    auto driver = createDriver();
    auto solver = createSolver();
    auto controller_ = createController();
    //因为lambda不能move，所以需要先构造一个shared_ptr
    auto controller = std::shared_ptr<modules::Controller>(std::move(controller_));
    auto predictor = createPredictor();
    auto detector = createDetector("../utils/models/armor_yolo_x.xml");
    auto tracker = createTracker();

    controller->registPredictFunc(predictor->predictFunc());

    SerialConfig config{"/dev/ttyACM0", 115200};
    //SerialConfig config{"/dev/pts/11", 115200};
    CameraConfig cameraConfig{.cameraSN = "KE0200060399", .exposureTime = 5000};
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);
    driver->registReadCallback([control_func = driver->sendSerialFunc() ,controller](const ParsedSerialData& parsedData) {
        ControlResult result = controller->control(parsedData);
        control_func(result);
    });
    driver->runSerialThread();
    driver->runCameraThread();

    Eigen::Matrix3d cameraIntrinsicMatrix;
    cameraIntrinsicMatrix << 1654.11137212488, 0, 637.460189924719,
                             0, 1651.88235365380, 496.536024907796,
                             0, 0, 1;
    solver->setCameraIntrinsicMatrix(cameraIntrinsicMatrix);
    auto cameraOffset = Eigen::Vector3d(0, -0.08, 0.05);
    solver->setCameraOffset(cameraOffset);
    Eigen::Vector5d distorationCoefficients = Eigen::Vector5d(-0.0658577209154935, 0.125641157995530, 0, 0, -0.0808951533858169);
    solver->setDistorationCoefficients(distorationCoefficients);

    detector->setEnemyColor(0);

    bool isLastDetected = false;
    int reCheckTime = 10;
    Time::TimeStamp lastDetectTime;
    while(1)
    {
        if(!driver->isExistNewCameraData())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }
        //if multiple frame, get the latest one and discard the others
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        auto frame = camera_data_pack.back();
        auto deltaTime = (frame->timestamp - lastDetectTime).toSeconds();
        lastDetectTime = frame->timestamp;
        ParsedSerialData imu = driver->findNearestSerialData(frame->timestamp);
        driver->clearSerialData();

        if(!isLastDetected || deltaTime > reCheckTime)
        {
            tracker->merge(detector->detect(frame->image));
        }
        std::vector<Prediction> predictions = predictor->predict(frame->timestamp);

        //calcROI func will automatically filter out the predictions that are not in the camera view
        std::vector<std::tuple<XYV, int, int>> old_predictions;
        for(const auto &prediction : predictions)
            for(const auto &armor : prediction.armors)
                if(armor.status != armor.NONEXIST)
                {
                    auto project = solver->world2camera(PYD::XYZ2PYD(armor.x, armor.y, armor.z), imu);
                    old_predictions.push_back(std::make_tuple(project, prediction.id, armor.id));
                }

        XYVs projects;
        std::transform(old_predictions.begin(), old_predictions.end(), 
            std::back_inserter(projects),
            [](const auto& pred) { return std::get<0>(pred); });
        auto roi_regions = tracker->calcROI(projects);

        for(const auto &roi : roi_regions) 
            tracker->merge(detector->detect(frame->image, roi));
        isLastDetected = tracker->isDetected();

        auto trackResults = tracker->getTrackResult(old_predictions);
        ArmorXYVs trackArmorXYVs;
        std::transform(trackResults.begin(), trackResults.end(), 
            std::back_inserter(trackArmorXYVs),
            [](const auto& result) { return result.armor; });
        auto pyds = solver->camera2world(trackArmorXYVs, imu, false);
        for(int i = 0; i < pyds.size(); i++)
            trackResults[i].location = pyds[i];
            
        predictor->update(trackResults, frame->timestamp);
    }



    return 0;
}