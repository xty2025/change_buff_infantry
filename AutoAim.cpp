#include <iostream>
#include <ranges>
#include <modules/modules.hpp>
#include <opencv2/opencv.hpp>
#include <Udpsend/udpsend.hpp>
#include <Param/param.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
#include <VideoStreamer/VideoStreamer.hpp>
#include <Recorder/recorder.hpp>
#include <Location/location.hpp>
using namespace modules;
using namespace aimlog;
using namespace recording;

//TODO:
//1.完善选板逻辑 controller
//2.加入整车预测 predictor
//3.日志完善
//4. FATAL 大小装甲板
//5. detector ROI 变换检查

// 控制指令最好不要以超高频率发送
// 控制指令间隔不能较长，须保证串口处于激活状态
// 对多线程共享变量的赋值不能极高频率，否则可能导致另一个线程无法访问。

std::map<std::string, int> enemyTrans = {
    {"red", 0},
    {"blue", 1},
    {"auto", -1}
};

int main() {
    param::Param param("../config.json");
    param = param[param["car_name"].String()];
    bool udp_enable = param["UDP"]["enable"].Bool();
    bool web_debug_enable = param["web_debug"].Bool();
    bool shoot_enable = param["shoot_enable"].Bool();
    bool record_enable = param["record_enable"].Bool();
    bool force_shoot = param["force_shoot"].Bool();
    if(udp_enable)
        UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());
    else
        UdpSend::disable();
    if(web_debug_enable)
        VideoStreamer::init();
    if(record_enable)
    {
        Recorder::instance().start();
        INFO("Recording started");
    }
    //auto driver = createDriver();
    auto driver = createReplayer("../record/record.mkv", "null", true);
    auto solver = createSolver(param["X"].Double(), param["Y"].Double(), param["X1"].Double(), 
                                param["Y1"].Double(), param["X2"].Double(), param["Y2"].Double());
    auto controller = createController();
    auto predictor = createPredictor();
    auto detector = createDetector(param["model_path"].String(), param["car_model_path"].String(), false);
    auto tracker = createTracker();


    location::Location::registerSolver(solver);
    ParsedSerialData imu;

    //controller->registPredictFunc(predictor->predictFunc());
    auto predictFunc = predictor->predictFunc();
    controller->registPredictFunc([&predictFunc,&solver,&imu](Time::TimeStamp timestamp) {
        auto result = predictFunc(timestamp);
        return result;
    });

    SerialConfig config{param["serial_name"].String(), param["baud_rate"].Int()};
    CameraConfig cameraConfig{
        .cameraSN = param["camera_id"].String(), 
        .autoWhiteBalance = param["auto_white_balance"].Bool(),
        .exposureTime = param["exposure_time"].Double(),
        .gain = param["gain"].Double()
    };
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);
    driver->registReadCallback([control_func = driver->sendSerialFunc() ,controller,shoot_enable,force_shoot,record_enable](const ParsedSerialData& parsedData) {
        if(record_enable)
        {
            Recorder::instance().addSerialData(parsedData);
        }
        ControlResult result = controller->control(parsedData);
        if(force_shoot) result.shoot_flag = 1;
        result.shoot_flag = shoot_enable? result.shoot_flag : 0;
        control_func(result);
    });
    driver->runSerialThread();
    driver->runCameraThread();

    int receive_enemy_color = 0;
    cv::Mat cameraIntrinsicMatrix = cv::Mat(3, 3, CV_64F);
    cv::Mat distorationCoefficients = cv::Mat(1, 5, CV_64F);
    
    // Load camera intrinsic matrix from parameter file
    auto intrinsicArray = param["camera_intrinsic_matrix"].to<std::vector<std::vector<double>>>();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            cameraIntrinsicMatrix.at<double>(i, j) = intrinsicArray[i][j];
    solver->setCameraIntrinsicMatrix(cameraIntrinsicMatrix);
    auto cameraOffset = Eigen::Vector3d(0.0, 0.0, 0.0);
    solver->setCameraOffset(cameraOffset);
    //Eigen::Vector5d distorationCoefficients = Eigen::Vector5d(-0.0658577209154935, 0.125641157995530, 0, 0, -0.0808951533858169);
    auto distorationCoefficientsArray = param["camera_distortion_matrix"].to<std::vector<double>>();
    for (int i = 0; i < 5; ++i)
        distorationCoefficients.at<double>(0, i) = distorationCoefficientsArray[i];
    solver->setDistorationCoefficients(distorationCoefficients);

    auto cameraTransArray = param["camera_external_matrix"]["camera_trans"].to<std::vector<double>>();
    Eigen::Vector3d cameraTrans(cameraTransArray[0], cameraTransArray[1], cameraTransArray[2]);
    auto cameraPitchAngle = param["camera_external_matrix"]["camera_pitch_angle"].Double();
    solver->setCameraExternalMatrix(cameraTrans, cameraPitchAngle);


    bool autoEnemy = (enemyTrans[param["enemy_color"].String()] == -1);
    if(!autoEnemy)
        detector->setEnemyColor(enemyTrans[param["enemy_color"].String()]);

    bool isLastDetected = false;
    int reCheckTime = 10;
    Time::TimeStamp lastDetectTime;
    while(1)
    {
        if(!driver->isExistNewCameraData())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            //INFO("Waiting for new camera data...");
            continue;
        }
        clearScreenNoDelete();

        //if multiple frame, get the latest one and discard the others
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        auto frame = camera_data_pack.back();
        auto deltaTime = (frame->timestamp - lastDetectTime).toSeconds();
        lastDetectTime = frame->timestamp;
        imu = driver->findNearestSerialData(frame->timestamp);
        ImuData imu_data = imu;
        receive_enemy_color = imu.enemy_color;
        if(autoEnemy)
            detector->setEnemyColor(1 - receive_enemy_color);
        driver->clearSerialData();


        std::vector<Prediction> predictions;
        if(predictor->Stable())
            predictions = predictor->predict(frame->timestamp);

        //calcROI func will automatically filter out the predictions that are not in the camera view
        std::vector<XYV> projects;
        for(const auto &prediction : predictions)
            for(const auto &armor : prediction.armors)
                if(armor.status != armor.NONEXIST)
                    projects.push_back(armor.location.getImuCXY(imu_data));
        auto roi_regions = tracker->calcROI(projects);

        //if(!isLastDetected || deltaTime > reCheckTime || roi_regions.empty())
        //{
            tracker->merge(detector->detect(frame->image).first);
        //}
        // for(const auto &roi : roi_regions) 
        // {
        //     tracker->merge(detector->detect(frame->image, roi).first);
        //     //cv::rectangle(frame->image, roi, cv::Scalar(0, 255, 0), 2);
        // }
        isLastDetected = tracker->isDetected();

        auto trackResults = tracker->getTrackResult(frame->timestamp, imu);
        for(auto& trackResult : trackResults)
        {
            trackResult.location.pyd_imu = solver->solveArmorPoses(trackResult.armor,trackResult.car_id,imu_data);
            trackResult.location.imu = imu_data;
            CXYD coord = trackResult.location.cxy;
            cv::circle(frame->image, cv::Point(coord.cx, coord.cy), 15, cv::Scalar(0, 255, 0), -1);
        }
        INFO("TrackResults size: {}", trackResults.size());
        predictor->update(trackResults, frame->timestamp);

        if(udp_enable)
            UdpSend::sendTail();
        if(web_debug_enable)
            VideoStreamer::setFrame(frame->image);
        if(record_enable)
        {
            INFO("ADD FRAME");
            while(camera_data_pack.size() > 0)
            {
                auto camera_data = camera_data_pack.front();
                camera_data_pack.pop();
                Recorder::instance().addFrame(camera_data);
            }
            INFO("FINISH FRAME");

        }
    }
    VideoStreamer::cleanup();
    if(record_enable)
    {
        Recorder::instance().stop();
        INFO("Recording stopped");
    }
    return 0;
}