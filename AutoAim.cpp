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
    auto driver = createReplayer("../record/test1.mkv", "null", true, 1);
    //auto solver = createSolver(param["X"].Double(), param["Y"].Double(), param["X1"].Double(),
    //                            param["Y1"].Double(), param["X2"].Double(), param["Y2"].Double());
    auto solver = createSolver(param["solver"]);
    auto controller = createController(param["controller"]);
    auto predictor = createPredictor();
    auto detector = createDetector(param["model_path"].String(), param["car_model_path"].String(), false);
    auto tracker = createTracker();


    location::Location::registerSolver(solver);
    ParsedSerialData imu;

    controller->registPredictFunc(predictor->predictFunc());
//    auto predictFunc = predictor->predictFunc();
//    controller->registPredictFunc([&predictFunc,&solver,&imu](Time::TimeStamp timestamp) {
//        auto result = predictFunc(timestamp);
//        return result;
//    });

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
    bool isLastDetected = false;
    //Eigen::Vector5d distortionCoefficients = Eigen::Vector5d(-0.0658577209154935, 0.125641157995530, 0, 0, -0.0808951533858169);
    bool autoEnemy = (enemyTrans[param["enemy_color"].String()] == -1);
    if(!autoEnemy)
        detector->setEnemyColor(enemyTrans[param["enemy_color"].String()]);
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


        // std::vector<Prediction> predictions;
        // if(predictor->Stable())
        //     predictions = predictor->predict(frame->timestamp);

        //calcROI func will automatically filter out the predictions that are not in the camera view
        // std::vector<XYV> projects;
        // for(const auto &prediction : predictions)
        //     for(const auto &armor : prediction.armors)
        //         if(armor.status != armor.NONEXIST)
        //             projects.push_back(armor.location.getImuCXY(imu_data));
        //auto roi_regions = tracker->calcROI(projects);
        
        auto detections = detector->detect(frame->image);
        tracker->merge(detections.first);
        tracker->merge(detections.second);
        isLastDetected = tracker->isDetected();

        auto trackResults = tracker->getTrackResult(frame->timestamp, imu);
        for(auto& trackResult : trackResults.first)
        {
            //search car_id in trackResults.second
            auto it = std::find_if(trackResults.second.begin(), trackResults.second.end(), [&trackResult](const auto& armor) {
                return armor.car_id == trackResult.car_id;
            });
            PYD pyd_imu;
            double yaw = 0;
            if(it == trackResults.second.end())
                std::tie(pyd_imu, yaw) = solver->camera2world(trackResult.armor, imu_data, trackResult.car_id == 1);
            else
            {
                std::tie(pyd_imu, yaw) = solver->camera2worldWithWholeCar(trackResult.armor, imu_data, it->bounding_rect, trackResult.car_id == 1);
            }
            trackResult.location.imu = imu_data;
            trackResult.location.pyd_imu = pyd_imu;
            //XYZ tmp = trackResult.location.xyz_imu;
            //trackResult.location.xyz_imu = tmp; 
            trackResult.yaw = yaw;
            CXYD coord = trackResult.location.cxy;
            cv::circle(frame->image, cv::Point(coord.cx, coord.cy), 12, cv::Scalar(255, 255, 0), -1);
            location::Location temp;
            temp.imu = imu_data;
            XYZ temp_xyz = trackResult.location.xyz_imu;
            temp_xyz.z = 0;
            temp.xyz_imu = temp_xyz;
            CXYD coord_z0 = temp.cxy;
            cv::line(frame->image, cv::Point(coord.cx, coord.cy), cv::Point(coord_z0.cx, coord_z0.cy), cv::Scalar(255, 255, 0), 2);

            std::string text = std::to_string(trackResult.yaw);
            cv::putText(frame->image, text, cv::Point(coord.cx, coord.cy), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
            std::string text_id = std::to_string(trackResult.armor_id);
            cv::putText(frame->image, text_id, cv::Point(coord.cx, coord.cy+10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        }
        //visualize trackResults on frame
        for(auto& trackResult : trackResults.second)
        {
            auto car_id = trackResult.car_id;
            auto car_type = trackResult.car_type;
            auto bounding_rect = trackResult.bounding_rect;
            cv::rectangle(frame->image, bounding_rect, cv::Scalar(255, 0, 0), 5);
            cv::putText(frame->image, std::to_string(car_id), bounding_rect.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 5);
            cv::putText(frame->image, std::to_string(car_type), bounding_rect.tl() + cv::Point2f(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 5);
        }
        INFO("TrackResults size: {}", trackResults.first.size());
        predictor->update(trackResults, frame->timestamp);

        //DEBUG
        auto predictions = predictor->predict(frame->timestamp);
        //visualize predictions on frame
        bool show = false;
        for(auto& prediction : predictions)
        {
            show = true;
            XYZ center = prediction.center;
            UdpSend::sendData((float)center.x);
            UdpSend::sendData((float)center.y);
            UdpSend::sendData((float)prediction.z1);
            UdpSend::sendData((float)prediction.z2);
            UdpSend::sendData((float)prediction.omega);
            UdpSend::sendData((float)prediction.vx);
            UdpSend::sendData((float)prediction.vy);
            UdpSend::sendData((float)prediction.theta);
            INFO("ENTER center: x: {}, y: {}, z: {}", center.x, center.y, center.z);
            //trans to CXYD
            location::Location temp;
            temp.imu = imu_data;
            temp.xyz_imu = center;
            CXYD coord = temp.cxy;
            INFO("ENTER coord: cx: {}, cy: {}, distance: {}", coord.cx, coord.cy, coord.distance);
            cv::circle(frame->image, cv::Point(coord.cx, coord.cy), 8, cv::Scalar(200, 200, 200), -1);
            for(auto& armor : prediction.armors)
            {
                auto armor_center = armor.center;
                temp.xyz_imu = armor_center;
                CXYD armor_coord = temp.cxy;
                if(armor.status == armor.UNSEEN)
                    cv::circle(frame->image, cv::Point(armor_coord.cx, armor_coord.cy), 5, cv::Scalar(0, 0, 255), -1);
                else
                    cv::circle(frame->image, cv::Point(armor_coord.cx, armor_coord.cy), 10, cv::Scalar(100, 255, 100), -1);
                armor_center.z = 0;
                temp.xyz_imu = armor_center;
                CXYD armor_coord_z0 = temp.cxy;
                cv::line(frame->image, cv::Point(armor_coord.cx, armor_coord.cy), cv::Point(armor_coord_z0.cx, armor_coord_z0.cy), cv::Scalar(255, 255, 0), 2);
                std::string text = std::to_string(armor.yaw - temp.imu.yaw);
                cv::putText(frame->image, text, cv::Point(armor_coord.cx, armor_coord.cy), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
                std::string text_id = std::to_string(armor.id);
                cv::putText(frame->image, text_id, cv::Point(armor_coord.cx, armor_coord.cy+10), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            }
        }
        int count_armor_debug = 0;
        if(show)
        for(auto& trackResult : trackResults.first)
        {
            XYZ armor_xyz = trackResult.location.xyz_imu;
            UdpSend::sendData(armor_xyz.x);
            INFO("debug_armor x: {}, y: {}, z: {}", armor_xyz.x, armor_xyz.y, armor_xyz.z);
            UdpSend::sendData(armor_xyz.y);
            count_armor_debug++;
            if(count_armor_debug>= 2)
                break;
        }
        if(show)
        if(count_armor_debug<2)
        {
            UdpSend::sendData((float)0);
            UdpSend::sendData((float)0);
        }

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