#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <algorithm>
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
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace modules;
using namespace aimlog;

std::map<std::string, int> enemyTrans = {
    {"red", 0},
    {"blue", 1},
    {"auto", -1},
    {"all", -2},
    {"r&b", -3}
};

// 射表记录结构
struct ShootingRecord {
    double z_height;           // z轴高度
    double horizontal_distance; // 水平距离
    double relative_yaw;       // 相对yaw角
    double relative_pitch;     // 相对pitch角
    XYZ target_world_coord;    // 目标世界坐标
    double absolute_yaw;       // IMU绝对yaw
    double absolute_pitch;     // IMU绝对pitch
    Time::TimeStamp timestamp; // 时间戳
    double target_yaw;
};
// 键盘输入处理
class KeyboardInput {
private:
    int original_flags;
    struct termios original_termios;
    
public:
    KeyboardInput() {
        // 设置非阻塞键盘输入
        original_flags = fcntl(STDIN_FILENO, F_GETFL);
        fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
        
        tcgetattr(STDIN_FILENO, &original_termios);
        struct termios new_termios = original_termios;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }
    
    ~KeyboardInput() {
        // 恢复原始设置
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
        fcntl(STDIN_FILENO, F_SETFL, original_flags);
    }
    
    char getKey() {
        char key;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            return key;
        }
        return 0; // 无输入
    }
};

// 保存射表记录到文件
void saveShootingRecord(const ShootingRecord& record, const std::string& filename) {
    std::ofstream file(filename, std::ios::app);
    if (file.is_open()) {
        // 使用当前时间戳转换为秒数
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        
        file << std::fixed << std::setprecision(6)
             << (double)time_t_now << ","
             << record.z_height << ","
             << record.horizontal_distance << ","
             << record.relative_yaw << ","
             << record.relative_pitch << ","
             << record.target_world_coord.x << ","
             << record.target_world_coord.y << ","
             << record.target_world_coord.z << ","
             << record.absolute_yaw << ","
             << record.absolute_pitch << ","
             << record.target_yaw
             << std::endl;
        file.close();
        INFO("Shooting record saved to {}", filename);
    } else {
        ERROR("Failed to open file: {}", filename);
    }
}

// 创建CSV文件头
void createCSVHeader(const std::string& filename) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << "timestamp,z_height,horizontal_distance,relative_yaw,relative_pitch,"
             << "target_x,target_y,target_z,absolute_yaw,absolute_pitch,target_yaw" << std::endl;
        file.close();
        INFO("Created CSV header in {}", filename);
    }
}

double pitch_intercept = 0;
double pitch_coef_z = 0;
double pitch_coef_d = 0;
double pitch_coef_z2 = 0;
double pitch_coef_zd = 0;
double pitch_coef_d2 = 0;
double fitPitch(double z_height, double horizontal_distance) {
    // Model 3: Full 2nd Order 参数 (PITCH)
    double intercept = pitch_intercept;
    double coef_z = pitch_coef_z;
    double coef_d = pitch_coef_d;
    double coef_z2 = pitch_coef_z2;
    double coef_zd = pitch_coef_zd;
    double coef_d2 = pitch_coef_d2;

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;
    
    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

double yaw_intercept = 0;
double yaw_coef_z = 0;
double yaw_coef_d = 0;
double yaw_coef_z2 = 0;
double yaw_coef_zd = 0;
double yaw_coef_d2 = 0;
double fitYaw(double z_height, double horizontal_distance) {
    // Model 3: Full 2nd Order 参数 (YAW)
    double intercept = yaw_intercept;
    double coef_z = yaw_coef_z;
    double coef_d = yaw_coef_d;
    double coef_z2 = yaw_coef_z2;
    double coef_zd = yaw_coef_zd;
    double coef_d2 = yaw_coef_d2;

    // 计算特征值
    double z2 = z_height * z_height;
    double d2 = horizontal_distance * horizontal_distance;
    double zd = z_height * horizontal_distance;
    
    // 完整二阶多项式: intercept + z + d + z² + z*d + d²
    return intercept + 
           coef_z * z_height + 
           coef_d * horizontal_distance + 
           coef_z2 * z2 + 
           coef_zd * zd + 
           coef_d2 * d2;
}

int main() {
    INFO("=== Shooting Table Calibration System ===");
    INFO("Controls:");
    INFO("  't' - Trigger single aim (detect target once & enable control)");
    INFO("  'SPACE' - Start shooting sequence");
    INFO("  's' - Adjust pitch down");
    INFO("  'w' - Adjust pitch up");
    INFO("  'a' - Adjust yaw left");
    INFO("  'd' - Adjust yaw right");
    INFO("  '/' - Adjust bias auto");
    INFO("  'c' - Confirm hit center (save record & disable control)");
    INFO("  'r' - Reset adjustments & disable control");
    INFO("  'q' - Quit program");
    INFO("=========================================");

    // 初始化参数
    param::Param param("../config.json");
    param = param[param["car_name"].String()];
    if(param["controller"]["shoot_table_adjust"]["enable"].Bool())
    {
        INFO("Shooting table adjustment enabled");
        pitch_intercept = param["controller"]["shoot_table_adjust"]["pitch"]["intercept"].Double();
        pitch_coef_z = param["controller"]["shoot_table_adjust"]["pitch"]["coef_z"].Double();
        pitch_coef_d = param["controller"]["shoot_table_adjust"]["pitch"]["coef_d"].Double();
        pitch_coef_z2 = param["controller"]["shoot_table_adjust"]["pitch"]["coef_z2"].Double();
        pitch_coef_zd = param["controller"]["shoot_table_adjust"]["pitch"]["coef_zd"].Double();
        pitch_coef_d2 = param["controller"]["shoot_table_adjust"]["pitch"]["coef_d2"].Double();
        yaw_intercept = param["controller"]["shoot_table_adjust"]["yaw"]["intercept"].Double();
        yaw_coef_z = param["controller"]["shoot_table_adjust"]["yaw"]["coef_z"].Double();
        yaw_coef_d = param["controller"]["shoot_table_adjust"]["yaw"]["coef_d"].Double();
        yaw_coef_z2 = param["controller"]["shoot_table_adjust"]["yaw"]["coef_z2"].Double();
        yaw_coef_zd = param["controller"]["shoot_table_adjust"]["yaw"]["coef_zd"].Double();
        yaw_coef_d2 = param["controller"]["shoot_table_adjust"]["yaw"]["coef_d2"].Double();
    }
    
    bool udp_enable = param["UDP"]["enable"].Bool();
    bool web_debug_enable = param["web_debug"].Bool();
    bool draw_debug_image = param["debug_on_image"].Bool();
    
    //if(udp_enable)
        UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());
    //else
    //    UdpSend::disable();
        
    if(web_debug_enable)
        VideoStreamer::init();

    // 创建模块
    auto driver = createDriver();
    auto solver = createSolver(param["solver"]);
    auto controller = createController(param["controller"]);
    //auto detector = createDetector("../utils/models/armor_yolo_x.xml", "../utils/models/yolo11n-416sgd4.xml", true, true);
    auto detector = createDetector(param["detector"], true);
    auto tracker = createTracker();
    detector->setEnemyColor(enemyTrans["all"]);

    location::Location::registerSolver(solver);
    
    // 配置硬件
    SerialConfig config{param["serial_name"].String(), param["baud_rate"].Int()};
    CameraConfig cameraConfig{
        .cameraSN = param["camera_id"].String(), 
        .autoWhiteBalance = param["auto_white_balance"].Bool(),
        .exposureTime = param["exposure_time"].Double(),
        .gain = param["gain"].Double()
    };
    
    driver->setSerialConfig(config);
    driver->setCameraConfig(cameraConfig);

    // 射表统计变量
    std::vector<ShootingRecord> records;
    std::string csv_filename = "../record/shooting_table_" + 
                              std::to_string(std::time(nullptr)) + ".csv";
    createCSVHeader(csv_filename);
    
    KeyboardInput keyboard;
    
    // 控制参数
    double pitch_adjustment = 0.0;
    double yaw_adjustment = 0.0;
    const double adjustment_step = 0.1; // 调整步长（度）
    bool is_shooting = false;
    Time::TimeStamp shoot_start_time;
    const double shoot_duration = 0.3; // 射击持续时间（秒）
    
    // 当前瞄准状态
    bool is_aiming = false;
    bool should_aim_once = false;  // 控制单次瞄准
    XYZ current_target_world;
    ParsedSerialData current_imu;
    double current_target_yaw = 0.0; // 当前目标yaw角度
    
    // 控制命令状态
    bool control_valid = false;    // 控制命令是否有效
    double target_yaw = 0.0;       // 目标yaw角度
    double target_pitch = 0.0;     // 目标pitch角度

    // 注册串口回调
    driver->registReadCallback([&](const ParsedSerialData& parsedData) {
        current_imu = parsedData;
        ControlResult result;
        
        if (control_valid) {
            // 发送瞄准角度加上用户调整
            result.pitch_setpoint = target_pitch + pitch_adjustment;
            result.yaw_setpoint = target_yaw + yaw_adjustment;
            result.valid = true;
        } else {
            // 控制无效时，发送当前角度（保持不动）
            result.pitch_setpoint = parsedData.pitch_now;
            result.yaw_setpoint = parsedData.yaw_now;
            result.valid = false;
        }
        
        result.shoot_flag = is_shooting ? 1 : 0;
        
        auto control_func = driver->sendSerialFunc();
        control_func(result);
    });

    // 启动线程
    driver->runSerialThread();
    driver->runCameraThread();

    INFO("System initialized. Waiting for target detection...");

    while(true) {
        // 检查键盘输入
        char key = keyboard.getKey();
        switch(key) {
            case 't':
                INFO("Triggering single aim detection...");
                should_aim_once = true;
                break;

            case ' ':
                if (is_aiming) {
                    INFO("Starting shooting sequence...");
                    is_shooting = true;
                    shoot_start_time.reset();
                } else {
                    WARN("No target locked. Press 't' to aim first.");
                }
                break;
            case '/':
                if (is_aiming) {
                    double z_height = current_target_world.z;
                    double horizontal_distance = std::sqrt(current_target_world.x * current_target_world.x + 
                                                          current_target_world.y * current_target_world.y);
                    yaw_adjustment = fitYaw(z_height, horizontal_distance);
                    pitch_adjustment = fitPitch(z_height, horizontal_distance);
                    INFO("Auto adjustment applied: Yaw: {:.2f}°, Pitch: {:.2f}°", 
                         yaw_adjustment, pitch_adjustment);
                } else {
                    WARN("No target locked. Press 't' to aim first.");
                }
                break;

            case 's':
                pitch_adjustment -= adjustment_step;
                INFO("Pitch adjustment: {:.2f}°", pitch_adjustment);
                break;
                
            case 'w':
                pitch_adjustment += adjustment_step;
                INFO("Pitch adjustment: {:.2f}°", pitch_adjustment);
                break;
                
            case 'a':
                yaw_adjustment += adjustment_step;
                INFO("Yaw adjustment: {:.2f}°", yaw_adjustment);
                break;
                
            case 'd':
                yaw_adjustment -= adjustment_step;
                INFO("Yaw adjustment: {:.2f}°", yaw_adjustment);
                break;
                
            case 'r':
                pitch_adjustment = 0.0;
                yaw_adjustment = 0.0;
                control_valid = false;  // 禁用控制
                is_aiming = false;
                INFO("Adjustments reset to zero and control disabled");
                break;
                
            case 'c':
                if (is_aiming) {
                    // 保存当前记录
                    ShootingRecord record;
                    record.z_height = current_target_world.z;
                    record.horizontal_distance = std::sqrt(current_target_world.x * current_target_world.x + 
                                                          current_target_world.y * current_target_world.y);
                    record.relative_yaw = yaw_adjustment;
                    record.relative_pitch = pitch_adjustment;
                    record.target_world_coord = current_target_world;
                    record.absolute_yaw = current_imu.yaw_now;
                    record.absolute_pitch = current_imu.pitch_now;
                    record.target_yaw = current_target_yaw;
                    record.timestamp.reset();
                    
                    saveShootingRecord(record, csv_filename);
                    records.push_back(record);
                    
                    INFO("Record saved! Z: {:.2f}m, Distance: {:.2f}m, RelYaw: {:.2f}°, RelPitch: {:.2f}°", 
                         record.z_height, record.horizontal_distance, record.relative_yaw, record.relative_pitch);
                    
                    // 重置所有状态，准备下一个目标
                    is_aiming = false;
                    control_valid = false;  // 禁用控制
                    INFO("Control disabled. Press 't' to aim at next target.");
                } else {
                    WARN("No target locked. Press 't' to aim first.");
                }
                break;
                
            case 'q':
                INFO("Exiting shooting table calibration...");
                goto exit_loop;
                
            default:
                break;
        }

        // 检查射击时间
        if (is_shooting) {
            auto current_time = Time::TimeStamp();
            current_time.reset();
            if ((current_time - shoot_start_time).toSeconds() >= shoot_duration) {
                is_shooting = false;
                INFO("Shooting sequence completed. Adjust aim and press 'c' when centered.");
            }
        }

        // 处理相机数据
        if(!driver->isExistNewCameraData()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            continue;
        }

        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        auto frame = camera_data_pack.back();
        
        auto imu = driver->findNearestSerialData(frame->timestamp);
        ImuData imu_data = imu;
        driver->clearSerialData();

        // 检测装甲板
        auto detections = detector->detect(frame->image);
        tracker->merge(detections.first);
        
        auto trackResults = tracker->getTrackResult(frame->timestamp, imu);
        
        XYZ xyz_imu;
        if(!trackResults.first.empty())
        {
            auto& trackResult = trackResults.first[0]; // 选择第一个目标
            double yaw = 0;
            std::tie(xyz_imu, yaw) = solver->camera2world(trackResult.armor, imu_data, trackResult.car_id == 1);
            
            current_target_world = xyz_imu;
            current_target_yaw = yaw;
            float distance = std::sqrt(xyz_imu.x * xyz_imu.x + xyz_imu.y * xyz_imu.y);
            UdpSend::sendData(distance);
        }
        // 如果检测到装甲板且需要进行单次瞄准
        if (!trackResults.first.empty() && should_aim_once) {
            // 通过controller获取ballistic calculator并计算（这样会自动应用射击表补偿）
            // 注意：这里假设可以访问controller的ballistic_calculator_，
            // 或者需要在controller中添加公共接口来计算弹道
            
            // 创建临时的弹道计算（这里仍需要手动应用射击表补偿）
            const double PI = 3.1415926;
            const double GRAVITY = 9.794;
            const double C_D = 0.42;
            const double RHO = 1.169;
            const double bullet_mass = 3.2e-3;
            const double bullet_diameter = 16.8e-3;
            const double bullet_speed = 23.0;
            const double tol = 1e-6;
            const int max_iter = 100;
            
            double distance = sqrt(xyz_imu.x * xyz_imu.x + xyz_imu.y * xyz_imu.y);
            double theta = 0.0;
            double delta_z = 0.0;
            double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
            
            // 迭代计算pitch角
            bool calc_success = false;
            for (int i = 0; i < max_iter; i++) {
                double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));
                delta_z = xyz_imu.z - bullet_speed * sin(theta) * t / cos(theta) + 0.5 * GRAVITY * t * t / cos(theta) / cos(theta);
                
                if (fabs(delta_z) < tol) {
                    calc_success = true;
                    break;
                }
                
                theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + GRAVITY * t * t / (bullet_speed * bullet_speed) * sin(theta) / pow(cos(theta), 3));
            }
            
            if (calc_success) {
                double aim_pitch_rad = theta;
                double aim_yaw_rad = atan2(xyz_imu.y, xyz_imu.x);
                
                // // 应用射击表补偿（如果启用）
                // if(param["controller"]["shoot_table_adjust"]["enable"].Bool()) {
                //     double horizontal_distance = sqrt(xyz_imu.x * xyz_imu.x + xyz_imu.y * xyz_imu.y);
                //     aim_pitch_rad += fitPitch(xyz_imu.z, horizontal_distance) * PI / 180.0;
                //     aim_yaw_rad += fitYaw(xyz_imu.z, horizontal_distance) * PI / 180.0;
                // }
                
                // 转换为角度制
                double aim_pitch_deg = aim_pitch_rad * 180 / PI;
                double aim_yaw_deg = aim_yaw_rad * 180 / PI;
                
                // 处理yaw角度连续性
                aim_yaw_deg = imu_data.yaw + std::remainder(aim_yaw_deg - imu_data.yaw, 360.0);
                
                target_yaw = aim_yaw_deg;
                target_pitch = aim_pitch_deg;
                control_valid = true;
                
                INFO("Target locked! Distance: {:.2f}m, Z: {:.2f}m - Aim angles: Y:{:.2f}°, P:{:.2f}° - Control enabled", 
                     distance, xyz_imu.z, aim_yaw_deg, aim_pitch_deg);
            } else {
                WARN("Ballistic calculation failed, using fallback angles");
                target_yaw = imu_data.yaw;
                target_pitch = imu_data.pitch;
                control_valid = true;
            }
            
            is_aiming = true;
            should_aim_once = false;  // 重置单次瞄准标志
        } else if (should_aim_once && trackResults.first.empty()) {
            WARN("No target detected. Try again with 't' key.");
            should_aim_once = false;
        }

        // 绘制调试信息
        if(draw_debug_image) {
            // 绘制十字准心
            int center_x = frame->image.cols / 2;
            int center_y = frame->image.rows / 2;
            cv::line(frame->image, cv::Point(center_x - 20, center_y), 
                     cv::Point(center_x + 20, center_y), cv::Scalar(0, 255, 0), 2);
            cv::line(frame->image, cv::Point(center_x, center_y - 20), 
                     cv::Point(center_x, center_y + 20), cv::Scalar(0, 255, 0), 2);
            
            // 显示调整信息和控制状态
            std::string adj_text = "Adj Y:" + std::to_string(yaw_adjustment) + 
                                  " P:" + std::to_string(pitch_adjustment) +
                                  (control_valid ? " [CTRL ON]" : " [CTRL OFF]");
            cv::putText(frame->image, adj_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            
            // 显示状态
            std::string status;
            if (is_shooting) {
                status = "SHOOTING";
            } else if (is_aiming) {
                status = "TARGET LOCKED";
            } else if (should_aim_once) {
                status = "AIMING...";
            } else {
                status = "READY - Press 't' to aim";
            }
            cv::putText(frame->image, status, cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
            // 绘制目标
            for(auto& trackResult : trackResults.first) {
                CXYD coord = trackResult.location.cxy;
                cv::circle(frame->image, cv::Point(coord.cx, coord.cy), 12, 
                          cv::Scalar(255, 0, 0), 3);
                
                // 正确访问xyz_imu
                XYZ xyz_data = trackResult.location.xyz_imu;
                std::string dist_text = std::to_string(
                    (int)std::sqrt(xyz_data.x * xyz_data.x + xyz_data.y * xyz_data.y)) + "m";
                cv::putText(frame->image, dist_text, cv::Point(coord.cx, coord.cy - 20), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            }
        }

        if(web_debug_enable)
            VideoStreamer::setFrame(frame->image);
            
        if(udp_enable)
            UdpSend::sendTail();
    }

exit_loop:
    // 清理资源
    //VideoStreamer::cleanup();
    
    // 输出统计信息
    INFO("=== Shooting Table Calibration Summary ===");
    INFO("Total records: {}", records.size());
    INFO("Records saved to: {}", csv_filename);
    
    if (!records.empty()) {
        INFO("Distance range: {:.2f}m - {:.2f}m", 
             std::min_element(records.begin(), records.end(), 
                 [](const auto& a, const auto& b) { return a.horizontal_distance < b.horizontal_distance; })->horizontal_distance,
             std::max_element(records.begin(), records.end(), 
                 [](const auto& a, const auto& b) { return a.horizontal_distance < b.horizontal_distance; })->horizontal_distance);
    }
    
    INFO("Calibration completed.");
    return 0;
}
