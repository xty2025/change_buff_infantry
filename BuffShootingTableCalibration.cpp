#include <array>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

#include <modules/modules.hpp>
#include <opencv2/opencv.hpp>
#include <Udpsend/udpsend.hpp>
#include <Param/param.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <Log/log.hpp>
#include <VideoStreamer/VideoStreamer.hpp>
#include <Recorder/recorder.hpp>
#include <Location/location.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace aimlog;
using namespace modules;

namespace fs = std::filesystem;

namespace {

struct BuffShootingRecord {
    double rotation_angle = 0.0;
    double horizontal_distance = 0.0;
    double height = 0.0;
    double relative_yaw = 0.0;
    double relative_pitch = 0.0;
    cv::Point3f target_robot{};
    double absolute_yaw = 0.0;
    double absolute_pitch = 0.0;
};

struct PeriodicAdjustConfig {
    bool enable = false;
    std::array<double, 7> pitch{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    std::array<double, 7> yaw{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

struct SharedControlState {
    std::mutex mutex;
    bool control_valid = false;
    bool shoot_flag = false;
    double target_pitch = 0.0;
    double target_yaw = 0.0;
    double pitch_adjustment = 0.0;
    double yaw_adjustment = 0.0;
};

class KeyboardInput {
public:
    KeyboardInput() {
        original_flags_ = fcntl(STDIN_FILENO, F_GETFL);
        fcntl(STDIN_FILENO, F_SETFL, original_flags_ | O_NONBLOCK);

        tcgetattr(STDIN_FILENO, &original_termios_);
        struct termios new_termios = original_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
    }

    ~KeyboardInput() {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        fcntl(STDIN_FILENO, F_SETFL, original_flags_);
    }

    char getKey() const {
        char key = 0;
        if (read(STDIN_FILENO, &key, 1) == 1) {
            return key;
        }
        return 0;
    }

private:
    int original_flags_ = 0;
    struct termios original_termios_{};
};

double fitBuffPeriodicDeg(const std::array<double, 7>& coeffs, double rotation_angle,
                         double distance_m, double height_m) {
    return coeffs[0] +
           coeffs[1] * std::sin(rotation_angle) +
           coeffs[2] * std::cos(rotation_angle) +
           coeffs[3] * std::sin(2.0 * rotation_angle) +
           coeffs[4] * std::cos(2.0 * rotation_angle) +
           coeffs[5] * distance_m +
           coeffs[6] * height_m;
}

PeriodicAdjustConfig loadAdjustConfig(const param::Param& param) {
    PeriodicAdjustConfig config;
    if (!param["buff"].exists("buff_shooting_table_calib")) {
        return config;
    }

    auto calib = param["buff"]["buff_shooting_table_calib"];
    config.enable = calib["enable"].Bool();
    if (!config.enable) {
        return config;
    }

    const char* keys[] = {"intercept", "coef_sin", "coef_cos", "coef_sin2",
                          "coef_cos2", "coef_dist", "coef_height"};
    for (std::size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
        config.pitch[i] = calib["pitch"][keys[i]].Double();
        config.yaw[i] = calib["yaw"][keys[i]].Double();
    }
    return config;
}

void createCSVHeader(const std::string& filename) {
    std::ofstream file(filename);
    file << "timestamp,rotation_angle,horizontal_distance,height,relative_yaw,relative_pitch,"
            "target_x,target_y,target_z,absolute_yaw,absolute_pitch\n";
}

void saveRecord(const BuffShootingRecord& record, const std::string& filename) {
    std::ofstream file(filename, std::ios::app);
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    file << std::fixed << std::setprecision(6)
         << static_cast<double>(now) << ","
         << record.rotation_angle << ","
         << record.horizontal_distance << ","
         << record.height << ","
         << record.relative_yaw << ","
         << record.relative_pitch << ","
         << record.target_robot.x << ","
         << record.target_robot.y << ","
         << record.target_robot.z << ","
         << record.absolute_yaw << ","
         << record.absolute_pitch << "\n";
}

}  // namespace

int main() {
    INFO("=== Big Buff Moving Compensation Calibration ===");
    INFO("Controls:");
    INFO("  t : start or resume tracking");
    INFO("  space : start a short shooting pulse");
    INFO("  w/s : adjust pitch");
    INFO("  a/d : adjust yaw");
    INFO("  / : apply current buff.buff_shooting_table_calib");
    INFO("  c : save current compensation sample");
    INFO("  r : reset adjustment and stop tracking");
    INFO("  q : quit");

    param::Param param("../config.json");
    param = param[param["car_name"].String()];

    const bool udp_enable = param["UDP"]["enable"].Bool();
    const bool web_debug_enable = param["web_debug"].Bool();
    const bool draw_debug_image = param["debug_on_image"].Bool();
    const bool draw_overlay = web_debug_enable || draw_debug_image;

    if (udp_enable) {
        UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());
    } else {
        UdpSend::disable();
    }
    if (web_debug_enable) {
        VideoStreamer::init();
    }

    fs::create_directories("../record");
    const std::string csv_filename =
        "../record/big_buff_moving_compensation_" + std::to_string(std::time(nullptr)) + ".csv";
    createCSVHeader(csv_filename);

    auto driver = createDriver();
    auto solver = createSolver(param["solver"]);
    location::Location::registerSolver(solver);

    std::string red_buff_model_path = param["buff"]["red_buff_model_path"].String();
    std::string blue_buff_model_path = param["buff"]["blue_buff_model_path"].String();
    BuffDetector buff_detector(red_buff_model_path, blue_buff_model_path);
    BuffCalculator buff_calculator(param);
    buff_calculator.setApplyStaticShootTableAdjust(true);
    buff_calculator.setApplyPeriodicShootTableAdjust(false);
    const auto adjust_config = loadAdjustConfig(param);

    SerialConfig serial_config{param["serial_name"].String(), param["baud_rate"].Int()};
    CameraConfig camera_config{
        .cameraSN = param["camera_id"].String(),
        .autoWhiteBalance = param["auto_white_balance"].Bool(),
        .exposureTime = param["exposure_time"].Double(),
        .gain = param["gain"].Double(),
    };
    driver->setSerialConfig(serial_config);
    driver->setCameraConfig(camera_config);

    auto shared_state = std::make_shared<SharedControlState>();
    driver->registReadCallback([control_func = driver->sendSerialFunc(), shared_state](const ParsedSerialData& parsedData) {
        ControlResult result;
        {
            std::lock_guard<std::mutex> lock(shared_state->mutex);
            if (shared_state->control_valid) {
                result.pitch_setpoint = shared_state->target_pitch + shared_state->pitch_adjustment;
                result.yaw_setpoint = shared_state->target_yaw + shared_state->yaw_adjustment;
                result.valid = true;
            } else {
                result.pitch_setpoint = parsedData.pitch_now;
                result.yaw_setpoint = parsedData.yaw_now;
                result.valid = false;
            }
            result.shoot_flag = shared_state->shoot_flag ? 1 : 0;
        }
        control_func(result);
    });

    driver->runSerialThread();
    driver->runCameraThread();

    KeyboardInput keyboard;

    constexpr int kBigBuffMode = 2;
    constexpr double kAdjustmentStep = 0.1;
    constexpr double kShootDurationSec = 0.3;
    bool tracking_enabled = false;
    bool is_aiming = false;
    bool have_target = false;
    bool reload_big_buff = true;
    std::chrono::steady_clock::time_point shoot_start_time{};

    BuffShootingRecord current_record;
    ParsedSerialData current_imu;

    while (true) {
        const char key = keyboard.getKey();
        switch (key) {
            case 't':
                tracking_enabled = true;
                INFO("Tracking enabled");
                break;
            case ' ':
                if (have_target) {
                    std::lock_guard<std::mutex> lock(shared_state->mutex);
                    shared_state->shoot_flag = true;
                    shoot_start_time = std::chrono::steady_clock::now();
                    INFO("Shoot pulse started");
                } else {
                    WARN("No buff target available");
                }
                break;
            case 'w': {
                std::lock_guard<std::mutex> lock(shared_state->mutex);
                shared_state->pitch_adjustment += kAdjustmentStep;
                INFO("Pitch adjustment: {:.2f}", shared_state->pitch_adjustment);
                break;
            }
            case 's': {
                std::lock_guard<std::mutex> lock(shared_state->mutex);
                shared_state->pitch_adjustment -= kAdjustmentStep;
                INFO("Pitch adjustment: {:.2f}", shared_state->pitch_adjustment);
                break;
            }
            case 'a': {
                std::lock_guard<std::mutex> lock(shared_state->mutex);
                shared_state->yaw_adjustment += kAdjustmentStep;
                INFO("Yaw adjustment: {:.2f}", shared_state->yaw_adjustment);
                break;
            }
            case 'd': {
                std::lock_guard<std::mutex> lock(shared_state->mutex);
                shared_state->yaw_adjustment -= kAdjustmentStep;
                INFO("Yaw adjustment: {:.2f}", shared_state->yaw_adjustment);
                break;
            }
            case '/':
                if (have_target && adjust_config.enable) {
                    std::lock_guard<std::mutex> lock(shared_state->mutex);
                    shared_state->pitch_adjustment = fitBuffPeriodicDeg(
                        adjust_config.pitch, current_record.rotation_angle,
                        current_record.horizontal_distance, current_record.height);
                    shared_state->yaw_adjustment = fitBuffPeriodicDeg(
                        adjust_config.yaw, current_record.rotation_angle,
                        current_record.horizontal_distance, current_record.height);
                    INFO("Applied fit adjustment: pitch={:.2f}, yaw={:.2f}",
                         shared_state->pitch_adjustment, shared_state->yaw_adjustment);
                } else {
                    WARN("No target or buff.buff_shooting_table_calib is disabled");
                }
                break;
            case 'c':
                if (have_target) {
                    {
                        std::lock_guard<std::mutex> lock(shared_state->mutex);
                        current_record.relative_pitch = shared_state->pitch_adjustment;
                        current_record.relative_yaw = shared_state->yaw_adjustment;
                    }
                    current_record.absolute_pitch = current_imu.pitch_now;
                    current_record.absolute_yaw = current_imu.yaw_now;
                    saveRecord(current_record, csv_filename);
                    INFO("Saved record: angle={:.3f}, dist={:.3f}m, height={:.3f}m, pitch={:.2f}, yaw={:.2f}",
                         current_record.rotation_angle, current_record.horizontal_distance,
                         current_record.height, current_record.relative_pitch, current_record.relative_yaw);

                    tracking_enabled = false;
                    is_aiming = false;
                    have_target = false;
                    std::lock_guard<std::mutex> lock(shared_state->mutex);
                    shared_state->control_valid = false;
                    shared_state->shoot_flag = false;
                } else {
                    WARN("No target locked");
                }
                break;
            case 'r': {
                tracking_enabled = false;
                is_aiming = false;
                have_target = false;
                std::lock_guard<std::mutex> lock(shared_state->mutex);
                shared_state->control_valid = false;
                shared_state->shoot_flag = false;
                shared_state->pitch_adjustment = 0.0;
                shared_state->yaw_adjustment = 0.0;
                INFO("Tracking stopped and adjustments reset");
                break;
            }
            case 'q':
                INFO("Exiting big buff moving compensation calibration");
                return 0;
            default:
                break;
        }

        {
            std::lock_guard<std::mutex> lock(shared_state->mutex);
            if (shared_state->shoot_flag &&
                std::chrono::duration<double>(std::chrono::steady_clock::now() - shoot_start_time).count() >=
                    kShootDurationSec) {
                shared_state->shoot_flag = false;
                INFO("Shoot pulse finished");
            }
        }

        if (!driver->isExistNewCameraData()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
            continue;
        }

        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack;
        driver->getCameraData(camera_data_pack);
        auto frame = camera_data_pack.back();
        current_imu = driver->findNearestSerialData(frame->timestamp);
        driver->clearSerialData();

        const bool detected = buff_detector.buffDetect(frame->image, current_imu.enemy_color);
        if (draw_overlay) {
            buff_detector.drawDebugOverlay(frame->image, true);
        }

        if (tracking_enabled && detected) {
            auto buff_camera_points = buff_detector.getCameraPointsByIndex(0);
            if (buff_camera_points.size() == 6) {
                buff_calculator.buff_frame.set(frame->image, std::chrono::steady_clock::now(),
                                               current_imu.pitch_now, current_imu.yaw_now, current_imu.roll_now);
                const bool ok = buff_calculator.calculate(
                    buff_calculator.buff_frame,
                    buff_camera_points,
                    kBigBuffMode,
                    current_imu.actual_bullet_speed > 20.0f ? current_imu.actual_bullet_speed : 24.0f,
                    reload_big_buff);

                if (ok) {
                    current_record.rotation_angle = buff_calculator.getPredictRotationAngle();
                    current_record.horizontal_distance = buff_calculator.getPredictDistanceM();
                    current_record.height = buff_calculator.getPredictHeightM();
                    current_record.target_robot = buff_calculator.getPredictRobot();

                    {
                        std::lock_guard<std::mutex> lock(shared_state->mutex);
                        shared_state->target_pitch = buff_calculator.get_predictPitch();
                        shared_state->target_yaw = buff_calculator.get_predictYaw();
                        shared_state->control_valid = true;
                    }

                    have_target = true;
                    is_aiming = true;
                } else {
                    have_target = false;
                    is_aiming = false;
                    std::lock_guard<std::mutex> lock(shared_state->mutex);
                    shared_state->control_valid = false;
                }
            }
        } else if (tracking_enabled) {
            have_target = false;
            is_aiming = false;
            std::lock_guard<std::mutex> lock(shared_state->mutex);
            shared_state->control_valid = false;
        }

        if (draw_overlay) {
            std::string status = tracking_enabled ? (is_aiming ? "TRACKING" : "SEARCHING") : "IDLE";
            cv::putText(frame->image, "BUFF CALIB " + status, cv::Point(20, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            if (have_target) {
                std::lock_guard<std::mutex> lock(shared_state->mutex);
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2)
                    << "pitch_adj=" << shared_state->pitch_adjustment
                    << " yaw_adj=" << shared_state->yaw_adjustment
                    << " angle=" << current_record.rotation_angle;
                cv::putText(frame->image, oss.str(), cv::Point(20, 60),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
            }
        }

        if (web_debug_enable) {
            VideoStreamer::setFrame(frame->image);
        }
        if (udp_enable) {
            UdpSend::sendTail();
        }
    }
}
