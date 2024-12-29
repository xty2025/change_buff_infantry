#include "driver.hpp"
#include "DaHengCamera.hpp"
#include <thread>
#include <atomic>
#include <mutex>

namespace camera
{
    class Camera : public driver::Camera
    {
    public:
        void setCameraConfig(CameraConfig config) override;
        void runCameraThread() override;
        bool isExistNewCameraData() override;
        void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) override;
    private:
        int max_camera_data_queue_size_ = 100;
        std::atomic<bool> running_camera_;
        std::mutex camera_data_mutex_;
        std::thread camera_thread_;
        std::queue<std::shared_ptr<TimeImageData>> camera_data_pack_;
        std::unique_ptr<DaHengCamera> daheng_camera = std::make_unique<DaHengCamera>();
        std::atomic<bool> existNewCameraData;
    };
    //A noneed statement only for reminding you.
    using driver::createCamera;
}