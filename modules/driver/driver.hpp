#include "modules/interfaceType.hpp"
#include "modules/modules.hpp"

namespace driver
{
    class Serial
    {
    public:
        virtual void setSerialConfig(SerialConfig config) = 0;
        virtual std::function<void(const ControlResult&)> sendSerialFunc() = 0;
        virtual void registReadCallback(std::function<void(const ParsedSerialData&)> callback) = 0;
        virtual void runSerialThread() = 0;
        virtual ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp) = 0;
        virtual void clearSerialData() = 0;
    };
    std::unique_ptr<Serial> createSerial();

    class Camera
    {
    public:
        virtual void setCameraConfig(CameraConfig config) = 0;
        virtual void runCameraThread() = 0;
        virtual bool isExistNewCameraData() = 0;
        virtual void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) = 0;
    };
    std::unique_ptr<Camera> createCamera();



    class Driver : public modules::Driver , public Serial, public Camera
    {
    private:
        std::unique_ptr<Serial> serial;
        std::unique_ptr<Camera> camera;
    public:
        Driver(std::unique_ptr<Serial> serial, std::unique_ptr<Camera> camera): serial(std::move(serial)), camera(std::move(camera)) {};
        void setSerialConfig(SerialConfig config) override;
        void setCameraConfig(CameraConfig config) override;
        std::function<void(const ControlResult&)> sendSerialFunc() override;
        void registReadCallback(std::function<void(const ParsedSerialData&)> callback) override;
        void runSerialThread() override;
        void runCameraThread() override;
        bool isExistNewCameraData() override;
        void getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack) override;
        ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp) override;
        void clearSerialData() override;
    };
    
    //A noneed statement only for reminding you.
    using modules::createDriver;
    
} // namespace driver