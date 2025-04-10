#include "driver.hpp"
using namespace driver;


auto driver::createDriver() -> std::unique_ptr<Driver> 
{ 
    return std::make_unique<driver::Driver>(createSerial(), createCamera()); 
}

void Driver::setSerialConfig(SerialConfig config)
{
    this->serial->setSerialConfig(config);
}

void Driver::setCameraConfig(CameraConfig config)
{
    this->camera->setCameraConfig(config);
}

std::function<void(const ControlResult&)> Driver::sendSerialFunc()
{
    return this->serial->sendSerialFunc();
}

void Driver::registReadCallback(std::function<void(const ParsedSerialData&)> callback)
{
    this->serial->registReadCallback(callback);
}

void Driver::runSerialThread()
{
    this->serial->runSerialThread();
}

void Driver::runCameraThread()
{
    this->camera->runCameraThread();
}

bool Driver::isExistNewCameraData()
{
    return this->camera->isExistNewCameraData();
}

void Driver::getCameraData(std::queue<std::shared_ptr<TimeImageData>>& camera_data_pack)
{
    return this->camera->getCameraData(camera_data_pack);
}

ParsedSerialData Driver::findNearestSerialData(const Time::TimeStamp& timestamp)
{
    return this->serial->findNearestSerialData(timestamp);
}

void Driver::clearSerialData()
{
    this->serial->clearSerialData();
}

RawSerialWriteData::RawSerialWriteData(const ControlResult& x)
: start_flag('!'), detect_number(static_cast<int>(x.valid)), shoot_flag(x.shoot_flag), 
            pitch_setpoint(x.pitch_setpoint), yaw_setpoint(x.yaw_setpoint) {}
