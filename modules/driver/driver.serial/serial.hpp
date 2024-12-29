#include "driver.hpp"
#include <thread>
#include <queue>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
// #include <spdlog/spdlog.h>
// #include <spdlog/sinks/stdout_color_sinks.h>

namespace serial
{
    class Serial : public driver::Serial
    {
        public:
            Serial() : io_context_(), serial_port_(io_context_), running_(false) {};
            ~Serial();
            void setSerialConfig(SerialConfig config) override;
            std::function<void(const ControlResult&)> sendSerialFunc() override;
            void registReadCallback(std::function<void(const ParsedSerialData&)> callback) override;
            void runSerialThread() override;
            ParsedSerialData findNearestSerialData(const Time::TimeStamp& timestamp) override;
            void clearSerialData() override;
        private:
            int max_serial_data_queue_size_ = 1000;
            bool enable_CRC_check = true;
            void stopSerialThread();
            //std::shared_ptr<spdlog::logger> logger;
            boost::asio::io_context io_context_;
            boost::asio::serial_port serial_port_;
            std::thread serial_thread_;
            std::atomic<bool> running_;
            std::mutex serial_data_mutex_;
            std::mutex write_mutex_;
            std::queue<ParsedSerialData> serial_data_queue_;

            std::function<void(const ParsedSerialData&)> read_callback_;

            std::atomic<bool> existNewCameraData;

            void startAsyncRead();
            void handleReadData(RawSerialData& raw_data);
            bool validateCRC(RawSerialData &data);
            void addCRC(RawSerialWriteData& data);

            RawSerialWriteData serializeControlResult(const ControlResult& data);
    };

    //A noneed statement only for reminding you.
    using driver::createSerial;

}