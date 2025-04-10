#include "serial.hpp"
#include "Log/log.hpp"
#include "CRC16.h"
#include <termios.h>
using namespace serial;
using namespace aimlog;
using namespace boost::asio;

auto serial::createSerial() -> std::unique_ptr<Serial> 
{ 
    return std::make_unique<serial::Serial>(); 
}

Serial::~Serial() {
    stopSerialThread();
    try{serial_port_.close();}
    catch(const boost::system::system_error& e){
        ERROR("Error during close serial port: {}", e.what());
    }
}

void Serial::setSerialConfig(SerialConfig config) {
    try{
    serial_port_.open(config.portName);
    serial_port_.set_option(serial_port_base::baud_rate(config.baudRate));
    serial_port_.set_option(serial_port_base::character_size(8));
    serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::none));
    serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
    int fd = serial_port_.native_handle();
    termios options;
    tcgetattr(fd, &options);

    // 禁用缓冲
    options.c_lflag &= ~ICANON;
    options.c_lflag &= ~ECHO;
    options.c_lflag &= ~ECHOE;
    options.c_lflag &= ~ISIG;
    options.c_oflag &= ~OPOST;

    // 设置立即写入
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 0;

    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIOFLUSH);
    }catch(const boost::system::system_error& e){
        ERROR("Error during open serial port '{}': {}", config.portName, e.what());
    }
}

void Serial::registReadCallback(std::function<void(const ParsedSerialData&)> callback) {
    read_callback_ = std::move(callback);
}

void Serial::runSerialThread() {
    running_ = true;
    serial_thread_ = std::thread([this]() {
        startAsyncRead();
        io_context_.run();
    });
}

void Serial::stopSerialThread() {
    running_ = false;
    io_context_.stop();
    if (serial_thread_.joinable()) {
        serial_thread_.join();
    }
}

std::function<void(const serial::ControlResult&)> Serial::sendSerialFunc() {
    return [this](const ControlResult& result) {
        std::lock_guard<std::mutex> lock(write_mutex_);
        ControlResult result_ = result;
        // Serialize and send data (implementation depends on data structure)
                //static Param param("../config.json");
        // static bool first =true;
        // if(first)
        // {param = param[param["car_name"].String()];
        //   first = false;
        // }
        //result_.pitch_setpoint -= param["pitch_begin_offset"].Double();
        RawSerialWriteData data = serializeControlResult(result_);
        auto write_buffer = std::make_shared<std::vector<uint8_t>>(reinterpret_cast<uint8_t*>(&data), reinterpret_cast<uint8_t*>(&data) + sizeof(data));
        // ASYNC form is dangerous, if you send data too fast, the memory will take up too much
        // async_write(serial_port_, buffer(*write_buffer),
        //              [write_buffer](const boost::system::error_code& ec, std::size_t /*bytes_transferred*/) {
        //                  if (ec) {
        //                      ERROR("Error during write: {}", ec.message());
        //                  }
        //              });
        boost::system::error_code ec;
        write(serial_port_, buffer(*write_buffer), ec);
        INFO("send pitch: {}", result.pitch_setpoint);
        INFO("send yaw: {}", result.yaw_setpoint);
        if (ec) {
            ERROR("Error during write: {}", ec.message());
        }
    };
}


serial::ParsedSerialData Serial::findNearestSerialData(const Time::TimeStamp& timestamp) {
    std::lock_guard<std::mutex> lock(serial_data_mutex_);
    // Simple nearest data search
    if (serial_data_queue_.empty()) {
        return {};  // Return empty data if no serial data is available
    }
    auto nearest_data = serial_data_queue_.front();
    while(!serial_data_queue_.empty()){
        auto data = serial_data_queue_.front();
        if (abs((data.timestamp - timestamp).count()) < abs((nearest_data.timestamp - timestamp).count())) {
            nearest_data = data;
        }
        serial_data_queue_.pop();
    }
    return nearest_data;
}

void Serial::clearSerialData() {
    std::lock_guard<std::mutex> lock(serial_data_mutex_);
    serial_data_queue_ = std::queue<ParsedSerialData>();
}


void Serial::startAsyncRead() {
    auto raw_data = std::make_shared<RawSerialData>();
    async_read(serial_port_, buffer(raw_data.get(), sizeof(RawSerialData)),
            [this, raw_data](const boost::system::error_code& ec, std::size_t bytes_transferred) {
                if (!ec && bytes_transferred == sizeof(RawSerialData)) {
                    if (raw_data->start_flag == '!') 
                        handleReadData(*raw_data);
                    else 
                    {
                        ERROR("Misaligned data, searching for start flag...");
                        // Blocking read until start flag is found
                        uint8_t byte;
                        boost::system::error_code ec_inner;
                        do {
                            read(serial_port_, buffer(&byte, 1), ec_inner);
                            if (ec_inner)
                                ERROR("Error during read: {}", ec_inner.message());
                            else
                                INFO("Skipping byte: {}", byte);
                        } while (byte != '!');
                        // Found the start flag, read the rest of the data (block)
                        read(serial_port_, buffer(reinterpret_cast<uint8_t*>(raw_data.get()) + 1, sizeof(RawSerialData) - 1), ec_inner);
                        if (!ec_inner) 
                        {
                            raw_data->start_flag = '!';
                            handleReadData(*raw_data);
                        } 
                        else 
                            ERROR("Error during read: {}", ec_inner.message());
                    }
                } 
                else 
                    ERROR("Error during read: {}", ec.message());
                startAsyncRead();
            });
}

void Serial::handleReadData(RawSerialData& raw_data) {
    if (!enable_CRC_check || validateCRC(raw_data)) {
        ParsedSerialData parsed_data(raw_data);
        //static Param param("../config.json");
        // static bool first =true;
        // if(first)
        // {param = param[param["car_name"].String()];
        //   first = false;
        // }
        // parsed_data.pitch_now += param["pitch_begin_offset"].Double();
        parsed_data.timestamp.reset();

        {
            std::lock_guard<std::mutex> lock(serial_data_mutex_);
            while (serial_data_queue_.size() >= max_serial_data_queue_size_) 
                serial_data_queue_.pop();
            serial_data_queue_.push(parsed_data);
        }
        if (read_callback_) {
            read_callback_(parsed_data);
        }
    }
    else {
        ERROR("CRC check failed");
    }
}

bool Serial::validateCRC(RawSerialData& data) {
    return verifyCRC16(&data);
}

void Serial::addCRC(RawSerialWriteData& data) {
    addCRC16(&data);
}

serial::RawSerialWriteData Serial::serializeControlResult(const ControlResult& data) {
    RawSerialWriteData result(data);
    addCRC(result);
    return result;
}