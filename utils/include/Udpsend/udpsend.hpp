// filepath: /root/AutoAim/utils/include/Udpsend/udpsend.hpp
#pragma once
#include <boost/asio.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <stdexcept>
#include <Log/log.hpp>  // 使用 ERROR 宏
#include <Param/param.hpp>
/*
static void disable();//禁用UDP：
// 初始化UDP连接
static UdpSend& instance(const std::string & ip = "", int port = 0);
UdpSend::instance(param["UDP"]["ip"].String(), param["UDP"]["port"].Int());

// 获取实例（用于内部调用）
UdpSend::instance().bufferData(data);
//发数据：
static void sendData(const float& data);
// 发送云台控制数据
UdpSend::sendData((float)result.pitch_setpoint);
UdpSend::sendData((float)result.yaw_setpoint);
UdpSend::sendData((float)parsedData.pitch_now);
UdpSend::sendData((float)parsedData.yaw_now);
static void sendTail();

#private :
UdpSend(const std::string & ip, int port);
//统一的调用方式：
UdpSend::functionName(parameters);
*/

#include <boost/asio.hpp>
#include<future>
//#include<async>
//boost通信库::
class UdpSend {
private :
    boost::asio::io_context io_context_;//异步IO
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;
    /*
    async::executor_work_guard<boost::asio::io_context::executor_type> work_guard_{io_context_.get_executor()};
    future<void> io_thread_ = async::make_future([this]() { io_context_.run(); });
    this_thread::sleep_for(100ms); // 确保 io_context 线程启动
    */
    inline static std::unique_ptr<UdpSend> s_instance = nullptr;
    //class ptr;
    inline static std::mutex s_mutex;
    inline static bool _disable = false;

    std::vector<float> buffer_;  // 缓冲区
    std::mutex buffer_mutex_;    // 缓冲区互斥锁
    const size_t buffer_limit_ = 1024;  // 缓冲区大小限制
    
    //buffer_.resize(buffer_limit_, 0);  // 后面再填充

public:
    UdpSend(const UdpSend&) = delete;
    UdpSend& operator=(const UdpSend&) = delete;

    static void disable() {
        _disable = true;
    }

    static UdpSend& instance(const std::string & ip = "", int port = 0) {
        //inline static std::mutex s_mutex;
        //单锁保护
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!s_instance) {//初始s_instance->nullptr;
            if(ip.empty() || port == 0) {
                ERROR("UdpSend 未初始化，请提供有效的 ip 和 port");
            }
            else
                s_instance.reset(new UdpSend(ip, port));
        } else if (!ip.empty() && port != 0) {
            s_instance->updateConnection(ip, port);
            //调用打开connection的配置。
            //找第一个出现的IPv4地址。并传入endpoint和socket;
        }
        return *s_instance;
    }
    //instance()建立联系，instance()->updateConnection()。
    //sendData和sendTAIL,调用instance().bufferData();发送和清除。
    //instance().bufferData()->flushBuffer();
    static void sendData(const float& data) {
        if(_disable) {WARN("UdpSend Disabled!"); return;}
        instance().bufferData(data);
    }

    static void sendTail() {
        if(_disable) {WARN("UdpSend Disabled!"); return;}
        unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
        //末置位校验，发送一个float的NaN值作为结束标志
        instance().bufferData(reinterpret_cast<const float*>(tail), true);
    }

private:
    UdpSend(const std::string & ip, int port)
            : io_context_(), socket_(io_context_)
    {
        updateConnection(ip, port);
    }

    void updateConnection(const std::string & ip, int port) {
        try {
            boost::asio::ip::udp::resolver resolver(io_context_);
            auto endpoints = resolver.resolve(boost::asio::ip::udp::v4(), ip, std::to_string(port));
            // 仅使用第一个解析到的端点（IPV4），未打开就打开第一个的UDP-Socket；
            endpoint_ = *endpoints.begin();
            if (!socket_.is_open()) {
                socket_.open(boost::asio::ip::udp::v4());
            }
        } catch (const std::exception & e) {
            ERROR("UdpSend updateConnection 错误: {}", e.what());
        }
    }

    void bufferData(const float& data, bool forceSend = false) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_.push_back(data);

        if (forceSend || buffer_.size() >= buffer_limit_) {
            flushBuffer();//释放所有的buffer空间
        }
    }

    void bufferData(const float* data, bool forceSend = false) {
        std::lock_guard<std::mutex> lock(buffer_mutex_);
        buffer_.insert(buffer_.end(), data, data + 1);

        if (forceSend || buffer_.size() >= buffer_limit_) {
            flushBuffer();
        }
    }

    void flushBuffer() {
        if (buffer_.empty()) return;

        boost::system::error_code ec;
        auto bytes_sent = socket_.send_to(boost::asio::buffer(buffer_.data(), buffer_.size() * sizeof(float)), endpoint_, 0, ec);
        if (ec) {
            ERROR("UdpSend 发送数据错误: {}", ec.message());
        }
        buffer_.clear();
    }
};