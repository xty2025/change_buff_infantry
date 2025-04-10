//// filepath: /root/AutoAim/utils/include/Udpsend/udpsend.hpp
#pragma once

#include <boost/asio.hpp>
#include <string>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <Log/log.hpp>  // 使用 ERROR 宏

class UdpSend {
public:
    // 删除拷贝构造和赋值函数，确保单例
    UdpSend(const UdpSend&) = delete;
    UdpSend& operator=(const UdpSend&) = delete;

    static void disable() {
        _disable = true;
    }

    // instance 函数：如果传入有效的 ip 和 port，则更新内部连接；否则使用现有设置
    static UdpSend& instance(const std::string & ip = "", int port = 0) {
        std::lock_guard<std::mutex> lock(s_mutex);
        if (!s_instance) {
            if(ip.empty() || port == 0) {
                ERROR("UdpSend 未初始化，请提供有效的 ip 和 port");
                throw std::runtime_error("UdpSend 未初始化");
            }
            s_instance.reset(new UdpSend(ip, port));
        } else if (!ip.empty() && port != 0) {
            s_instance->updateConnection(ip, port);
        }
        return *s_instance;
    }

    // 静态 sendData 方法，直接调用内部单例发送数据
    template<typename T>
    static void sendData(const T & data) {
        if(_disable) {WARN("UdpSend Disabled!");return;}
        instance().sendDataImpl(data);
    }
    static void sendTail() {
        if(_disable) {WARN("UdpSend Disabled!");return;}
        unsigned char tail[4] = {0x00, 0x00, 0x80, 0x7f};
        instance().sendDataImpl(tail);
    }
private:
    // 私有构造函数，仅供内部单例调用
    UdpSend(const std::string & ip, int port)
      : io_context_(), socket_(io_context_)
    {
        updateConnection(ip, port);
    }

    // 更新连接：解析 ip 和 port，建立目标 endpoint
    void updateConnection(const std::string & ip, int port) {
        try {
            boost::asio::ip::udp::resolver resolver(io_context_);
            auto endpoints = resolver.resolve(boost::asio::ip::udp::v4(), ip, std::to_string(port));
            endpoint_ = *endpoints.begin();
            if (!socket_.is_open()) {
                socket_.open(boost::asio::ip::udp::v4());
            }
        } catch (const std::exception & e) {
            ERROR("UdpSend updateConnection 错误: {}", e.what());
            throw std::runtime_error("UdpSend updateConnection 错误: " + std::string(e.what()));
        }
    }

    // 实际发送数据的实现函数
    template<typename T>
    void sendDataImpl(const T & data) {
        boost::system::error_code ec;
        // 使用 send_to 明确指定发送目标 endpoint
        auto bytes_sent = socket_.send_to(boost::asio::buffer(&data, sizeof(T)), endpoint_, 0, ec);
        if (ec) {
            ERROR("UdpSend 发送数据错误: {}", ec.message());
            throw std::runtime_error("UdpSend 发送数据错误: " + ec.message());
        }
    }

    boost::asio::io_context io_context_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint endpoint_;

    // 使用 C++17 inline static 变量实现 header-only 单例模式
    inline static std::unique_ptr<UdpSend> s_instance = nullptr;
    inline static std::mutex s_mutex;
    inline static bool _disable = false;
};