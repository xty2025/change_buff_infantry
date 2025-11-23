//#pragma once
#include <string>
#include <fstream>
#include <stdexcept>
#include <Log/log.hpp>
#include <nlohmann/json.hpp> 
namespace param{
class Param{
private:
    enum class InternalTag{Internal};//标志位不多。
    nlohmann::json json_;
    // 私有构造函数：用于内部创建子级 Param 对象
    Param(const nlohmann::json& json,InternalTag):json_(json){};
public:
    explicit Param(const std::string &filepath){
        std::ifstream in(filepath);
        if(!in.is_open()){
            ERROR("无法打开文件：{}",filepath);    
        }
        try{
            in>>json_;
        }catch(const nlohmann::json::parse_error &e){
            ERROR("{}",e.what());
        } 
    }
    Param operator[](const std::string &key) const {
        if(!json_.contains(key)){ERROR("{}",key);}
        return Param(json_.at(key),InternalTag::Internal);
    }
    //get方法转换为指定类型并返回。
    template<typename T>
    T get(const std::string &key)const{
        if(!json_.contains(key)){ERROR("{}",key);
        return json_.at(key).template get<T>();}
    }
};

};