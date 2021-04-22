#ifndef CONFIG_H
#define CONFIG_H

#include <memory> 
#include <opencv2/opencv.hpp>

class Config {
private:
    std::string file_name_;

public:
    Config();
    Config(const std::string& file_name);
    ~Config();

    //　配置文件名称
    void setParameterFile(const std::string &file_name);

    //　获取配置值
    template  <typename T>
    T get (const std::string &key) const{
        cv::FileStorage file = cv::FileStorage(file_name_.c_str(), cv::FileStorage::READ);
        if (!file.isOpened()){
            std::cerr << "parameter file" << file_name_ << "does not exist." << std::endl;
        }
        T res;
        file[key] >> res;
        file.release();
        return res;
    }

    template  <typename T>
    void set(const std::string &key, const T&value){
        cv::FileStorage file = cv::FileStorage(file_name_.c_str(), cv::FileStorage::WRITE);
        file << key << value;
        file.release();
    }

};

#endif
