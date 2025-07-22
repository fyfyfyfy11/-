#ifndef CAMERA_DRIVER_H
#define CAMERA_DRIVER_H

#include "driver_interface.h" // 包含驱动基类
#include <vector>
#include <string>
#include <cstdint>
#include <memory>

class CameraDriver {
public:
    struct MarkerPosition {
        int NP = 1;          // 默认未读到码
        int TAG = 0;         // 数据类型标志
        float AgvX = 0.0f;   // X坐标(mm)
        float AgvY = 0.0f;   // Y坐标(mm)
        float AgvAngle = 0.0f;  // 角度(°)
        uint32_t AgvTagNum = 0; // 标签号
        
        std::string toString() const;
    };

    // 构造函数
    explicit CameraDriver(const std::string& server_ip = "172.19.3.196", 
                         uint16_t port = 3001);
    
    // 析构函数
    ~CameraDriver();
    
    // 连接相机
    bool connect();
    
    // 断开连接
    void disconnect();
    
    // 获取标记位置信息
    MarkerPosition getMarkerPosition();
    
    // 获取原始HEX数据字符串
    std::string getMarkerData();
    
    // 检查是否连接
    bool isConnected() const;

private:
    int sock_ = -1;
    std::string server_ip_;
    uint16_t port_;
    std::vector<uint8_t> buffer_;
    
    // 接收数据
    bool receiveData();
    
    // 解析AGV数据
    MarkerPosition parseAGVData(const std::vector<uint8_t>& data) const;
    
    // 生成HEX字符串
    std::string generateHexString(const std::vector<uint8_t>& data) const;
};

#endif // CAMERA_DRIVER_H