#ifndef LASER_DRV_VANJEE_H
#define LASER_DRV_VANJEE_H

#include <string>
#include <vector>
#include <stdint.h>
#include <stdbool.h>
#include <sys/time.h>
#include "laser_net.h"
#include "efifo.h"
#include "rtthread_port.h"
#include "laser_time.h"
#include "drivers/driver_interface.h"

class VanjeeLaserDriver : public stu::DriverInterface {
public:
    VanjeeLaserDriver(const std::string& name, const std::string& dest_ip, uint16_t dest_port, uint16_t local_port);
    virtual ~VanjeeLaserDriver();

    // DriverInterface实现
    bool initialize() override;
    bool isInitialized() const override;
    std::string getName() const override;
    void shutdown() override;

    // 新增功能接口
    bool readData(); // 读取网络数据到FIFO
    bool getOneFrame(); // 从FIFO解析一帧
    bool getRawData(std::vector<uint8_t>& out, uint32_t& timestamp, uint8_t& freq, uint16_t& point_count); // 获取原始数据

private:
    enum class Status {
        IDLE = 0,
        RUNNING = 1,
        ERROR = 2
    };

    struct BaseInfo {
        std::string drv_name;
        bool is_init = false;
        bool is_connected = false;
        int read_cnt = 0;
        int ok_cnt = 0;
        int error_cnt = 0;
        int reconnect_cnt = 0;
        struct timeval start_time{};
        struct timeval end_time{};
        long diff_time = 0;
    };

    struct RawData {
        uint32_t seq = 0;
        struct timeval stamp{};
        std::string frame_id;
        uint32_t raw_data_count = 0;
        uint8_t* raw_data_buffer = nullptr;
    };

    BaseInfo base_info_;
    RawData raw_data_;
    laser_net_t laser_net_;
    efifo_t* parase_fifo_ = nullptr;
    uint8_t* net_rx_buffer_ = nullptr;
    uint8_t* net_tx_buffer_ = nullptr;
    uint32_t net_rx_count_ = 0;
    uint32_t net_tx_count_ = 0;

    std::string dest_ip_;
    uint16_t dest_port_;
    uint16_t local_port_;

    void cleanup();
};

#endif