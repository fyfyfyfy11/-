/**
 * @file    laser_drv_vanjee.c
 * @brief   万集数据处理
 * @details 
 * @par Copyright (C):
 *          HangZhou JiaZhi Science and Technology Ltd. All Rights Reserved.
 * @par Encoding:
 *          UTF-8
 * @par Modification:
 * -# Date    -#Author     -#details
 * 20230904     pez         create
 */

/* =============================================================================
 *                              header files
 * ========================================================================== */

#include "laser_drv_vanjee.h"
#include <cstring>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

/* =============================================================================
 *                            macro definition
 * ========================================================================== */



//#define DATA_BUFFER_SIZE 1500
#define DATA_BUFFER_SIZE 8192

#define NET_DEFAULT_PORT 2110

/* =============================================================================
 *                        static function definition
 * ========================================================================== */

namespace {
static unsigned int check_xor(const uint8_t *recv_buf, int recv_len) {
    int i = 0;
    unsigned char check = 0;
    const unsigned char *p = recv_buf;
    int len;
    if (*p == 0xFF) {
        p = p + 2;
        len = recv_len - 6;
        for (i = 0; i < len; i++) {
            check ^= *p++;
        }
        p++;
        if (check == *p) {
            return 0;
        } else {
            return -1;
        }
    } else {
        return -1;
    }
}
}

/* =============================================================================
 *                             global function
 * ========================================================================== */

VanjeeLaserDriver::VanjeeLaserDriver(const std::string& name, const std::string& dest_ip, uint16_t dest_port, uint16_t local_port)
    : dest_ip_(dest_ip), dest_port_(dest_port), local_port_(local_port) {
    base_info_.drv_name = name;
    raw_data_.frame_id = name;
    raw_data_.raw_data_buffer = nullptr;
    net_rx_buffer_ = nullptr;
    net_tx_buffer_ = nullptr;
    parase_fifo_ = nullptr;
}

VanjeeLaserDriver::~VanjeeLaserDriver() {
    shutdown();
}

bool VanjeeLaserDriver::initialize() {
    base_info_.is_init = false;
    base_info_.is_connected = false;
    base_info_.read_cnt = 0;
    base_info_.ok_cnt = 0;
    base_info_.error_cnt = 0;
    base_info_.reconnect_cnt = 0;
    raw_data_.seq = 0;
    raw_data_.raw_data_count = 0;
    if (raw_data_.raw_data_buffer) {
        rt_free(raw_data_.raw_data_buffer);
        raw_data_.raw_data_buffer = nullptr;
    }
    if (net_rx_buffer_) {
        rt_free(net_rx_buffer_);
        net_rx_buffer_ = nullptr;
    }
    if (net_tx_buffer_) {
        rt_free(net_tx_buffer_);
        net_tx_buffer_ = nullptr;
    }
    if (parase_fifo_) {
        efifo_destroy(parase_fifo_);
        parase_fifo_ = nullptr;
    }
    raw_data_.raw_data_buffer = (uint8_t*)rt_malloc(DATA_BUFFER_SIZE);
    if (!raw_data_.raw_data_buffer) return false;
    net_rx_buffer_ = (uint8_t*)rt_malloc(DATA_BUFFER_SIZE);
    if (!net_rx_buffer_) return false;
    net_tx_buffer_ = (uint8_t*)rt_malloc(DATA_BUFFER_SIZE);
    if (!net_tx_buffer_) return false;
    if (laser_net_init(&laser_net_, base_info_.drv_name.c_str(), dest_ip_.c_str(), dest_port_, local_port_) != eLaserNetOk)
        return false;
    if (dest_port_ == NET_DEFAULT_PORT) {
        if (laser_net_connect_udp(&laser_net_) != eLaserNetOk)
            return false;
    } else {
        return false;
    }
    parase_fifo_ = efifo_creat(base_info_.drv_name.c_str(), DATA_BUFFER_SIZE*4, true);
    efifo_clean(parase_fifo_);
    base_info_.is_init = true;
    base_info_.is_connected = true;
    gettimeofday(&base_info_.start_time, nullptr);
    
    // 初始化成功后，发送一条"success"到设备
    const char* msg = "success";
    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(local_port_); //UDP激光设备端口6060
    dest_addr.sin_addr.s_addr = inet_addr(dest_ip_.c_str()); // 使用传入的目标IP
    int ret = sendto(laser_net_.sockfd, msg, strlen(msg), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    if (ret < 0) {
        perror("发送success信息失败");
    } else {
        printf("已发送success信息到设备\n");
        printf("端口：%d\n", dest_addr.sin_port);
        printf("IP：%s\n", dest_ip_.c_str());
    }
    
    return true;
}

bool VanjeeLaserDriver::isInitialized() const {
    return base_info_.is_init;
}

std::string VanjeeLaserDriver::getName() const {
    return base_info_.drv_name;
}

void VanjeeLaserDriver::shutdown() {
    cleanup();
}

void VanjeeLaserDriver::cleanup() {
    if (base_info_.is_init) {
        laser_net_close(&laser_net_);
        if (net_rx_buffer_) {
            rt_free(net_rx_buffer_);
            net_rx_buffer_ = nullptr;
        }
        if (net_tx_buffer_) {
            rt_free(net_tx_buffer_);
            net_tx_buffer_ = nullptr;
        }
        if (parase_fifo_) {
            efifo_destroy(parase_fifo_);
            parase_fifo_ = nullptr;
        }
        if (raw_data_.raw_data_buffer) {
            rt_free(raw_data_.raw_data_buffer);
            raw_data_.raw_data_buffer = nullptr;
        }
        base_info_.is_init = false;
        base_info_.is_connected = false;
    }
}

bool VanjeeLaserDriver::readData() {
    if (!base_info_.is_init || !base_info_.is_connected) return false;
    uint32_t rx_count = 0;
    if (laser_net_recv_some(&laser_net_, &net_rx_buffer_[net_rx_count_], DATA_BUFFER_SIZE - net_rx_count_, &rx_count) != eLaserNetOk) {
        return false;
    }
    base_info_.read_cnt++;
    net_rx_count_ += rx_count;
    int fifo_write_cnt = efifo_write(parase_fifo_, net_rx_buffer_, net_rx_count_);
    net_rx_count_ = 0;
    if (fifo_write_cnt != (int)rx_count) {
        base_info_.error_cnt++;
        return false;
    }
    return true;
}

bool VanjeeLaserDriver::getOneFrame() {
    while (efifo_get_used(parase_fifo_) > 4) {
        efifo_pick(parase_fifo_, net_tx_buffer_, 2);
        if (net_tx_buffer_[0] != 0xFF || net_tx_buffer_[1] != 0xAA) {
            base_info_.error_cnt++;
            efifo_cut(parase_fifo_, 1);
            continue;
        }
        efifo_pick(parase_fifo_, net_tx_buffer_, 4);
        unsigned short frame_len = ((net_tx_buffer_[2] & 0x00ff) << 8) | (net_tx_buffer_[3] & 0x00ff);
        frame_len += 4;
        if (efifo_get_used(parase_fifo_) < frame_len) return false;
        efifo_pick(parase_fifo_, net_tx_buffer_, frame_len);
        if (net_tx_buffer_[frame_len - 1] != 0xEE || net_tx_buffer_[frame_len - 2] != 0xEE) {
            base_info_.error_cnt++;
            efifo_cut(parase_fifo_, 2);
            continue;
        }
        if (0 != check_xor(net_tx_buffer_, frame_len)) {
            base_info_.error_cnt++;
            efifo_cut(parase_fifo_, frame_len);
            continue;
        }
        gettimeofday(&base_info_.end_time, nullptr);
        base_info_.diff_time = laser_timeval_diff(&base_info_.start_time, &base_info_.end_time);
        base_info_.ok_cnt++;
        raw_data_.seq++;
        raw_data_.stamp = base_info_.start_time;
        raw_data_.raw_data_count = frame_len;
        memcpy(raw_data_.raw_data_buffer, net_tx_buffer_, frame_len);
        efifo_cut(parase_fifo_, frame_len);
        gettimeofday(&base_info_.start_time, nullptr);
        return true;
    }
    return false;
}

bool VanjeeLaserDriver::getRawData(std::vector<uint8_t>& out, uint32_t& timestamp, uint8_t& freq, uint16_t& point_count) {
    if (!base_info_.is_init) return false;
    if (raw_data_.raw_data_count > 0) {
        out.resize(raw_data_.raw_data_count);
        memcpy(out.data(), raw_data_.raw_data_buffer, raw_data_.raw_data_count);
        if (raw_data_.raw_data_count >= 84) {
            timestamp = 0;
            timestamp |= (uint32_t)out[6] << 24;
            timestamp |= (uint32_t)out[7] << 16;
            timestamp |= (uint32_t)out[8] << 8;
            timestamp |= (uint32_t)out[9];
            freq = out[79];
            point_count = 0;
            point_count |= (uint16_t)out[83] << 8;
            point_count |= (uint16_t)out[84];
        } else {
            timestamp = 0;
            freq = 0;
            point_count = 0;
        }
        raw_data_.raw_data_count = 0;
        return true;
    }
    return false;
}

/* =============================================================================
 *                              static function 
 * ========================================================================== */

/**
 * bbc校验.
 *
 * @param[in]
 *      recv_buf : 数据指针
 *      recv_len : 数据长度
 * @param[out]
 * @return
 *      0 - 校验成功
 *     -1 - 校验失败
 */
static unsigned int check_xor(unsigned char *recv_buf, int recv_len)
{
    int i = 0;
    unsigned char check = 0;
    unsigned char *p = recv_buf;
    int len;

    if (*p == 0xFF)
    {
        p = p + 2;
        len = recv_len - 6;
        for (i = 0; i < len; i++)
        {
            check ^= *p++;
        }
        p++;
        if (check == *p)
        {
            return 0;
        }
        else
            return -1;
    }
    else
    {
        return -1;
    }
}

/* =============================================================================
 *                              shell function
 * ========================================================================== */


/*******************************************************************************
 *                                  EOF
 ******************************************************************************/
