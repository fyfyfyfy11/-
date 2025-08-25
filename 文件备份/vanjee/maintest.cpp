#include "laser_drv_vanjee.h"
#include "laser_net.h"
#include "efifo.h"
#include <iostream>
#include <stdio.h>
#include <time.h> // Added for localtime
#include <string.h> // Added for memset and strlen
#include <arpa/inet.h> // Added for inet_addr
#include <sys/socket.h> // Added for sendto
#include <unistd.h> // Added for usleep
#include <errno.h> // Added for perror
//本代码检测的报文是区域报警下的报文
//g++ main.cpp laser_drv_vanjee.cpp laser_net.cpp efifo.cpp laser_time.cpp -o demo
int main() {
    laser_drv_t laser_obj;
    // 初始化设备，参数请根据实际情况填写
    if (laser_drv_vanjee_init(&laser_obj, "laser1", LASER_TYPE_VANJEE, "192.168.0.2", 2110, 6060) != eLaserDrvOk) {
        printf("激光初始化失败\n");
        return -1;
    }

    // 初始化成功后，发送一条“success”到设备
    
        const char* msg = "success";
        struct sockaddr_in dest_addr;
        memset(&dest_addr, 0, sizeof(dest_addr));
        dest_addr.sin_family = AF_INET;
        // dest_addr.sin_port = htons(laser_obj.laser_net.dest_port);
        // dest_addr.sin_addr.s_addr = inet_addr(laser_obj.laser_net.dest_ip);
        dest_addr.sin_port = htons(6060); // 目标端口写死
        dest_addr.sin_addr.s_addr = inet_addr("192.168.0.2"); // 目标IP写死
        int ret = sendto(laser_obj.laser_net.sockfd, msg, strlen(msg), 0, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
        if (ret < 0) {
            perror("发送success信息失败");
        } else {
            printf("已发送success信息到设备\n");
            printf("端口：%d\n", dest_addr.sin_port);
            printf("IP：%s\n", "192.168.0.2");
        }
    usleep(10000000); // 10000ms

    while (1) {
        // 读取网络数据到FIFO
        if (laser_drv_vanjee_read_data(&laser_obj) != eLaserDrvOk) {
            printf("读取数据失败\n");
            continue;
        }

        // 解析一帧
        if (laser_drv_vanjee_get_one_frame(&laser_obj) == eLaserDrvOk) {
            laser_drv_raw_data_t raw_data = {0};
            raw_data.raw_data_buffer = NULL; // 必须初始化为NULL，内部会分配
            if (laser_drv_vanjee_get_raw_data(&laser_obj, &raw_data) == eLaserDrvOk) {
                // printf("收到一帧，长度: %d, 数据: ", raw_data.raw_data_count);
                // // for (int i = 0; i < raw_data.raw_data_count; ++i) {
                // //     printf("%02X ", raw_data.raw_data_buffer[i]);
                // // }
                // printf("\n");
                // printf("收到一帧，数据: ");
                // 解析6-9字节为时间戳（假设大端序）
                
                    // 大端序解析时间戳
                    uint32_t timestamp = 0;
                    timestamp |= (uint32_t)raw_data.raw_data_buffer[6] << 24;
                    timestamp |= (uint32_t)raw_data.raw_data_buffer[7] << 16;
                    timestamp |= (uint32_t)raw_data.raw_data_buffer[8] << 8;
                    timestamp |= (uint32_t)raw_data.raw_data_buffer[9];
                    printf("时间戳: %u 微秒\n", timestamp);
                    if (raw_data.raw_data_count >= 84)
                    {
                        printf("点云数据：\n", timestamp);
                        uint8_t freq_byte = raw_data.raw_data_buffer[79];
                        if (freq_byte == 0x01) {
                            printf("频率: 15Hz, ");
                        } else if (freq_byte == 0x02) {
                        printf("频率: 25Hz, ");
                        } else {
                        printf("频率: 未知(0x%02X), ", freq_byte);
                        }
                    
                        uint16_t point_count = 0;
                        point_count |= (uint16_t)raw_data.raw_data_buffer[83] << 8;
                        point_count |= (uint16_t)raw_data.raw_data_buffer[84];
                        printf("点数值: %u\n", point_count); 
                    }
                    else{
                        printf("--------------------------------------\n");
                        printf("报警报文:\n", timestamp);
                        uint8_t freq_byte = raw_data.raw_data_buffer[26];
                        printf("字节数: 0x%02X\n", freq_byte);
                        // 输出OUT1、OUT2、OUT3的值
                        printf("freq_byte: 0x%02X, 二进制: %c%c%c%c%c%c%c%c\n", freq_byte,
                            (freq_byte & 0x80) ? '1' : '0',
                            (freq_byte & 0x40) ? '1' : '0',
                            (freq_byte & 0x20) ? '1' : '0',
                            (freq_byte & 0x10) ? '1' : '0',
                            (freq_byte & 0x08) ? '1' : '0',
                            (freq_byte & 0x04) ? '1' : '0',
                            (freq_byte & 0x02) ? '1' : '0',
                            (freq_byte & 0x01) ? '1' : '0');
                        printf("OUT1: %d, OUT2: %d, OUT3: %d\n",
                            (freq_byte & 0x01) ? 1 : 0,
                            (freq_byte & 0x02) ? 1 : 0,
                            (freq_byte & 0x04) ? 1 : 0);
                        printf("--------------------------------------\n");
                    }

                // 用完记得释放
                if (raw_data.raw_data_buffer) {
                    rt_free(raw_data.raw_data_buffer);
                }
            }
        }
        //可适当延时
        usleep(100000); // 100ms
    }

    laser_drv_vanjee_destroy(&laser_obj);
    return 0;
}