#include "laser_drv_vanjee.h"
#include "laser_net.h"
#include "efifo.h"
#include <iostream>
#include <stdio.h>
#include <time.h> // Added for localtime
//本代码检测的报文是区域报警下的报文
//g++ main.cpp laser_drv_vanjee.cpp laser_net.cpp efifo.cpp laser_time.cpp -o demo
int main() {
    laser_drv_t laser_obj;
    // 初始化设备，参数请根据实际情况填写
    if (laser_drv_vanjee_init(&laser_obj, "laser1", LASER_TYPE_VANJEE, "192.168.0.88", 2110, 6060) != eLaserDrvOk) {
        printf("激光初始化失败\n");
        return -1;
    }

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
                printf("收到一帧，长度: %d, 数据: ", raw_data.raw_data_count);
                // for (int i = 0; i < raw_data.raw_data_count; ++i) {
                //     printf("%02X ", raw_data.raw_data_buffer[i]);
                // }
                printf("\n");
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
                        if (freq_byte == 0x01) {
                        printf("IO输出：1\n");
                        } else if (freq_byte == 0x00) {
                        printf("IO输出：0\n");
                        } else {
                        printf("频率: 未知(0x%02X)\n", freq_byte);
                        }
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