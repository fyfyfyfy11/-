#include "drivers/vanjee_716mini_controller.h"
#include <vector>
#include <cstdio>
#include <unistd.h>

int main() {
    VanjeeLaserDriver driver("laser3", "192.168.0.2", 2110, 6060);
    if (!driver.initialize()) {
        printf("激光初始化失败\n");
        return -1;
    }
    printf("激光初始化成功\n");

    while (true) {
        if (!driver.readData()) {
            printf("读取数据失败\n");
            usleep(100000); // 100ms
            continue;
        }
        if (driver.getOneFrame()) {
            std::vector<uint8_t> raw;
            uint32_t timestamp = 0;
            uint8_t freq = 0;
            uint16_t point_count = 0;
            if (driver.getRawData(raw, timestamp, freq, point_count)) {
                if (raw.size() >= 84) {
                    //printf("点云报文: 时间戳: %u, 频率: %u, 点数: %u\n", timestamp, freq, point_count);
                    continue;
                } else {
                    // printf("--------------------------------------\n");
                    // printf("报警报文: ");
                    // if (raw.size() > 26) {
                    //     uint8_t freq_byte = raw[26];
                    //     printf("字节数为0x%02X\n", freq_byte);
                    //     printf("freq_byte: 0x%02X, 二进制: %c%c%c%c%c%c%c%c\n",
                    //         freq_byte,
                    //         (freq_byte & 0x80) ? '1' : '0',
                    //         (freq_byte & 0x40) ? '1' : '0',
                    //         (freq_byte & 0x20) ? '1' : '0',
                    //         (freq_byte & 0x10) ? '1' : '0',
                    //         (freq_byte & 0x08) ? '1' : '0',
                    //         (freq_byte & 0x04) ? '1' : '0',
                    //         (freq_byte & 0x02) ? '1' : '0',
                    //         (freq_byte & 0x01) ? '1' : '0');
                    //     printf("OUT1: %d, OUT2: %d, OUT3: %d\n",
                    //         (freq_byte & 0x01) ? 1 : 0,
                    //         (freq_byte & 0x02) ? 1 : 0,
                    //         (freq_byte & 0x04) ? 1 : 0);
                    // }
                    // printf("--------------------------------------\n");

                    printf("报警报文: ");
                    if (raw.size() > 26) {
                        uint8_t freq_byte = raw[26];
                        printf("字节数为0x%02X\n", freq_byte);
                        printf("二进制: %c%c%c%c%c%c%c%c\n",
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
                    }
                }
            }
        }
        usleep(100000); // 100ms
    }

    driver.shutdown();
    return 0;
}