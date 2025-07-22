//本代码可以直接运行，但需要确认IP地址和端口号是否和相机一致
//编译命令：g++ camera_driver.cpp -o camera_driver_demo
//本代码编译生成的demo文件为camera_driver_demo,可以在终端./camera_driver_demo运行
//本代码服务于R3138MG010
#include <iostream>
#include <iomanip>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdint>

#define BUFFER_SIZE 1024

void printHex(const uint8_t* data, size_t len) {
    std::cout<<"--------------------------------------------------------"<<std::endl;
    std::cout << "HEX数据: ";
    for (size_t i = 0; i < len; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                 << static_cast<int>(data[i]) << " ";
    }
    std::cout << std::dec << std::endl;  // 恢复十进制输出
}

// 解析并打印AGV数据
void parseAGVData(const uint8_t* data, size_t len) {
    if (len < 19) {
        std::cerr << "数据长度不足，无法解析" << std::endl;
        return;
    }

    // 解析TAG
    int NP  = (data[0] >> 1) & 0x01;
    int TAG = (data[1] >> 6) & 0x01;
    std::cout << "TAG=" << TAG  <<","<< "NP=" << NP <<std::endl;

    // 解析AgvX
    int32_t AgvX = 0;
    uint32_t X_tmp =  ((uint32_t)(data[2] & 0x07) << 21)
                      |  ((uint32_t)(data[3] & 0x7F) << 14)
                      |  ((uint32_t)(data[4] & 0x7F) << 7)
                      |  ((uint32_t)(data[5] & 0x7F) );

    if((X_tmp & 0x00400000) != 0)
    {
        AgvX = (int32_t)(X_tmp | 0xFF800000);
    }
    else
    {
        AgvX = X_tmp;
    }
    std::cout << "AgvX=" << AgvX*0.1 << " mm" << std::endl;

    // 解析AgvY
    int16_t  AgvY = 0;
    uint16_t Y_tmp = data[6] * 0x80 + data[7];
    if((Y_tmp >> 13) & 0x0001 == 1)
    {
        AgvY = Y_tmp + 0xC000;
    }
    else
    {
        AgvY = Y_tmp;
    }
    std::cout << "AgvY=" << AgvY*0.1 << " mm" << std::endl;

    // 解析AgvAngle
    uint16_t AgvAngle = (((uint16_t)(data[10] & 0x7F) << 7)
                   | (uint16_t)(data[11] & 0x7F ) ) ;
    std::cout << "AgvAngle=" << AgvAngle*0.1 << "°" << std::endl;

    // 解析AgvTagNum
    uint32_t AgvTagNum  = (
                      ((uint32_t)(data[13] & 0x7F)  << 28)
                    | ((uint32_t)(data[14] & 0x7F ) << 21)
                    | ((uint32_t)(data[15] & 0x7F ) << 14)
                    | ((uint32_t)(data[16] & 0x7F ) << 7)
                    | ((uint32_t)(data[17] & 0x7F ) << 0)
                    ) ;
    std::cout << "AgvTagNum=" << AgvTagNum << std::endl;
}

int main() {
    // 创建 Socket 并连接服务器
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return -1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(3001); //在此处修改端口
    if (inet_pton(AF_INET, "172.19.3.196", &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid server address." << std::endl;
        close(sock);
        return -1;
    }

    if (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr))) {
        std::cerr << "Failed to connect to server." << std::endl;
        close(sock);
        return -1;
    }

    std::cout << "Connected to server. Waiting for data..." << std::endl;

    uint8_t buffer[BUFFER_SIZE];
    while (true) {
        ssize_t bytes_received = recv(sock, buffer, BUFFER_SIZE, 0);
        if (bytes_received <= 0) {
            std::cerr << "Connection closed or error occurred." << std::endl;
            break;
        }

        // 判断报文类型
        if (bytes_received >= 2) {
            if (buffer[0] == 0x00 && buffer[1] == 0x44) {
                printHex(buffer, bytes_received);  // HEX显示
                parseAGVData(buffer, bytes_received); //解析数据包
            } else if (buffer[0] == 0x02 && buffer[1] == 0x04) {
                // 未读到码，不处理
                continue;
            }
        }
    }

    close(sock);
    return 0;
}