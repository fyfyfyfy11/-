#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdexcept>
#include <sstream>

#define BUFFER_SIZE 1024
#define SERVER_PORT 3002
#define SERVER_IP "172.19.3.114"

class TCPSocketClient {
public:
    // ... [保持原有的构造函数、析构函数和receiveString方法不变] ...

    // 修改后的解析方法，支持不定长原始码值
    static void parseAndPrintData(const std::string& data) {
        std::cout << "--------------------------------------------------------" << std::endl;
        
        // 检查是否为"NoRead"情况
        if (data.find("NoRead") != std::string::npos) {
            std::cout << "[未识别到码] 跳过处理" << std::endl;
            return;
        }

        // 尝试解析格式：<不定长数字>(X,Y)角度
        size_t paren_start = data.find('(');
        size_t paren_end = data.find(')');
        
        if (paren_start != std::string::npos && paren_end != std::string::npos) {
            // 提取原始码值（从开头到左括号前的所有数字）
            std::string code = data.substr(0, paren_start);
            
            // 提取坐标部分
            std::string coords = data.substr(paren_start + 1, paren_end - paren_start - 1);
            size_t comma_pos = coords.find(',');
            
            // 提取角度部分（右括号后的数字）
            std::string angle_str = data.substr(paren_end + 1);
            
            if (comma_pos != std::string::npos) {
                std::string x_str = coords.substr(0, comma_pos);
                std::string y_str = coords.substr(comma_pos + 1);
                
                // 输出解析结果
                std::cout << "原始码值: " << code << std::endl;
                std::cout << "X坐标: " << x_str << std::endl;
                std::cout << "Y坐标: " << y_str << std::endl;
                std::cout << "角度: " << angle_str << "°" << std::endl;
                return;
            }
        }

        // 如果不符合上述格式，直接输出原始数据
        std::cout << "接收到的原始数据: " << data << std::endl;
    }

private:
    int sock_fd_ = -1;
};

int main() {
    try {
        TCPSocketClient client;
        
        while (true) {
            std::string received = client.receiveString();
            if (received.empty()) {
                std::cout << "连接已关闭" << std::endl;
                break;
            }

            TCPSocketClient::parseAndPrintData(received);
        }
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}