#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <cstring>
#include <cmath>

// 常量定义
const float MARGINAL_DEADZONE = 0.02f;
const float CENTER_DEADZONE = 0.025f;
const int AXIS_COUNT = 8;
const int BUTTON_COUNT = 15;

// 全局变量用于信号处理
volatile bool running = true;

// D-pad状态跟踪
struct DpadState {
    bool left_pressed = false;
    bool right_pressed = false;
    bool up_pressed = false;
    bool down_pressed = false;
};

DpadState dpad_state;

// RT+按钮组合状态跟踪
struct RTButtonState {
    bool rt_button0_pressed = false;  // RT + 按钮0 (A)
    bool rt_button1_pressed = false;  // RT + 按钮1 (B)
    bool rt_button2_pressed = false;  // RT + 按钮2 (X)
    bool rt_button3_pressed = false;  // RT + 按钮3 (Y)
    float last_rt_value = 0.0f;
    bool last_button0_state = false;
    bool last_button1_state = false;
    bool last_button2_state = false;
    bool last_button3_state = false;
};

RTButtonState rt_button_state;

// RT+D-pad组合状态跟踪
struct RTDpadState {
    bool rt_dpad_left_pressed = false;
    bool rt_dpad_right_pressed = false;
    bool rt_dpad_up_pressed = false;
    bool rt_dpad_down_pressed = false;
    
    // 时间跟踪
    std::chrono::steady_clock::time_point left_start_time;
    std::chrono::steady_clock::time_point right_start_time;
    std::chrono::steady_clock::time_point up_start_time;
    std::chrono::steady_clock::time_point down_start_time;
    
    bool left_timing = false;
    bool right_timing = false;
    bool up_timing = false;
    bool down_timing = false;
};

RTDpadState rt_dpad_state;

// 信号处理函数
void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "收到Ctrl+C信号，正在退出..." << std::endl;
        running = false;
    }
}

// 死区处理函数
float apply_deadzone(float value) {
    if (value + MARGINAL_DEADZONE > 1.0f) {
        return 1.0f;
    } else if (value - MARGINAL_DEADZONE < -1.0f) {
        return -1.0f;
    }
    if (std::abs(value) < CENTER_DEADZONE) {
        return 0.0f;
    }
    return value;
}

// 将原始值转换为-1到1的范围
float normalize_axis(int16_t value) {
    return static_cast<float>(value) / 32767.0f;
}

// D-pad处理函数
void handle_dpad(int axis, float value) {
    if (axis == 6) { // D-pad X轴 (左右)
        bool left_pressed = (value < -0.5f);
        bool right_pressed = (value > 0.5f);
        
        // 左方向
        if (left_pressed && !dpad_state.left_pressed) {
            std::cout << "D-pad 左 按下" << std::endl;
            dpad_state.left_pressed = true;
        } else if (!left_pressed && dpad_state.left_pressed) {
            std::cout << "D-pad 左 释放" << std::endl;
            dpad_state.left_pressed = false;
        }
        
        // 右方向
        if (right_pressed && !dpad_state.right_pressed) {
            std::cout << "D-pad 右 按下" << std::endl;
            dpad_state.right_pressed = true;
        } else if (!right_pressed && dpad_state.right_pressed) {
            std::cout << "D-pad 右 释放" << std::endl;
            dpad_state.right_pressed = false;
        }
    } else if (axis == 7) { // D-pad Y轴 (上下)
        bool up_pressed = (value < -0.5f);
        bool down_pressed = (value > 0.5f);
        
        // 上方向
        if (up_pressed && !dpad_state.up_pressed) {
            std::cout << "D-pad 上 按下" << std::endl;
            dpad_state.up_pressed = true;
        } else if (!up_pressed && dpad_state.up_pressed) {
            std::cout << "D-pad 上 释放" << std::endl;
            dpad_state.up_pressed = false;
        }
        
        // 下方向
        if (down_pressed && !dpad_state.down_pressed) {
            std::cout << "D-pad 下 按下" << std::endl;
            dpad_state.down_pressed = true;
        } else if (!down_pressed && dpad_state.down_pressed) {
            std::cout << "D-pad 下 释放" << std::endl;
            dpad_state.down_pressed = false;
        }
    }
}

// RT+按钮组合处理函数
void handle_rt_button_combinations(float rt_value, bool button0_pressed, bool button1_pressed, bool button2_pressed, bool button3_pressed) {
    bool rt_in_range = (rt_value >= 0.7f && rt_value <= 1.0f);
    
    // RT + 按钮0 (A)
    bool combination0_active = rt_in_range && button0_pressed;
    if (combination0_active && !rt_button_state.rt_button0_pressed) {
        std::cout << "RT+按钮A已同时按下" << std::endl;
        rt_button_state.rt_button0_pressed = true;
    } else if (!combination0_active && rt_button_state.rt_button0_pressed) {
        std::cout << "RT+按钮A组合释放" << std::endl;
        rt_button_state.rt_button0_pressed = false;
    }
    
    // RT + 按钮1 (B)
    bool combination1_active = rt_in_range && button1_pressed;
    if (combination1_active && !rt_button_state.rt_button1_pressed) {
        std::cout << "RT+按钮B已同时按下" << std::endl;
        rt_button_state.rt_button1_pressed = true;
    } else if (!combination1_active && rt_button_state.rt_button1_pressed) {
        std::cout << "RT+按钮B组合释放" << std::endl;
        rt_button_state.rt_button1_pressed = false;
    }
    
    // RT + 按钮2 (X)
    bool combination2_active = rt_in_range && button2_pressed;
    if (combination2_active && !rt_button_state.rt_button2_pressed) {
        std::cout << "RT+按钮X已同时按下" << std::endl;
        rt_button_state.rt_button2_pressed = true;
    } else if (!combination2_active && rt_button_state.rt_button2_pressed) {
        std::cout << "RT+按钮X组合释放" << std::endl;
        rt_button_state.rt_button2_pressed = false;
    }
    
    // RT + 按钮3 (Y)
    bool combination3_active = rt_in_range && button3_pressed;
    if (combination3_active && !rt_button_state.rt_button3_pressed) {
        std::cout << "RT+按钮Y已同时按下" << std::endl;
        rt_button_state.rt_button3_pressed = true;
    } else if (!combination3_active && rt_button_state.rt_button3_pressed) {
        std::cout << "RT+按钮Y组合释放" << std::endl;
        rt_button_state.rt_button3_pressed = false;
    }
    
    // 更新状态
    rt_button_state.last_rt_value = rt_value;
    rt_button_state.last_button0_state = button0_pressed;
    rt_button_state.last_button1_state = button1_pressed;
    rt_button_state.last_button2_state = button2_pressed;
    rt_button_state.last_button3_state = button3_pressed;
}

// RT+D-pad组合处理函数
void handle_rt_dpad_combinations(float rt_value, bool dpad_left, bool dpad_right, bool dpad_up, bool dpad_down) {
    bool rt_in_range = (rt_value >= 0.7f && rt_value <= 1.0f);
    auto current_time = std::chrono::steady_clock::now();
    const int hold_time_ms = 200; // 200ms保持时间
    
    // RT + D-pad 左
    if (rt_in_range && dpad_left) {
        if (!rt_dpad_state.left_timing) {
            rt_dpad_state.left_start_time = current_time;
            rt_dpad_state.left_timing = true;
        } else {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - rt_dpad_state.left_start_time);
            if (elapsed.count() >= hold_time_ms && !rt_dpad_state.rt_dpad_left_pressed) {
                std::cout << "RT+D-pad左已同时按下(保持" << hold_time_ms << "ms)" << std::endl;
                rt_dpad_state.rt_dpad_left_pressed = true;
            }
        }
    } else {
        if (rt_dpad_state.left_timing) {
            rt_dpad_state.left_timing = false;
        }
        if (rt_dpad_state.rt_dpad_left_pressed) {
            std::cout << "RT+D-pad左组合释放" << std::endl;
            rt_dpad_state.rt_dpad_left_pressed = false;
        }
    }
    
    // RT + D-pad 右
    if (rt_in_range && dpad_right) {
        if (!rt_dpad_state.right_timing) {
            rt_dpad_state.right_start_time = current_time;
            rt_dpad_state.right_timing = true;
        } else {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - rt_dpad_state.right_start_time);
            if (elapsed.count() >= hold_time_ms && !rt_dpad_state.rt_dpad_right_pressed) {
                std::cout << "RT+D-pad右已同时按下(保持" << hold_time_ms << "ms)" << std::endl;
                rt_dpad_state.rt_dpad_right_pressed = true;
            }
        }
    } else {
        if (rt_dpad_state.right_timing) {
            rt_dpad_state.right_timing = false;
        }
        if (rt_dpad_state.rt_dpad_right_pressed) {
            std::cout << "RT+D-pad右组合释放" << std::endl;
            rt_dpad_state.rt_dpad_right_pressed = false;
        }
    }
    
    // RT + D-pad 上
    if (rt_in_range && dpad_up) {
        if (!rt_dpad_state.up_timing) {
            rt_dpad_state.up_start_time = current_time;
            rt_dpad_state.up_timing = true;
        } else {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - rt_dpad_state.up_start_time);
            if (elapsed.count() >= hold_time_ms && !rt_dpad_state.rt_dpad_up_pressed) {
                std::cout << "RT+D-pad上已同时按下(保持" << hold_time_ms << "ms)" << std::endl;
                rt_dpad_state.rt_dpad_up_pressed = true;
            }
        }
    } else {
        if (rt_dpad_state.up_timing) {
            rt_dpad_state.up_timing = false;
        }
        if (rt_dpad_state.rt_dpad_up_pressed) {
            std::cout << "RT+D-pad上组合释放" << std::endl;
            rt_dpad_state.rt_dpad_up_pressed = false;
        }
    }
    
    // RT + D-pad 下
    if (rt_in_range && dpad_down) {
        if (!rt_dpad_state.down_timing) {
            rt_dpad_state.down_start_time = current_time;
            rt_dpad_state.down_timing = true;
        } else {
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - rt_dpad_state.down_start_time);
            if (elapsed.count() >= hold_time_ms && !rt_dpad_state.rt_dpad_down_pressed) {
                std::cout << "RT+D-pad下已同时按下(保持" << hold_time_ms << "ms)" << std::endl;
                rt_dpad_state.rt_dpad_down_pressed = true;
            }
        }
    } else {
        if (rt_dpad_state.down_timing) {
            rt_dpad_state.down_timing = false;
        }
        if (rt_dpad_state.rt_dpad_down_pressed) {
            std::cout << "RT+D-pad下组合释放" << std::endl;
            rt_dpad_state.rt_dpad_down_pressed = false;
        }
    }
}

// 打印手柄信息
void print_joystick_info(const std::string& device_path) {
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) {
        std::cerr << "无法打开设备: " << device_path << std::endl;
        return;
    }

    char name[256];
    if (ioctl(fd, JSIOCGNAME(sizeof(name)), name) >= 0) {
        std::cout << "手柄名称: " << name << std::endl;
    }

    uint8_t axes, buttons;
    if (ioctl(fd, JSIOCGAXES, &axes) >= 0 && ioctl(fd, JSIOCGBUTTONS, &buttons) >= 0) {
        std::cout << "轴数量: " << static_cast<int>(axes) << std::endl;
        std::cout << "按钮数量: " << static_cast<int>(buttons) << std::endl;
    }

    close(fd);
}

// 检测是否为Xbox手柄
bool is_xbox_controller(const std::string& device_path) {
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) {
        return false;
    }

    char name[256];
    bool is_xbox = false;
    if (ioctl(fd, JSIOCGNAME(sizeof(name)), name) >= 0) {
        std::string device_name(name);
        is_xbox = (device_name.find("Xbox") != std::string::npos || 
                   device_name.find("X-Box") != std::string::npos);
    }

    close(fd);
    return is_xbox;
}

int main() {
    // 设置信号处理
    signal(SIGINT, signal_handler);
    
    std::cout << "手柄测试程序启动... v1.0.0" << std::endl;
    
    // 查找手柄设备
    std::string device_path = "/dev/input/js0";
    int fd = open(device_path.c_str(), O_RDONLY);
    if (fd < 0) {
        std::cerr << "无法打开手柄设备: " << device_path << std::endl;
        std::cerr << "请确保手柄已连接并且设备存在" << std::endl;
        return 1;
    }

    // 打印手柄信息
    print_joystick_info(device_path);
    bool is_xbox = is_xbox_controller(device_path);
    std::cout << "是否为Xbox手柄: " << (is_xbox ? "是" : "否") << std::endl;

    // 设置非阻塞模式
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    // 初始化数据
    std::vector<float> axes(AXIS_COUNT, 0.0f);
    std::vector<int> buttons(BUTTON_COUNT, 0);
    
    std::cout << "开始读取手柄输入... (按Ctrl+C退出)" << std::endl;
    std::cout << "轴值范围: -1.0 到 1.0" << std::endl;
    std::cout << "按钮值: 0(未按下) 或 1(按下)" << std::endl;
    std::cout << "D-pad支持: 轴6(左右) 轴7(上下)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    auto last_time = std::chrono::steady_clock::now();
    int run_cnt = 0;

    while (running) {
        js_event event;
        int bytes_read = read(fd, &event, sizeof(event));
        
        if (bytes_read == sizeof(event)) {
            // 处理事件
            if (event.type & JS_EVENT_BUTTON) {
                if (event.number < BUTTON_COUNT) {
                    buttons[event.number] = event.value;
                    
                    // 打印按钮状态变化
                    if (event.value) {
                        std::cout << "按钮 " << static_cast<int>(event.number) << " 按下" << std::endl;
                    } else {
                        std::cout << "按钮 " << static_cast<int>(event.number) << " 释放" << std::endl;
                    }
                    
                    // 如果按钮0-3状态改变，重新检查RT+按钮组合
                    if (event.number >= 0 && event.number <= 3) {
                        handle_rt_button_combinations(axes[5], buttons[0], buttons[1], buttons[2], buttons[3]);
                    }
                }
            } else if (event.type & JS_EVENT_AXIS) {
                if (event.number < AXIS_COUNT) {
                    float normalized_value = normalize_axis(event.value);
                    float deadzone_value = apply_deadzone(normalized_value);
                    axes[event.number] = deadzone_value;
                    
                    // 特殊处理D-pad (轴6和轴7)
                    if (event.number == 6 || event.number == 7) {
                        handle_dpad(event.number, deadzone_value);
                    } else {
                        // 打印其他轴值变化（只在有显著变化时）
                        if (std::abs(deadzone_value) > 0.1f) {
                            std::cout << "轴 " << static_cast<int>(event.number) << ": " << deadzone_value << std::endl;
                        }
                    }
                    
                    // 检查RT+按钮组合 (轴5是右扳机)
                    if (event.number == 5) {
                        handle_rt_button_combinations(deadzone_value, buttons[0], buttons[1], buttons[2], buttons[3]);
                    }
                }
            }
        }

        // 定期检查RT+D-pad组合状态（每10ms检查一次）
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time);
        
        if (elapsed.count() >= 10) { // 10ms间隔检查
            // 实时检查RT+D-pad组合
            handle_rt_dpad_combinations(axes[5], dpad_state.left_pressed, dpad_state.right_pressed, dpad_state.up_pressed, dpad_state.down_pressed);
            
            if (run_cnt < 40 * 10) {
                run_cnt++;
            } else {
                run_cnt = 0;
                std::cout << "手柄程序正在运行..." << std::endl;
            }
            last_time = current_time;
        }

        // 短暂休眠以减少CPU使用
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    close(fd);
    std::cout << "程序已退出" << std::endl;
    return 0;
} 