#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <vector>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <algorithm>

#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16

// 串口初始化函数
int init_serial(const std::string &device, int baud_rate) {
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK); // 非阻塞打开串口
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    cfsetispeed(&options, baud_rate);
    cfsetospeed(&options, baud_rate);

    // 配置8N1模式
    options.c_cflag &= ~PARENB; // 无奇偶校验
    options.c_cflag &= ~CSTOPB; // 1位停止位
    options.c_cflag &= ~CSIZE; // 清除数据位掩码
    options.c_cflag |= CS8;    // 8位数据位

    options.c_cflag |= (CLOCAL | CREAD); // 启用接收器，忽略调制解调器控制线
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入模式
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控制
    options.c_oflag &= ~OPOST; // 原始输出模式

    // 设置选项
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

bool parse_elrs_data(const uint8_t *elrs_data_temp, std::vector<uint16_t> &elrs_data) {
    if (elrs_data_temp[2] == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        const unsigned numOfChannels = 16;
        const unsigned srcBits = 11;
        const unsigned inputChannelMask = (1 << srcBits) - 1;

        uint8_t bitsMerged = 0;
        uint32_t readValue = 0;
        unsigned readByteIndex = 3;

        for (uint8_t n = 0; n < numOfChannels; n++) {
            while (bitsMerged < srcBits) {
                uint8_t readByte = elrs_data_temp[readByteIndex++];
                readValue |= static_cast<uint32_t>(readByte) << bitsMerged;
                bitsMerged += 8;
            }

            elrs_data[n] = (readValue & inputChannelMask);
            readValue >>= srcBits;
            bitsMerged -= srcBits;
        }
        return true;
    }
    return false;
}

bool read_elrs_data(int serial_fd, std::vector<uint16_t> &elrs_data) {
    static std::vector<uint8_t> data_buffer(512, 0); // 增大缓冲区
    static size_t buffer_pos = 0;
    uint8_t buffer[256];

    int bytes_read = read(serial_fd, buffer, sizeof(buffer));
    if (bytes_read > 0) {
        // 将读取到的数据追加到缓冲区
        for (int i = 0; i < bytes_read; ++i) {
            if (buffer_pos < data_buffer.size()) {
                data_buffer[buffer_pos++] = buffer[i];
            } else {
                // 缓冲区溢出，丢弃最旧数据
                std::rotate(data_buffer.begin(), data_buffer.begin() + 1, data_buffer.end());
                data_buffer.back() = buffer[i];
            }
        }

        // 查找以0xc8开头的26字节数据
        for (size_t i = 0; i <= buffer_pos - 26; ++i) {
            if (data_buffer[i] == 0xc8) {
                // 确保索引范围有效
                if (i + 26 <= buffer_pos) {
                    uint8_t elrs_data_temp[26];
                    std::copy(data_buffer.begin() + i, data_buffer.begin() + i + 26, elrs_data_temp);

                    // 删除已处理的数据
                    buffer_pos -= (i + 26);
                    std::rotate(data_buffer.begin(), data_buffer.begin() + i + 26, data_buffer.begin() + buffer_pos);

                    if (parse_elrs_data(elrs_data_temp, elrs_data)) {
                        return true;
                    }
                } else {
                    // 如果数据不足，退出本次处理
                    break;
                }
            }
        }
    }
    return false;
}

// ROS 2节点类
class ElrsReceiverNode : public rclcpp::Node {
public:
    ElrsReceiverNode() : Node("rc_node") {
        // 创建发布者
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("rc_control", 10);

        // 打开串口
        serial_fd_ = init_serial("/dev/ttyS1", B460800);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
        }

        // 定时器，每10ms发布一次数据
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ElrsReceiverNode::timer_callback, this));
    }

    ~ElrsReceiverNode() {
        close(serial_fd_);
    }

private:
    void timer_callback() {
        std::vector<uint16_t> elrs_data(16);
        if (read_elrs_data(serial_fd_, elrs_data)) {
            // 转换数据为 int16_t 类型
            std::vector<int16_t> msg_data(elrs_data.begin(), elrs_data.end());
            
            std_msgs::msg::Int16MultiArray msg;
            msg.data = msg_data;

            publisher_->publish(msg);
        } else {
        }
    }

    int serial_fd_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ElrsReceiverNode>());
    rclcpp::shutdown();
    return 0;
}