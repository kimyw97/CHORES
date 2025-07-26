#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <sstream>

class CmdVelToSerial : public rclcpp::Node {
public:
    CmdVelToSerial()
    : Node("cmdvel_to_serial")
    {
        this->declare_parameter("port", "/dev/ttyAMA0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("wheel_base", 0.3);  // 바퀴 간 거리

        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("wheel_base", wheel_base_);

        RCLCPP_INFO(this->get_logger(), "Serial port: %s", port_.c_str());
        RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);
        RCLCPP_INFO(this->get_logger(), "Wheel base: %.3f", wheel_base_);

        open_serial();

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToSerial::cmdvel_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /cmd_vel");
    }

    ~CmdVelToSerial() {
        if (serial_fd_ >= 0) {
            RCLCPP_INFO(this->get_logger(), "Closing serial port.");
            close(serial_fd_);
        }
    }

private:
    void open_serial() {
        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", port_.c_str());
            return;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get serial attributes");
            return;
        }

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set serial attributes");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
    }

    void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double v = msg->linear.x;
        double w = msg->angular.z;
    
        double left_speed = v - (wheel_base_ / 2.0) * w;
        double right_speed = v + (wheel_base_ / 2.0) * w;
    
        int l = static_cast<int>(left_speed * 100);
        int r = static_cast<int>(right_speed * 100);
    
    
        std::stringstream ss;
        ss << "L" << l << "R" << r << "\n";
        std::string command = ss.str();
    
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel: linear=%.2f, angular=%.2f", v, w);
        RCLCPP_INFO(this->get_logger(), "Mapped speeds: L=%d, R=%d", l, r);
        RCLCPP_INFO(this->get_logger(), "Sending: %s", command.c_str());
    
        if (serial_fd_ >= 0) {
            ssize_t bytes_written = write(serial_fd_, command.c_str(), command.length());
            if (bytes_written < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port.");
            }
        }
    }
    

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    std::string port_;
    int baudrate_;
    double wheel_base_;
    int serial_fd_ = -1;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToSerial>());
    rclcpp::shutdown();
    return 0;
}
