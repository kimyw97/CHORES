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
        this->declare_parameter("port", "/dev/serial0");
        this->declare_parameter("baudrate", 115200);
        this->declare_parameter("wheel_base", 0.3);  // 바퀴 간 거리

        this->get_parameter("port", port_);
        this->get_parameter("baudrate", baudrate_);
        this->get_parameter("wheel_base", wheel_base_);

        open_serial();

        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToSerial::cmdvel_callback, this, std::placeholders::_1)
        );
    }

    ~CmdVelToSerial() {
        if (serial_fd_ >= 0) {
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
        tcgetattr(serial_fd_, &tty);
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

        tcsetattr(serial_fd_, TCSANOW, &tty);
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

        if (serial_fd_ >= 0) {
            write(serial_fd_, command.c_str(), command.length());
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
