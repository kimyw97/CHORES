#include "rclcpp/rclcpp.hpp"
#include "toad_auto_drive/msg/toad_teleop_msg.hpp"
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>

class TeleopDriveNode : public rclcpp::Node
{
public:
    TeleopDriveNode() : Node("teleop_drive_node")
    {
        pub_ = this->create_publisher<toad_auto_drive::msg::ToadTeleopMsg>("/auto_drive", 10);
        RCLCPP_INFO(this->get_logger(), "Teleop node started. Use WASD to control, Q to quit.");

        configureTerminal();
        run();
        restoreTerminal();
    }

private:
    rclcpp::Publisher<toad_auto_drive::msg::ToadTeleopMsg>::SharedPtr pub_;


    struct termios old_tio_;

    void configureTerminal()
    {
        struct termios new_tio;
        tcgetattr(STDIN_FILENO, &old_tio_);
        new_tio = old_tio_;
        new_tio.c_lflag &= ~(ICANON | ECHO); // 비표준 입력, 에코 끄기
        tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    }

    void restoreTerminal()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_tio_);
    }

    void run()
    {
        char c;
        int speed = 25;

        while (rclcpp::ok())
        {
            c = getchar();
            auto msg = toad_auto_drive::msg::ToadTeleopMsg();

            if (c == 'w') {        // 앞으로
                msg.left_motor = speed;
                msg.right_motor = speed;
            }
            else if (c == 's') {   // 뒤로
                msg.left_motor = -speed;
                msg.right_motor = -speed;
            }
            else if (c == 'a') {   // 좌회전
                msg.left_motor = -speed;
                msg.right_motor = speed;
            }
            else if (c == 'd') {   // 우회전
                msg.left_motor = speed;
                msg.right_motor = -speed;
            }
            else if (c == 'x') {   // 정지
                msg.left_motor = 0;
                msg.right_motor = 0;
            }
            else if (c == 'q') {   // 종료
                break;
            }
            else {
                continue;
            }

            pub_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Sent: left: %d, right: %d", msg.left_motor, msg.right_motor);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::make_shared<TeleopDriveNode>();  // run()은 생성자에서 호출
    rclcpp::shutdown();
    return 0;
}
