/*ros2 c++ 관련 기본 헤더*/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
/* 벡터 사용을 위한 STL */
#include <vector>
/* 시리얼 통신을 위한 POSIX 관련 헤더 */
#include <termios.h>     //터미널 설정용
#include <unistd.h>      //write, open, close 등
#include <fcntl.h>       //파일 제어 옵션
#include <cstring>       //memset 등 문자열 처리


// 시리얼 통신을 위한 POSIX 관련 헤더
class IKSerialNode : public rclcpp::Node
{
public:
    IKSerialNode() : Node("ik_serial_node")                        //생성자에서 초기화 작업 수행
    {
        // 시리얼 포트 설정
        serial_fd_ = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "시리얼 포트를 열 수 없습니다.");
            rclcpp::shutdown();
        } else {
            configureSerial(serial_fd_);
            RCLCPP_INFO(this->get_logger(), "시리얼 포트 열림.");
        }

         // 거리값 구독 설정: "/target_distance" 토픽, 큐 크기 10
         subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/target_distance", 10,
            std::bind(&IKSerialNode::distanceCallback, this, std::placeholders::_1));
    }

     // 소멸자에서 시리얼 포트 닫기
     ~IKSerialNode()
     {
         if (serial_fd_ != -1) {
             close(serial_fd_);
         }
     }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;  // 거리값 구독 객체
    int serial_fd_;  // 시리얼 파일 디스크립터

    // 시리얼 통신 설정 함수
    void configureSerial(int fd)
    {
        struct termios options;
        tcgetattr(fd, &options);             // 현재 시리얼 설정 가져오기
        cfsetispeed(&options, B115200);      // 입력 속도 설정
        cfsetospeed(&options, B115200);      // 출력 속도 설정
        options.c_cflag |= (CLOCAL | CREAD); // 로컬 연결, 읽기 허용
        options.c_cflag &= ~CSIZE;           // 데이터 크기 비트 초기화
        options.c_cflag |= CS8;              // 8비트 데이터
        options.c_cflag &= ~PARENB;          // 패리티 비트 사용 안 함
        options.c_cflag &= ~CSTOPB;          // 1 스톱 비트
        options.c_cflag &= ~CRTSCTS;         // 하드웨어 흐름제어 비활성화
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 비정규 모드 (즉시 입력 처리)
        options.c_iflag &= ~(IXON | IXOFF | IXANY);         // 소프트웨어 흐름 제어 끄기
        options.c_oflag &= ~OPOST;           // 출력 후처리 비활성화
        tcsetattr(fd, TCSANOW, &options);    // 위 설정을 즉시 적용
    }
  
    // 거리값 수신 시 호출되는 콜백 함수
    void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        double distance = msg->data;  // 거리값 읽기
  
        // 거리 기반 간단한 XYZ 좌표 설정 (z 고정)
        std::vector<double> joint_angles = computeIK(distance, 0.0, 0.2);
  
        // 조인트 각도를 시리얼로 전송
        sendJointAngles(joint_angles);
    }
  
    // 거리 -> XYZ -> 조인트 각도로 바꾸는 간단한 더미 IK 함수
    std::vector<double> computeIK(double x, double y, double z)
    {
        std::vector<double> angles(6, 0.0);  // 6개 조인트 초기화
        angles[0] = x;         // 1번 조인트는 거리값에 비례
        angles[1] = y + 0.1;   // 임의 조정값
        angles[2] = z + 0.1;
        angles[3] = 0.5;
        angles[4] = 0.2;
        angles[5] = -0.1;
        return angles;         // 총 6개 각도 반환
    }
  
    // 조인트 각도 값을 시리얼로 전송
    void sendJointAngles(const std::vector<double>& angles)
    {
        if (serial_fd_ == -1) return;  // 포트가 안 열렸으면 무시
  
        uint8_t buffer[32];  // 전송 버퍼
        buffer[0] = 0xFF;    // 헤더 바이트 (동기화용)
  
        // 각 조인트 각도를 정수형으로 변환해 2바이트씩 저장
        for (size_t i = 0; i < 6; ++i) {
            int16_t val = static_cast<int16_t>(angles[i] * 100.0);  // 소수점 보존 위해 100배
            buffer[1 + i * 2] = (val >> 8) & 0xFF;  // 상위 바이트
            buffer[2 + i * 2] = val & 0xFF;         // 하위 바이트
        }
  
        // 전체 13바이트 전송 (0xFF + 6조인트 * 2바이트)
        write(serial_fd_, buffer, 13);
    }
};
  
// 노드 실행 함수 (main 함수)
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);                               // ROS2 초기화
    rclcpp::spin(std::make_shared<IKSerialNode>());         // 노드 실행
    rclcpp::shutdown();                                     // ROS2 종료
    return 0;
}








// /*거리값 토픽 구독*/
// rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
// /*콜백 함수 안에서 역기구학 계산*/
// void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg)
// {
//     double distance = msg->data;

//     // 1. 거리 기반 좌표 변환
//     double x = distance;
//     double y = 0.0;
//     double z = 0.2;

//     // 2. 역기구학 계산 (예: MoveIt, custom IK)
//     std::vector<double> joint_angles = computeIK(x, y, z);

//     // 3. 시리얼 전송
//     sendToSerial(joint_angles);
// }



// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);
//   //ROS2 노드 초기화
//   // NodeOptions에서 파라미터 자동선언을 켜놓음 (Moveit이 내부에서 declare없이 쓰는 경우 있음)

//   // We spin up a SingleThreadedExecutor for the current state monitor to get information
//   // about the robot's state.
//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(move_group_node);
//   std::thread([&executor]() { executor.spin(); }).detach();
//   //move_group_node를 실행시키는 executor 생성
//   //spin()을 별도 스레드로 실행해서 노드가 동작하는 동안 메인 스레드는 계속 아래 코드 실행 가능
