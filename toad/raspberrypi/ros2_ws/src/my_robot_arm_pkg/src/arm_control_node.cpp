// subscriber노드(특정 단계 msg받으면 해당 함수에 맞는 시리얼 내보내는 노드)

#include <rclcpp/rclcpp.hpp>    //ROS2의 기본 node 클래스 정의한 헤더파일 (rclcpp::Node를 상속해 나만의 노드에 필수)
#include <std_msgs/msg/string.hpp>  //pub정보 - string
#include <serial/serial.h>
#include <vector>
#include <string>
#include <unordered_map>

class ArmControlNode : public rclcpp::Node
{
public:
    ArmControlNode();

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg);
    void gripper_callback(const std_msgs::msg::String::SharedPtr msg);
    void send_joint_angles(const std::vector<int> &angles);
    void send_gripper_angle(int angle);
    std::string format_joint_angles(const std::vector<int>& angles);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr gripper_subscription_;
    serial::Serial serial_;
    std::unordered_map<std::string, std::vector<int>> joint_angles_map_;
    std::unordered_map<std::string, std::vector<int>> gripper_angles_map_;
};

//생성자 구현
ArmControlNode::ArmControlNode()
: Node("arm_control_node")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "arm_command",10,
        std::bind(&ArmControlNode::topic_callback, this, std::placeholders::_1));

    gripper_subscription_=this ->create_subscription<std_msgs::msg::String>(
        "arm_command" , 10,
        std::bind(&ArmControlNode::gripper_callback, this, std::placeholders::_1));
    

    try{
        serial_.setPort("/dev/ttyUSB0");
        serial_.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(to);
        serial_.open();
    } catch (const serial::IOException &e) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open port");
    }

    //각 pose 이름에 대응하는 조인트 각도 설정
    joint_angles_map_ = {
        {"pose1", {500,2050,2100,2400,2000,1800}},  // 기본위치
        {"pose2", {500,1800,1900,2400,1800,2000}},  // pick 위치
        {"pose3", {600,1750,1750,2400,1800,2000}},  // pick up 위치
        {"pose4", {400,2400,1400,2400,1500,2000}},  // 이송 위치
        {"pose5", {800,1900,2000,2400,2000,2000}},  // 쓰레기통 위치
    };

    //gripper각도 설정
    gripper_angles_map_ = {
        {"gripper_open", {2300}},  // 그리퍼 열림
        {"gripper_close", {500}},  // 그리퍼 닫힘
    };
            
}

std::string ArmControlNode::format_joint_angles(const std::vector<int>& angles) {
    std::string result;
    for (size_t i = 0; i< angles.size(); ++i){
        result += std::to_string(i+1) + ":" + std::to_string(angles[i]);
        if (i != angles.size() - 1)
            result += ",";
    }
    return result;

}

void ArmControlNode::topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
    auto it = joint_angles_map_.find(msg->data);
    if (it != joint_angles_map_.end()) {
        send_joint_angles(it->second);
        RCLCPP_INFO(this->get_logger(), "Sent angles for %s", msg->data.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown command: %s", msg->data.c_str());
    }
}

void ArmControlNode::gripper_callback(const std_msgs::msg::String::SharedPtr msg)
{
    auto it = gripper_angles_map_.find(msg->data);
    if (it != gripper_angles_map_.end()) {
        int angle = it -> second[0];
        send_gripper_angle(angle);
        RCLCPP_INFO(this->get_logger(), "sent gripper angle for mode '%s': %d", msg->data.c_str(), angle);
    } else {
        RCLCPP_WARN(this->get_logger(), "Unknown gripper command: %s", msg->data.c_str());
    }
}

void ArmControlNode::send_joint_angles(const std::vector<int> &angles)
{
    if (!serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port not open");
        return;
    }

    std::string packet = format_joint_angles(angles) + "\n"; // start delimiter
    serial_.write(packet);
}

void ArmControlNode::send_gripper_angle(int angle)
{
    if(!serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "serial port not open");
        return;
    }

    std::string packet = "7:" + std::to_string(angle) + "\n"; 
    serial_.write(packet);
}     //	#G,2000\n 이렇게 시리얼로 내보냄.

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmControlNode>());
    rclcpp::shutdown();
    return 0;
}


/* 
"pose1"	기본 위치로 이동
"pose2"	픽업 대상 위치로 이동
"pose3"	집은 후 들어올리는 위치
"pose4"	물체 이송 위치
"pose5"	쓰레기통에 떨어트릴 위치
"gipper_state1"	그리퍼 열림
*/





// //거리값 수신(pub으로부터)

// class ArmControlNode : public rclcpp::Node
// {
// public:
//     ArmControlNode() : Node("arm_control_node")
//     {
//         // subscriber 생성
//         distance_subscriber_ = this -> create_subscription<std_msgs::msg::Float32>(
//             "target_distance",10,   //토픽이름:target_distance
//             std::bind(&ArmControlNode::distanceCallback, this, std::placeholders::_1));
        
//     }

// private:
//     //callback함수정의
//     void distanceCallback(const std_msgs::msg::Float32::sharedPtr msg)
//     {
//         float distance = msg->data;
//         RCLCPP_INFO(this->get_logger(), "Received distance: %.2f", distance);

//         //todo(해야할거):IK solver에 넘기기, 시리얼 송신은 이후 단계
//     }    

//     rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr distance_subscriber_;
// };

// //IK 계산

// //Joint 1~6 값 시리얼로 내보내기

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<ArmControlNode>();

//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



// ------------------------------------------------------------------------
// 토픽메세지가 오면 콜백함수 실행 -> 계속 반복 노드 실행(spin)
// 아무 토픽메세지 없으면 대기
//반복해서 메세지 받는 구조를 이벤트 루프라고 함.
//spin이 종료됬을 때 shutdown실행.spin - 대기 상태가 되는 것.


/* 
[cpp지식]

-std란 C++표준라이브러리-

std::string 문자열타입
std::vector 동적 배열
std::cout 출력 스트림
std::make_shared 스마트 포인터 생성기
등등

::는 스코프 연산자 - 어디 소속인지 알려주는 기호 (주소표 역할)
shared_ptr 자동 메모리 관리되는 똑똑한 포인터
make_shared<T>() 스마트포인터를 안전하게 생성하는 함수 - 메모리를 자동으로 해제해주는 포인터. 내가 delete안해도 됌. 

ros2에서도 거의 모든 노드는 shared_ptr로 관리됨.

*/