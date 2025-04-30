#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>

class StereoLitterNode : public rclcpp::Node {
public:
  StereoLitterNode()
  : Node("stereo_litter_direct_node")
  {
    RCLCPP_INFO(this->get_logger(), "노드 초기화됨. 카메라와 모델을 준비 중입니다...");

    pub_ = this->create_publisher<std_msgs::msg::Float32>("/detected_litter", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&StereoLitterNode::process_frame, this)
    );

    capL_.open(0);
    capR_.open(2);

    if (!capL_.isOpened() || !capR_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "카메라를 열 수 없습니다.");
      rclcpp::shutdown();
    }

    stereo_ = cv::StereoSGBM::create(0, 64, 5);
    Q_ = (cv::Mat_<double>(4, 4) << 1, 0, 0, -320,
                                    0, 1, 0, -240,
                                    0, 0, 0, 700,
                                    0, 0, 1.0/50, 0);

    std::string pkg_share = "/home/kyw/temp_cow_repo/cow/raspberrypi/ros2_ws/install/stereo_litter_direct_node/share/stereo_litter_direct_node";
    std::string onnx_path = pkg_share + "/models/best.onnx";
    std::string classes_path = pkg_share + "/models/classes.txt";

    net_ = cv::dnn::readNet(onnx_path);

    std::ifstream ifs(classes_path);
    std::string line;
    while (std::getline(ifs, line)) class_names_.push_back(line);

    RCLCPP_INFO(this->get_logger(), "모델과 클래스 로드 완료. 거리 계산 시작!");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture capL_, capR_;
  cv::Ptr<cv::StereoSGBM> stereo_;
  cv::Mat Q_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;

  void process_frame() {
    cv::Mat frameL, frameR;
    capL_ >> frameL;
    capR_ >> frameR;

    if (frameL.empty() || frameR.empty()) {
      RCLCPP_WARN(this->get_logger(), "프레임을 읽을 수 없습니다. 카메라 상태를 확인하세요.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "프레임 수신 완료. 객체 탐지 중...");

    cv::Mat blob = cv::dnn::blobFromImage(frameL, 1.0/255.0, cv::Size(640,640), cv::Scalar(), true, false);
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    for (int i = 0; i < outputs[0].rows; ++i) {
      float conf = outputs[0].at<float>(i, 4);
      if (conf < 0.5) continue;

      int class_id = std::max_element(outputs[0].ptr<float>(i) + 5,
                                      outputs[0].ptr<float>(i) + outputs[0].cols) -
                     (outputs[0].ptr<float>(i) + 5);

      if (class_id < 0 || static_cast<size_t>(class_id) >= class_names_.size()) continue;

      std::string name = class_names_[class_id];
      if (!(name.find("litter") != std::string::npos ||
            name.find("trash") != std::string::npos ||
            name.find("waste") != std::string::npos ||
            name.find("wrapper") != std::string::npos ||
            name.find("bag") != std::string::npos ||
            name.find("packet") != std::string::npos ||
            name.find("straw") != std::string::npos ||
            name.find("cup") != std::string::npos ||
            name.find("container") != std::string::npos ||
            name.find("foil") != std::string::npos ||
            name.find("tube") != std::string::npos ||
            name.find("piece") != std::string::npos ||
            name.find("Shoe") != std::string::npos ||
            name.find("Cigarette") != std::string::npos))
        continue;

      int cx = static_cast<int>(outputs[0].at<float>(i, 0) * frameL.cols);
      int cy = static_cast<int>(outputs[0].at<float>(i, 1) * frameL.rows);

      cv::Mat grayL, grayR;
      cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
      cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);

      cv::Mat disparity;
      stereo_->compute(grayL, grayR, disparity);
      cv::Mat depth;
      cv::reprojectImageTo3D(disparity, depth, Q_);

      float z = depth.at<cv::Vec3f>(cy, cx)[2];

      if (z > 0 && z < 300) {
        std_msgs::msg::Float32 msg;
        msg.data = z;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🧹 %s 감지됨! 거리: %.2f cm", name.c_str(), z);
      }
      break;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoLitterNode>());
  rclcpp::shutdown();
  return 0;
}

