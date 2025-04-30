#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

class StereoLitterNode : public rclcpp::Node {
public:
  StereoLitterNode() : Node("stereo_litter_direct_node") {
    pub_ =
        this->create_publisher<std_msgs::msg::Float32>("/detected_litter", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&StereoLitterNode::process_frame, this));

    capL_.open(0);
    capR_.open(1);

    if (!capL_.isOpened() || !capR_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "???? ? ? ????.");
      rclcpp::shutdown();
    }

    stereo_ = cv::StereoSGBM::create(0, 64, 5);
    Q_ = (cv::Mat_<double>(4, 4) << 1, 0, 0, -320, 0, 1, 0, -240, 0, 0, 0, 700,
          0, 0, 1.0 / 50, 0);

    net_ = cv::dnn::readNet("yolov8n.onnx");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture capL_, capR_;
  cv::Ptr<cv::StereoSGBM> stereo_;
  cv::Mat Q_;
  cv::dnn::Net net_;

  void process_frame() {
    cv::Mat frameL, frameR;
    capL_ >> frameL;
    capR_ >> frameR;

    if (frameL.empty() || frameR.empty())
      return;

    // YOLO ??
    cv::Mat blob = cv::dnn::blobFromImage(
        frameL, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);
    net_.setInput(blob);
    std::vector<cv::Mat> outputs;
    net_.forward(outputs, net_.getUnconnectedOutLayersNames());

    for (int i = 0; i < outputs[0].rows; ++i) {
      float conf = outputs[0].at<float>(i, 4);
      if (conf < 0.5)
        continue;

      int class_id =
          std::max_element(outputs[0].ptr<float>(i) + 5,
                           outputs[0].ptr<float>(i) + outputs[0].cols) -
          (outputs[0].ptr<float>(i) + 5);

      if (class_id != YOUR_LITTER_CLASS_ID)
        continue;

      int cx = static_cast<int>(outputs[0].at<float>(i, 0) * frameL.cols);
      int cy = static_cast<int>(outputs[0].at<float>(i, 1) * frameL.rows);

      // ?? ??
      cv::Mat grayL, grayR;
      cv::cvtColor(frameL, grayL, cv::COLOR_BGR2GRAY);
      cv::cvtColor(frameR, grayR, cv::COLOR_BGR2GRAY);
      cv::Mat disparity = stereo_->compute(grayL, grayR);
      cv::Mat depth;
      cv::reprojectImageTo3D(disparity, depth, Q_);
      float z = depth.at<cv::Vec3f>(cy, cx)[2];

      if (z > 0 && z < 300) {
        std_msgs::msg::Float32 msg;
        msg.data = z;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Detected litter at %.2f cm", z);
      }
      break;
    }
  }
};