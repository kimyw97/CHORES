#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <serial/serial.h>
#include <iostream>
#include <string>
#include "toad_auto_drive/msg/toad_drive_msg.hpp"
#include <regex>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

class ToadAutoDrivePubNode : public rclcpp::Node {
public:
    ToadAutoDrivePubNode() : Node("auto_drive_node") {
        publisher_ = this->create_publisher<toad_auto_drive::msg::ToadDriveMsg>("/auto_drive", 10);
        right_motor_ = 0.0;
        left_motor_ = 0.0;
        edge_detect = false;
        midle_clean = false;
        is_turn = false;
        cnt = 0;
        midle_cnt = 0;

        net_ = cv::dnn::readNetFromONNX("./best.onnx");
        net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

   
        try {
            serial_.setPort("/dev/ttyUSB0");
            serial_.setBaudrate(115200);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            serial_.setTimeout(to);
            serial_.open();
        } catch (serial::IOException &e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port.");
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
        }

        cap_.open(0);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open one or both cameras");
            throw std::runtime_error("camera open failed");
        }

        cv::namedWindow("cap", cv::WINDOW_AUTOSIZE);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ToadAutoDrivePubNode::check_go_possible_, this)
        );
    }

    ~ToadAutoDrivePubNode() {
        cv::destroyWindow("cap");
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    float right_motor_;
    float left_motor_;
    bool edge_detect;
    bool midle_clean;
    bool is_turn;
    rclcpp::Publisher<toad_auto_drive::msg::ToadDriveMsg>::SharedPtr publisher_;
    serial::Serial serial_;
    int cnt;
    int midle_cnt;
    cv::VideoCapture cap_;
    cv::dnn::Net net_;

    void check_go_possible_() {
        auto msg = toad_auto_drive::msg::ToadDriveMsg();
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Captured empty frame");
            return;
        }

        if (detect_trash(frame)) {
            msg.trash_detected = true;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        if (msg.trash_detected) {
            RCLCPP_INFO(this->get_logger(), "쓰레기 발견");
            msg.left_motor = 0.0;
            msg.right_motor = 0.0;
            publisher_->publish(msg);
            return;
        }
        
        cv::imshow("cap", frame);
        cv::waitKey(1);

        if (serial_.available()) {
            std::string data = serial_.readline(1024, "\n");
            RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());

            bool L = false, R = false;
            std::regex s1_regex("s1(\\d+)");
            std::regex s2_regex("s2(\\d+)");
            std::smatch match;
            if (std::regex_search(data, match, s1_regex) && match.size() > 1) {
                L = (std::stoi(match[1]) == 1);
            }
            if (std::regex_search(data, match, s2_regex) && match.size() > 1) {
                R = (std::stoi(match[1]) == 1);
            }

            if (edge_detect && !midle_clean) {
                if (!L && !R) {
                    msg = turn_left(msg);
                } else if (L && R) {
                    msg = go_straight(msg);
                    edge_detect = false;
                    cnt++;
                    if (cnt == 4) {
                        cnt = 0;
                        midle_clean = true;
                    }
                } else if (L && !R) {
                    msg = turn_left(msg);
                } else if (!L && R) {
                    msg = turn_right(msg);
                }
            } else if (!edge_detect && !midle_clean) {
                if (L && R) {
                    msg = go_straight(msg);
                } else if (L && !R) {
                    msg = turn_left(msg);
                } else if (!L && R) {
                    msg = turn_right(msg);
                } else if (!L && !R) {
                    msg = turn_left(msg);
                    edge_detect = true;
                }
            } else if (!edge_detect && midle_clean) {
                if (midle_cnt == 0) {
                    msg = go_straight(msg);
                    midle_cnt++;
                } else if (midle_cnt == 1) {
                    msg = turn_left(msg);
                    midle_cnt++;
                } else if (midle_cnt >= 2) {
                    if (L && R) {
                        msg = go_straight(msg);
                        midle_cnt++;
                    } else if (!L && !R) {
                        midle_cnt = 0;
                        midle_clean = false;
                        msg.left_motor = 0.0;
                        msg.right_motor = 0.0;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "Recive : left : %f, right : %f, %d", msg.left_motor, msg.right_motor, msg.trash_detected);
        publisher_->publish(msg);
        }

        
    }

    // ========== 유틸 함수 ==========
    toad_auto_drive::msg::ToadDriveMsg go_straight(toad_auto_drive::msg::ToadDriveMsg msg) {
        msg.left_motor = 30.0;
        msg.right_motor = 30.0;
        return msg;
    }

    toad_auto_drive::msg::ToadDriveMsg turn_left(toad_auto_drive::msg::ToadDriveMsg msg) {
        msg.left_motor = -30.0;
        msg.right_motor = 30.0;
        return msg;
    }

    toad_auto_drive::msg::ToadDriveMsg turn_right(toad_auto_drive::msg::ToadDriveMsg msg) {
        msg.left_motor = 30.0;
        msg.right_motor = -30.0;
        return msg;
    }

    // 쓰레기 감지 함수
    bool detect_trash(const cv::Mat &frame) {
        if (frame.empty())
            return false;

        cv::Mat blob;
        cv::dnn::blobFromImage(frame, blob, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(), true, false);

        net_.setInput(blob);
        std::vector<cv::Mat> outputs;
        net_.forward(outputs, net_.getUnconnectedOutLayersNames());

        float conf_threshold = 0.5;
        float nms_threshold = 0.4;

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (int i = 0; i < outputs[0].rows; i++) {
            float confidence = outputs[0].at<float>(i, 4);
            if (confidence > conf_threshold) {
                cv::Mat scores = outputs[0].row(i).colRange(5, outputs[0].cols);
                cv::Point class_id_point;
                double max_class_score;
                minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
                if (max_class_score > conf_threshold) {
                    int center_x = static_cast<int>(outputs[0].at<float>(i, 0) * frame.cols);
                    int center_y = static_cast<int>(outputs[0].at<float>(i, 1) * frame.rows);
                    int width = static_cast<int>(outputs[0].at<float>(i, 2) * frame.cols);
                    int height = static_cast<int>(outputs[0].at<float>(i, 3) * frame.rows);
                    int left = center_x - width / 2;
                    int top = center_y - height / 2;

                    class_ids.push_back(class_id_point.x);
                    confidences.push_back(max_class_score);
                    boxes.emplace_back(left, top, width, height);
                }
            }
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

        if (!indices.empty()) {
            for (int idx : indices) {
                cv::rectangle(frame, boxes[idx], cv::Scalar(0, 255, 0), 2);
                cv::putText(frame, "cap", boxes[idx].tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
            return true;
        }

        return false;
    }
};

// 메인 함수
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToadAutoDrivePubNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

